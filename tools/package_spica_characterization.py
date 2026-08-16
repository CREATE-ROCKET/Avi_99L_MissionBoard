#!/usr/bin/env python3
"""Strict-validated V5 capturesをSpica引渡しpackageへ梱包する。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile

from verify_motor_id_characterization import (
    ValidationError,
    build_golden_fixture,
    verify_file,
)


STAGES = ("FV", "FH_positive", "FH_negative", "M0")
RATES = (1000,)


class PackageError(RuntimeError):
    pass


def _reject_json_constant(value: str) -> object:
    raise ValueError(f"non-finite JSON number {value}")


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _json_text(value: object) -> str:
    try:
        return json.dumps(
            value, ensure_ascii=False, indent=2, allow_nan=False
        ) + "\n"
    except (TypeError, ValueError) as error:
        raise PackageError(f"value is not strict JSON: {error}") from error


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path) -> dict[str, object]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
            object_pairs_hook=_unique_json_object,
        )
    except (OSError, UnicodeError, ValueError) as error:
        raise PackageError(f"cannot read JSON object {path}: {error}") from error
    if not isinstance(value, dict):
        raise PackageError(f"JSON root must be an object: {path}")
    return value


def _parse_uart(values: list[str]) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for value in values:
        stage, separator, path = value.partition("=")
        if not separator or stage not in STAGES or stage in result:
            raise PackageError(
                f"--uart must be a unique STAGE=PATH entry, got {value!r}"
            )
        result[stage] = Path(path)
    if set(result) != set(STAGES):
        raise PackageError(
            "one --uart STAGE=PATH entry is required for every stage"
        )
    return result


def _manifest_entry(
    path: Path, root: Path, **metadata: object
) -> dict[str, object]:
    return {
        "path": path.relative_to(root).as_posix(),
        "sha256": _sha256(path),
        **metadata,
    }


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("x", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(
            output,
            fieldnames=fields,
            extrasaction="raise",
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


def _csv_rows(
    captures: list[tuple[Path, dict[str, object]]],
) -> tuple[
    list[dict[str, object]],
    list[dict[str, object]],
    list[dict[str, object]],
]:
    epochs: list[dict[str, object]] = []
    samples: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    for package_path, capture in captures:
        previous: tuple[object, object, object] | None = None
        for epoch in capture["epochs"]:
            raw_samples = epoch["raw_samples"]
            epoch_row = {
                key: value
                for key, value in epoch.items()
                if key != "raw_samples"
            }
            epoch_row["source_path"] = package_path.as_posix()
            epochs.append(epoch_row)
            for sample in raw_samples:
                if sample is None:
                    continue
                samples.append(
                    {
                        "source_path": package_path.as_posix(),
                        "sequence": epoch["sequence"],
                        "epoch_index": epoch["epoch_index"],
                        **sample,
                    }
                )
            current = (
                epoch["profile_phase_value"],
                epoch["episode_id"],
                epoch["abort_reason_value"],
            )
            if current != previous:
                events.append(
                    {
                        "source_path": package_path.as_posix(),
                        "sequence": epoch["sequence"],
                        "epoch_index": epoch["epoch_index"],
                        "timestamp_us": epoch["epoch_start_timestamp_us"],
                        "profile_phase": epoch["profile_phase"],
                        "episode_id": epoch["episode_id"],
                        "abort_reason": epoch["abort_reason"],
                    }
                )
                previous = current
    return epochs, samples, events


def package(
    input_directory: Path,
    output_directory: Path,
    conditions_path: Path,
    operator_label: str,
    uart_paths: dict[str, Path],
    include_csv: bool,
) -> Path:
    if output_directory.exists():
        raise PackageError(f"output already exists: {output_directory}")
    if not operator_label or any(
        ord(character) < 0x20 for character in operator_label
    ):
        raise PackageError("operator label must be nonempty printable text")
    captures = sorted(input_directory.rglob("*.bin"))
    if not captures:
        raise PackageError("input directory contains no .bin capture")
    conditions = _load_object(conditions_path)
    for stage, path in uart_paths.items():
        if not path.is_file():
            raise PackageError(f"UART log for {stage} does not exist: {path}")

    decoded: list[tuple[Path, dict[str, object], Path]] = []
    session_id: str | None = None
    profile_seed: int | None = None
    campaign_contract: tuple[object, ...] | None = None
    coverage: dict[tuple[str, int], set[tuple[str, str]]] = {}
    for source in captures:
        capture = verify_file(source)
        if not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9._-]*\.bin", source.name
        ):
            raise PackageError(
                f"capture filename is not portable: {source.name!r}"
            )
        if not capture["integrity"]["strict_pass"]:
            raise PackageError(
                f"capture did not pass strict validation: {source}"
            )
        metadata = capture["metadata"]
        footer = capture["footer"]
        current_session = str(metadata["session_id"])
        if session_id is None:
            session_id = current_session
        elif current_session != session_id:
            raise PackageError("captures contain multiple session IDs")
        current_seed = int(metadata["profile_seed"])
        if profile_seed is None:
            profile_seed = current_seed
        elif current_seed != profile_seed:
            raise PackageError("captures contain multiple profile seeds")
        current_contract = tuple(
            metadata[key]
            for key in (
                "profile_contract_version",
                "pwm_frequency_hz",
                "firmware_sha",
                "avi_esp_libs_sha",
                "board_build_id",
                "total_reduction",
                "physical_limit_deg",
                "routine_guard_deg",
                "hard_abort_deg",
                "backlash_full_width_deg",
            )
        )
        if campaign_contract is None:
            campaign_contract = current_contract
        elif current_contract != campaign_contract:
            raise PackageError(
                "captures contain multiple firmware or physical contracts"
            )
        stage = str(metadata["stage"])
        rate = int(metadata["encoder_rate_hz"])
        if rate not in RATES:
            raise PackageError(
                f"new motor-ID package accepts 1000 Hz captures only, got {rate} Hz"
            )
        profile = str(metadata["run_kind"])
        coverage.setdefault((stage, rate), set()).add(
            (profile, str(footer["completion"]))
        )
        decoded.append(
            (Path(stage) / f"{rate}Hz" / source.name, capture, source)
        )

    assert session_id is not None
    if "session_id" in conditions and conditions["session_id"] != session_id:
        raise PackageError(
            "conditions.json session_id does not match capture headers"
        )
    if "profile_seed" in conditions and (
        isinstance(conditions["profile_seed"], bool)
        or not isinstance(conditions["profile_seed"], int)
        or conditions["profile_seed"] != profile_seed
    ):
        raise PackageError(
            "conditions.json profile_seed does not match capture headers"
        )
    conditions = {
        **conditions,
        "session_id": session_id,
        "profile_seed": profile_seed,
        "operator_label": operator_label,
        "encoder_rate_hz": 1000,
        "campaign_purpose": "motor_system_identification",
    }
    for stage in STAGES:
        outcomes = coverage.get((stage, 1000), set())
        if ("rate-check", "normal") not in outcomes:
            raise PackageError(
                f"{stage} lacks a normal 1000 Hz motor-ID rate-check"
            )
        if ("full", "normal") not in outcomes:
            raise PackageError(
                f"{stage} lacks a normal 1000 Hz full capture"
            )

    output_directory.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".99l-package-", dir=output_directory.parent
    ) as temporary:
        root = Path(temporary) / output_directory.name
        root.mkdir()
        manifest_entries: list[dict[str, object]] = []
        packaged_captures: list[tuple[Path, dict[str, object]]] = []
        for relative, capture, source in decoded:
            destination = root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            if destination.exists():
                raise PackageError(f"duplicate package path: {relative}")
            shutil.copyfile(source, destination)
            copied_capture = verify_file(destination)
            if (
                copied_capture["integrity"]["sha256"]
                != capture["integrity"]["sha256"]
            ):
                raise PackageError(
                    f"capture changed while it was being packaged: {source}"
                )
            capture = copied_capture
            metadata = capture["metadata"]
            epochs = capture["epochs"]
            manifest_entries.append(
                _manifest_entry(
                    destination,
                    root,
                    stage=metadata["stage"],
                    rate_hz=metadata["encoder_rate_hz"],
                    profile=metadata["run_kind"],
                    profile_seed=metadata["profile_seed"],
                    start_timestamp_us=(
                        epochs[0]["epoch_start_timestamp_us"]
                        if epochs
                        else metadata["epoch_zero_timestamp_us"]
                    ),
                    end_timestamp_us=(
                        epochs[-1]["epoch_end_timestamp_us"]
                        if epochs
                        else metadata["epoch_zero_timestamp_us"]
                    ),
                    firmware_sha=metadata["firmware_sha"],
                    avi_esp_libs_sha=metadata["avi_esp_libs_sha"],
                    completion=capture["footer"]["completion"],
                    completion_value=capture["footer"]["completion_value"],
                    rate_supported=capture["footer"]["rate_supported"],
                    unsupported_reason=capture["footer"][
                        "unsupported_reason"
                    ],
                    unsupported_reason_value=capture["footer"][
                        "unsupported_reason_value"
                    ],
                    rate_check_valid_epochs=capture["footer"][
                        "rate_check_valid_epochs"
                    ],
                    rate_check_total_epochs=capture["footer"][
                        "rate_check_total_epochs"
                    ],
                    first_error=capture["footer"]["first_error"],
                    shutdown_step_mask=capture["footer"][
                        "shutdown_step_mask"
                    ],
                )
            )
            packaged_captures.append((relative, capture))

        for stage, source in uart_paths.items():
            destination = root / stage / "uart.log"
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            manifest_entries.append(
                _manifest_entry(destination, root, stage=stage)
            )

        conditions_destination = root / "conditions.json"
        conditions_destination.write_text(
            _json_text(conditions), encoding="utf-8"
        )
        manifest_entries.append(
            _manifest_entry(conditions_destination, root)
        )

        aggregate_integrity = {
            "schema_version": 5,
            "session_id": session_id,
            "strict_pass": all(
                capture["integrity"]["strict_pass"]
                for _, capture in packaged_captures
            ),
            "captures": [
                {
                    "path": relative.as_posix(),
                    **{
                        key: value
                        for key, value in capture["integrity"].items()
                        if key != "source"
                    },
                }
                for relative, capture in packaged_captures
            ],
        }
        integrity_destination = root / "integrity.json"
        integrity_destination.write_text(
            _json_text(aggregate_integrity), encoding="utf-8"
        )
        manifest_entries.append(
            _manifest_entry(integrity_destination, root)
        )

        if include_csv:
            csv_directory = root / "optional_csv"
            csv_directory.mkdir()
            epoch_rows, sample_rows, event_rows = _csv_rows(
                packaged_captures
            )
            for name, rows in (
                ("epochs.csv", epoch_rows),
                ("raw_samples.csv", sample_rows),
                ("events.csv", event_rows),
            ):
                destination = csv_directory / name
                _write_csv(destination, rows)
                manifest_entries.append(
                    _manifest_entry(destination, root)
                )

        manifest = {
            "schema_version": 5,
            "session_id": session_id,
            "operator_label": operator_label,
            "files": sorted(
                manifest_entries, key=lambda item: str(item["path"])
            ),
        }
        manifest_destination = root / "manifest.json"
        manifest_destination.write_text(
            _json_text(manifest), encoding="utf-8"
        )

        sum_paths = [
            path
            for path in root.rglob("*")
            if path.is_file() and path.name != "SHA256SUMS"
        ]
        sums = "".join(
            f"{_sha256(path)}  {path.relative_to(root).as_posix()}\n"
            for path in sorted(sum_paths)
        )
        (root / "SHA256SUMS").write_text(sums, encoding="ascii")
        root.replace(output_directory)
    return output_directory


def self_test() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        inputs = root / "inputs"
        inputs.mkdir()
        for stage_value, stage in enumerate(STAGES, start=1):
            for run_kind in (1, 2):
                path = inputs / f"{stage}_1000_{run_kind}.bin"
                path.write_bytes(
                    build_golden_fixture(
                        stage_value=stage_value,
                        encoder_rate_hz=1000,
                        run_kind=run_kind,
                        sequence_base=(
                            stage_value * 100_000 + run_kind * 10_000
                        ),
                    )
                )
        conditions = root / "conditions.json"
        conditions.write_text(
            '{"fixture":"self-test"}\n', encoding="utf-8"
        )
        uart: dict[str, Path] = {}
        for stage in STAGES:
            path = root / f"{stage}.log"
            path.write_text("self-test\n", encoding="utf-8")
            uart[stage] = path
        output = root / "99l_characterization_golden-session"
        package(inputs, output, conditions, "self-test", uart, True)
        manifest = _load_object(output / "manifest.json")
        integrity = _load_object(output / "integrity.json")
        assert manifest["session_id"] == "golden-session"
        assert integrity["strict_pass"] is True
        assert all(
            "source" not in capture for capture in integrity["captures"]
        )
        assert (output / "optional_csv" / "raw_samples.csv").is_file()
        assert len(list(output.rglob("*.bin"))) == len(STAGES) * 2
        listed = (output / "SHA256SUMS").read_text(encoding="ascii")
        assert "manifest.json" in listed and "SHA256SUMS" not in listed

        bad_inputs = root / "bad-inputs"
        bad_inputs.mkdir()
        (bad_inputs / "capture\ninjected.bin").write_bytes(
            build_golden_fixture()
        )
        try:
            package(
                bad_inputs,
                root / "bad-package",
                conditions,
                "self-test",
                uart,
                False,
            )
        except PackageError as error:
            assert "not portable" in str(error)
        else:
            raise AssertionError("nonportable capture filename was accepted")

        for name, malformed in (
            ("duplicate.json", '{"profile_seed":1,"profile_seed":1}\n'),
            ("nan.json", '{"value":NaN}\n'),
        ):
            path = root / name
            path.write_text(malformed, encoding="utf-8")
            try:
                _load_object(path)
            except PackageError:
                pass
            else:
                raise AssertionError(
                    f"non-strict JSON was accepted: {name}"
                )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path)
    parser.add_argument("output", nargs="?", type=Path)
    parser.add_argument("--conditions", type=Path)
    parser.add_argument("--operator-label")
    parser.add_argument(
        "--uart", action="append", default=[], metavar="STAGE=PATH"
    )
    parser.add_argument("--csv", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.self_test:
            self_test()
            print("package_spica_characterization self-test: PASS")
            return 0
        if (
            args.input is None
            or args.output is None
            or args.conditions is None
            or args.operator_label is None
        ):
            raise PackageError(
                "input, output, --conditions, and --operator-label are required"
            )
        result = package(
            args.input,
            args.output,
            args.conditions,
            args.operator_label,
            _parse_uart(args.uart),
            args.csv,
        )
        print(f"package created: {result}")
        return 0
    except (OSError, ValidationError, PackageError, AssertionError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
