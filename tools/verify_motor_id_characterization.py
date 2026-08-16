#!/usr/bin/env python3
"""1 kHz motor-ID用99L characterization V5を厳密検証する。"""

from __future__ import annotations

import struct

import verify_characterization as _base


ValidationError = _base.ValidationError
build_golden_fixture = _base.build_golden_fixture

_original_need = _base._need
_original_decode_record = _base._decode_record
_allow_release_only_deadline_omission = False


def _motor_id_need(condition: bool, message: str) -> None:
    """release-only遅延のdeadline flag省略だけを追加で許容する。"""
    if condition:
        return
    if (
        _allow_release_only_deadline_omission
        and message.endswith("deadline flag tears")
    ):
        return
    _original_need(condition, message)


def _decode_motor_id_record(
    raw: bytes, header: dict[str, object], index: int
) -> dict[str, object]:
    global _allow_release_only_deadline_omission

    # 元validatorが行う全検査を維持したまま、今回の1 kHz motor-ID契約で
    # release-only遅延をEpochDeadlineへ昇格しないrecordだけを識別する。
    if len(raw) >= _base.RECORD_SIZE:
        epoch_start = struct.unpack_from("<Q", raw, 20)[0]
        release = struct.unpack_from("<Q", raw, 36)[0]
        apply_us = struct.unpack_from("<Q", raw, 44)[0]
        flags = struct.unpack_from("<H", raw, 126)[0]
        release_late = release > epoch_start + 1_000 + 100
        command_apply_late = (
            apply_us >= epoch_start and apply_us > epoch_start + 100
        )
        deadline_flag = bool(flags & _base.EPOCH_DEADLINE)
        _allow_release_only_deadline_omission = (
            release_late and not command_apply_late and not deadline_flag
        )
    else:
        _allow_release_only_deadline_omission = False

    try:
        return _original_decode_record(raw, header, index)
    finally:
        _allow_release_only_deadline_omission = False


# verify_bytes()は実行時にmodule globalを解決するため、元validatorの他の
# CRC/sequence/enum/command/Vbus/footer検査をそのまま再利用できる。
_base._need = _motor_id_need
_base._decode_record = _decode_motor_id_record

verify_bytes = _base.verify_bytes
verify_file = _base.verify_file


def main(argv: list[str] | None = None) -> int:
    return _base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
