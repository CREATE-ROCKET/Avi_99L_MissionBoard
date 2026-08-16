#!/usr/bin/env python3
"""99L characterization V5 binaryを厳密検証する。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import zlib


SCHEMA_VERSION = 5
HEADER_SIZE = 256
RECORD_SIZE = 320
FOOTER_SIZE = 192
HEADER_MAGIC = b"99LMCV5\0"
RECORD_MAGIC = b"EPV5"
FOOTER_MAGIC = b"99LEND5\0"

STAGES = {1: "FV", 2: "FH_positive", 3: "FH_negative", 4: "M0"}
RATES = {1000, 2000, 5000}
MOTOR_MODES = {0: "Coast", 1: "DriveIn1", 2: "DriveIn2", 3: "Brake"}
PHASES = {
    0: "Idle",
    1: "StationaryBaseline",
    2: "ZeroApproach",
    3: "PolarityCheck",
    4: "BreakawaySweep",
    5: "SustainedMotionSweep",
    6: "BoundedPulseGrid",
    7: "Coast",
    8: "ShortBrake",
    9: "PositiveToNegative",
    10: "NegativeToPositive",
    11: "BoundedPrbs",
    12: "BandLimitedNoise",
    13: "Chirp",
    14: "Recenter",
}
BRANCHES = {0: "None", 1: "FromPositive", 2: "FromNegative"}
GUARD_STATES = {0: "Allow", 1: "ForceCoast", 2: "Abort"}
ABORT_REASONS = {
    0: "None",
    1: "StopRequested",
    2: "EncoderInvalid",
    3: "PositionGuard",
    4: "Overshoot",
    5: "Timeout",
    6: "Deadline",
    7: "QueueFull",
    8: "WriterError",
    9: "MotorApplyError",
    10: "VbusInvalid",
    11: "SamplerError",
    12: "ValidationError",
    13: "StageError",
}
COMPLETIONS = {1: "normal", 2: "aborted", 3: "unsupported"}
UNSUPPORTED_REASONS = {
    0: "None",
    1: "TriggerCoalesced",
    2: "IncompleteEpoch",
    3: "InvalidRead",
    4: "DeadlineMiss",
    5: "QueueOverflow",
    6: "WriterFailure",
    7: "SensorHealth",
    8: "OperatorMarkedUnsupported",
}
RUN_KINDS = {1: "rate-check", 2: "full"}
QUALIFICATIONS = {0: "pending", 1: "accepted", 2: "unsupported"}

EPOCH_INCOMPLETE = 1 << 0
EPOCH_REPEATED = 1 << 1
EPOCH_SKIPPED = 1 << 2
EPOCH_INVALID = 1 << 3
EPOCH_LATE = 1 << 4
EPOCH_DEADLINE = 1 << 5
EPOCH_AGGREGATE_VALID = 1 << 6
EPOCH_STARTUP_INCOMPLETE = 1 << 7
KNOWN_EPOCH_FLAGS = (1 << 8) - 1
KNOWN_DIAGNOSTIC_FLAGS = (1 << 7) - 1
MAX_VBUS_AGE_US = 100_000
COMPLETE_SHUTDOWN_MASK = 0x3F
MAXIMUM_COMMAND_PERMILLE = 350
REQUIRED_FULL_PHASES = frozenset(range(1, 15))

COUNTER_NAMES = (
    "trigger_coalesced_missed",
    "pre_epoch",
    "repeated",
    "skipped",
    "invalid",
    "late_after_release",
    "startup_incomplete",
    "steady_state_incomplete",
    "consumer_deadline_miss",
    "raw_queue_overflow",
    "writer_queue_overflow",
)


class ValidationError(ValueError):
    """V5 wire contract違反。"""


def crc32(data: bytes | bytearray | memoryview) -> int:
    """CRC-32/ISO-HDLC（Python zlib互換）を返す。"""
    return zlib.crc32(data) & 0xFFFF_FFFF


def _need(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def _text(
    field: bytes,
    name: str,
    *,
    exact_hex: bool = False,
    allow_space: bool = True,
) -> str:
    nul = field.find(b"\0")
    if not exact_hex:
        _need(nul >= 0, f"{name} must be NUL terminated")
    payload = field if nul < 0 else field[:nul]
    if nul >= 0:
        _need(not any(field[nul + 1 :]), f"{name} has nonzero bytes after NUL")
    try:
        value = payload.decode("ascii")
    except UnicodeDecodeError as error:
        raise ValidationError(f"{name} is not ASCII") from error
    if exact_hex:
        _need(len(value) == len(field), f"{name} must contain {len(field)} hex digits")
        _need(all(character in "0123456789abcdefABCDEF" for character in value), f"{name} is not hexadecimal")
    else:
        minimum = 0x20 if allow_space else 0x21
        _need(bool(value) and all(minimum <= ord(character) <= 0x7E for character in value), f"{name} must be nonempty printable ASCII")
    return value


def _mode_matches(command: int, mode: int) -> bool:
    return (command > 0 and mode == 2) or (command < 0 and mode == 1) or (
        command == 0 and mode in (0, 3)
    )


def _decode_header(data: bytes) -> dict[str, object]:
    _need(len(data) >= HEADER_SIZE, "truncated V5 header")
    raw = data[:HEADER_SIZE]
    _need(raw[:8] == HEADER_MAGIC, "invalid V5 header magic")
    fields = struct.unpack_from("<HHHHBBHIIIIIfffffIQ", raw, 8)
    (
        schema,
        header_size,
        record_size,
        footer_size,
        stage,
        reserved_u8,
        consumer_hz,
        encoder_hz,
        profile_contract_version,
        profile_seed,
        pwm_hz,
        reset_reason,
        total_reduction,
        physical_limit_deg,
        routine_guard_deg,
        hard_abort_deg,
        backlash_full_width_deg,
        reserved_u32,
        epoch_zero_us,
    ) = fields
    _need(schema == SCHEMA_VERSION, f"unsupported schema {schema}")
    _need(header_size == HEADER_SIZE, f"header size is {header_size}, expected {HEADER_SIZE}")
    _need(record_size == RECORD_SIZE, f"record size is {record_size}, expected {RECORD_SIZE}")
    _need(footer_size == FOOTER_SIZE, f"footer size is {footer_size}, expected {FOOTER_SIZE}")
    _need(stage in STAGES, f"unknown stage {stage}")
    _need(consumer_hz == 1000, f"consumer rate must be 1000 Hz, got {consumer_hz}")
    _need(encoder_hz in RATES, f"unsupported encoder rate {encoder_hz}")
    _need(profile_contract_version == 1, f"unknown profile contract {profile_contract_version}")
    _need(profile_seed != 0, "profile seed must be nonzero")
    _need(pwm_hz > 0, "PWM frequency must be positive")
    _need(reserved_u8 == 0 and reserved_u32 == 0, "header reserved fields must be zero")
    numeric = (total_reduction, physical_limit_deg, routine_guard_deg, hard_abort_deg, backlash_full_width_deg)
    _need(all(math.isfinite(value) and value > 0 for value in numeric), "header physical metadata is invalid")
    _need(routine_guard_deg < hard_abort_deg < physical_limit_deg, "guard limits are not strictly nested")
    _need(epoch_zero_us != 0, "epoch zero timestamp must be nonzero")
    header_flags, expected_crc = struct.unpack_from("<II", raw, 248)
    _need(header_flags in (1, 2), f"unknown header flags 0x{header_flags:08x}")
    _need(crc32(raw[:252]) == expected_crc, "header CRC32 mismatch")
    return {
        "magic": "99LMCV5",
        "schema_version": schema,
        "header_bytes": header_size,
        "record_bytes": record_size,
        "footer_bytes": footer_size,
        "stage": STAGES[stage],
        "stage_value": stage,
        "consumer_rate_hz": consumer_hz,
        "encoder_rate_hz": encoder_hz,
        "profile_contract_version": profile_contract_version,
        "profile_seed": profile_seed,
        "pwm_frequency_hz": pwm_hz,
        "reset_reason": reset_reason,
        "total_reduction": total_reduction,
        "physical_limit_deg": physical_limit_deg,
        "routine_guard_deg": routine_guard_deg,
        "hard_abort_deg": hard_abort_deg,
        "backlash_full_width_deg": backlash_full_width_deg,
        "epoch_zero_timestamp_us": epoch_zero_us,
        "session_id": _text(raw[72:104], "session_id", allow_space=False),
        "firmware_sha": _text(raw[104:144], "firmware_sha", exact_hex=True),
        "avi_esp_libs_sha": _text(raw[144:184], "avi_esp_libs_sha", exact_hex=True),
        "board_build_id": _text(raw[184:248], "board_build_id", allow_space=False),
        "header_flags": header_flags,
        "run_kind": RUN_KINDS[1 if header_flags == 1 else 2],
        "header_crc32": expected_crc,
    }


def _decode_raw_slot(
    raw: bytes,
    base: int,
    expected_slot: int,
    expected_slots: int,
    epoch_start: int,
    epoch_end: int,
) -> dict[str, object] | None:
    slot_bytes = raw[base : base + 36]
    if not any(slot_bytes):
        return None
    generation, scheduled, capture, angle, diagnostics, read_result, valid, slot, reserved = struct.unpack_from(
        "<QQQHHiBBH", raw, base
    )
    _need(reserved == 0, "raw sample reserved field must be zero")
    _need(slot == expected_slot, f"raw sample slot is {slot}, expected {expected_slot}")
    _need(generation != 0 and scheduled != 0 and capture != 0, "present raw sample has a zero generation/timestamp")
    _need(epoch_start <= capture < epoch_end, "raw capture timestamp is outside its epoch")
    slot_start = epoch_start + expected_slot * 1000 // expected_slots
    slot_end = epoch_start + (expected_slot + 1) * 1000 // expected_slots
    _need(slot_start <= capture < slot_end, "raw capture timestamp is outside its slot")
    _need(scheduled <= capture, "raw capture timestamp precedes its schedule")
    _need(angle <= 0x3FFF, "raw angle exceeds the AS5047D 14-bit range")
    _need(diagnostics & ~KNOWN_DIAGNOSTIC_FLAGS == 0, "raw sample has unknown diagnostic flags")
    _need(valid in (0, 1), "raw valid field is not boolean")
    _need(valid == 0 or (read_result == 0 and diagnostics & 0x3F == 0), "valid raw sample reports an error")
    _need(valid == 1 or read_result != 0 or diagnostics & 0x3F != 0, "invalid raw sample has no failure evidence")
    return {
        "generation": generation,
        "scheduled_timestamp_us": scheduled,
        "capture_timestamp_us": capture,
        "angle_raw": angle,
        "diagnostic_flags": diagnostics,
        "read_result": read_result,
        "valid": bool(valid),
        "slot_index": slot,
    }


def _decode_record(raw: bytes, header: dict[str, object], index: int) -> dict[str, object]:
    _need(len(raw) == RECORD_SIZE, f"record {index} is truncated")
    _need(raw[:4] == RECORD_MAGIC, f"record {index} magic mismatch")
    _need(crc32(raw[:316]) == struct.unpack_from("<I", raw, 316)[0], f"record {index} CRC32 mismatch")
    sequence, epoch_index = struct.unpack_from("<QQ", raw, 4)
    epoch_start, epoch_end, release, apply_us, snapshot_us, vbus_us, generation = struct.unpack_from("<QQQQQQQ", raw, 20)
    episode = struct.unpack_from("<I", raw, 76)[0]
    target, fin_angle, fin_rate, lateness, apply_result, vbus_result = struct.unpack_from("<iiiiii", raw, 80)
    requested_command, applied_command, vbus_mv, abort_reason = struct.unpack_from("<hhHH", raw, 104)
    stage, phase, branch, requested_mode, applied_mode, guard, vbus_valid, reserved = struct.unpack_from("<BBBBBBBB", raw, 112)
    expected, actual, valid_count, repeated, skipped, invalid = struct.unpack_from("<BBBBBB", raw, 120)
    epoch_flags = struct.unpack_from("<H", raw, 126)[0]
    zero_reference, run_kind, qualification, reserved_2 = struct.unpack_from("<BBBB", raw, 128)
    first_error = struct.unpack_from("<i", raw, 132)[0]

    _need(stage in STAGES and stage == header["stage_value"], f"record {index} stage mismatch")
    _need(phase in PHASES, f"record {index} has unknown profile phase {phase}")
    _need(branch in BRANCHES, f"record {index} has unknown approach branch {branch}")
    _need(requested_mode in MOTOR_MODES and applied_mode in MOTOR_MODES, f"record {index} has unknown motor mode")
    _need(guard in GUARD_STATES, f"record {index} has unknown guard state {guard}")
    _need(abort_reason in ABORT_REASONS, f"record {index} has unknown abort reason {abort_reason}")
    _need(vbus_valid in (0, 1), f"record {index} vbus valid is not boolean")
    _need(zero_reference in (0, 1), f"record {index} zero reference is unknown")
    _need(run_kind in RUN_KINDS, f"record {index} run kind is unknown")
    _need(qualification in QUALIFICATIONS, f"record {index} rate qualification is unknown")
    _need(qualification == (0 if run_kind == 1 else 1), f"record {index} rate qualification/run-kind mismatch")
    _need(reserved == 0 and reserved_2 == 0, f"record {index} reserved field is nonzero")
    _need(epoch_flags & ~KNOWN_EPOCH_FLAGS == 0, f"record {index} has unknown epoch flags")
    expected_slots = int(header["encoder_rate_hz"]) // 1000
    _need(expected == expected_slots, f"record {index} expected sample count mismatch")
    _need(valid_count + invalid == expected - skipped, f"record {index} selected sample counts tear")
    _need(actual == valid_count + invalid + repeated, f"record {index} actual sample count tears")
    _need(epoch_end - epoch_start == 1000, f"record {index} epoch is not 1 ms")
    expected_start = int(header["epoch_zero_timestamp_us"]) + epoch_index * 1000
    _need(epoch_start == expected_start and epoch_end == expected_start + 1000, f"record {index} fixed epoch boundary mismatch")
    _need(release >= epoch_end and lateness == min(release - epoch_end, 0x7FFF_FFFF), f"record {index} release/lateness mismatch")
    _need(
        snapshot_us != 0 and apply_us != 0 and apply_us <= release <= snapshot_us,
        f"record {index} command timestamps tear",
    )
    _need(generation != 0, f"record {index} command generation is zero")
    _need(_mode_matches(requested_command, requested_mode), f"record {index} requested command/mode mismatch")
    _need(_mode_matches(applied_command, applied_mode), f"record {index} applied command/mode mismatch")
    _need(
        abs(requested_command) <= MAXIMUM_COMMAND_PERMILLE
        and abs(applied_command) <= MAXIMUM_COMMAND_PERMILLE,
        f"record {index} command exceeds the profile contract limit",
    )
    _need(run_kind == (1 if header["header_flags"] == 1 else 2), f"record {index} run kind/header mismatch")
    _need(zero_reference == (1 if stage == 4 else 0), f"record {index} zero reference/stage mismatch")

    samples = [
        _decode_raw_slot(raw, 136 + slot * 36, slot, expected_slots, epoch_start, epoch_end)
        for slot in range(5)
    ]
    _need(all(sample is None for sample in samples[expected:]), f"record {index} contains samples beyond the selected rate")
    present = sum(sample is not None for sample in samples[:expected])
    valid_from_samples = sum(bool(sample and sample["valid"]) for sample in samples[:expected])
    _need(present == valid_count + invalid, f"record {index} raw sample presence tears")
    _need(valid_from_samples == valid_count, f"record {index} raw valid count tears")

    complete = repeated == skipped == invalid == 0 and valid_count == expected
    _need(bool(epoch_flags & EPOCH_AGGREGATE_VALID) == complete, f"record {index} aggregate-valid flag tears")
    _need(bool(epoch_flags & EPOCH_INCOMPLETE) == (not complete), f"record {index} incomplete flag tears")
    _need(bool(epoch_flags & EPOCH_REPEATED) == (repeated != 0), f"record {index} repeated flag tears")
    _need(bool(epoch_flags & EPOCH_SKIPPED) == (skipped != 0), f"record {index} skipped flag tears")
    _need(bool(epoch_flags & EPOCH_INVALID) == (invalid != 0), f"record {index} invalid flag tears")
    command_applied_this_epoch = apply_us >= epoch_start
    command_apply_target_late = command_applied_this_epoch and apply_us > epoch_start + 100
    command_apply_hard_deadline_missed = command_applied_this_epoch and apply_us >= epoch_start + 1000
    deadline_flag = bool(epoch_flags & EPOCH_DEADLINE)
    release_late = lateness > 100
    _need(
        not command_apply_hard_deadline_missed or deadline_flag,
        f"record {index} hard command deadline lacks deadline flag",
    )
    _need(
        run_kind != 1 or not command_apply_target_late or deadline_flag,
        f"record {index} rate-check late command lacks deadline flag",
    )
    _need(
        not deadline_flag or release_late or command_apply_target_late,
        f"record {index} deadline flag tears",
    )
    legacy_command_deadline = (
        deadline_flag
        and command_apply_target_late
        and not command_apply_hard_deadline_missed
    )
    _need(
        bool(epoch_flags & EPOCH_STARTUP_INCOMPLETE)
        == (epoch_index == 0 and not complete),
        f"record {index} startup flag does not match epoch zero",
    )
    _need((first_error == 0) == (abort_reason == 0), f"record {index} first error/abort reason tear")
    _need(
        run_kind != 2
        or not (command_apply_hard_deadline_missed or legacy_command_deadline)
        or (first_error != 0 and abort_reason == 6),
        f"record {index} late full-run command lacks deadline abort evidence",
    )
    vbus_validity = (vbus_valid == 1 and vbus_result == 0 and vbus_us != 0) or (
        vbus_valid == 0 and vbus_result != 0
    )
    _need(vbus_validity, f"record {index} Vbus validity tears")
    _need(vbus_us <= snapshot_us, f"record {index} Vbus timestamp is from the future")
    _need(vbus_valid == 0 or snapshot_us - vbus_us <= MAX_VBUS_AGE_US, f"record {index} Vbus sample is stale")

    return {
        "sequence": sequence,
        "epoch_index": epoch_index,
        "epoch_start_timestamp_us": epoch_start,
        "epoch_end_timestamp_us": epoch_end,
        "release_timestamp_us": release,
        "command_apply_timestamp_us": apply_us,
        "logger_snapshot_timestamp_us": snapshot_us,
        "vbus_capture_timestamp_us": vbus_us,
        "command_generation": generation,
        "episode_id": episode,
        "target_fin_millideg": target,
        "fin_angle_millideg": fin_angle,
        "fin_rate_millideg_s": fin_rate,
        "consumer_lateness_us": lateness,
        "command_apply_result": apply_result,
        "vbus_read_result": vbus_result,
        "requested_command_permille": requested_command,
        "applied_command_permille": applied_command,
        "motor_vbus_mv": vbus_mv,
        "abort_reason": ABORT_REASONS[abort_reason],
        "abort_reason_value": abort_reason,
        "stage": STAGES[stage],
        "profile_phase": PHASES[phase],
        "profile_phase_value": phase,
        "approach_branch": BRANCHES[branch],
        "approach_branch_value": branch,
        "requested_motor_mode": MOTOR_MODES[requested_mode],
        "requested_motor_mode_value": requested_mode,
        "applied_motor_mode": MOTOR_MODES[applied_mode],
        "applied_motor_mode_value": applied_mode,
        "guard_state": GUARD_STATES[guard],
        "guard_state_value": guard,
        "vbus_valid": bool(vbus_valid),
        "expected_raw_sample_count": expected,
        "actual_raw_sample_count": actual,
        "valid_raw_sample_count": valid_count,
        "repeated_raw_sample_count": repeated,
        "skipped_raw_sample_count": skipped,
        "invalid_raw_sample_count": invalid,
        "epoch_flags": epoch_flags,
        "zero_reference_kind": "common" if zero_reference == 0 else "m0",
        "run_kind": RUN_KINDS[run_kind],
        "rate_qualification": QUALIFICATIONS[qualification],
        "first_error": first_error,
        "raw_samples": samples,
        "record_crc32": struct.unpack_from("<I", raw, 316)[0],
    }


def _decode_footer(data: bytes) -> dict[str, object]:
    _need(len(data) == FOOTER_SIZE, "truncated V5 footer")
    _need(data[:8] == FOOTER_MAGIC, "invalid V5 footer magic")
    schema, size, completion, rate_supported, unsupported_reason = struct.unpack_from("<HHBBH", data, 8)
    _need(schema == SCHEMA_VERSION and size == FOOTER_SIZE, "footer schema/size mismatch")
    _need(completion in COMPLETIONS, f"unknown completion {completion}")
    _need(rate_supported in (0, 1), "rate_supported is not boolean")
    _need(unsupported_reason in UNSUPPORTED_REASONS, f"unknown unsupported reason {unsupported_reason}")
    completion_fields_valid = (
        completion == 1 and rate_supported == 1 and unsupported_reason == 0
    ) or (
        completion == 2 and unsupported_reason == 0
    ) or (
        completion == 3 and rate_supported == 0 and unsupported_reason != 0
    )
    _need(completion_fields_valid, "completion/rate-supported/unsupported-reason mismatch")
    total_records, first_sequence, last_sequence = struct.unpack_from("<QQQ", data, 16)
    counters = dict(zip(COUNTER_NAMES, struct.unpack_from("<11Q", data, 40), strict=True))
    encoder_transport_errors, encoder_status_errors, vbus_invalid = struct.unpack_from("<QQQ", data, 128)
    first_error, rate_valid, rate_total, shutdown_mask, file_crc = struct.unpack_from("<iIIII", data, 152)
    _need(not any(data[172:188]), "footer reserved bytes must be zero")
    footer_crc = struct.unpack_from("<I", data, 188)[0]
    _need(crc32(data[:188]) == footer_crc, "footer CRC32 mismatch")
    _need(rate_valid <= rate_total, "rate-check numerator exceeds denominator")
    return {
        "schema_version": schema,
        "footer_bytes": size,
        "completion": COMPLETIONS[completion],
        "completion_value": completion,
        "rate_supported": bool(rate_supported),
        "unsupported_reason": UNSUPPORTED_REASONS[unsupported_reason],
        "unsupported_reason_value": unsupported_reason,
        "total_records": total_records,
        "first_sequence": first_sequence,
        "last_sequence": last_sequence,
        "counters": counters,
        "encoder_transport_errors": encoder_transport_errors,
        "encoder_status_errors": encoder_status_errors,
        "vbus_invalid": vbus_invalid,
        "first_error": first_error,
        "rate_check_valid_epochs": rate_valid,
        "rate_check_total_epochs": rate_total,
        "shutdown_step_mask": shutdown_mask,
        "file_crc32": file_crc,
        "footer_crc32": footer_crc,
    }


def verify_bytes(data: bytes, source: str = "<memory>") -> dict[str, object]:
    """V5 capture全体を検証し、losslessなdecode結果を返す。"""
    _need(len(data) >= HEADER_SIZE + FOOTER_SIZE, "capture is shorter than header+footer")
    header = _decode_header(data)
    footer = _decode_footer(data[-FOOTER_SIZE:])
    payload = data[HEADER_SIZE:-FOOTER_SIZE]
    _need(len(payload) % RECORD_SIZE == 0, "record payload is truncated or has trailing bytes")
    record_count = len(payload) // RECORD_SIZE
    _need(record_count == footer["total_records"], "footer total_records mismatch")
    _need(crc32(data[:-FOOTER_SIZE]) == footer["file_crc32"], "file CRC32 mismatch")
    records = [
        _decode_record(payload[offset : offset + RECORD_SIZE], header, index)
        for index, offset in enumerate(range(0, len(payload), RECORD_SIZE))
    ]
    if records:
        sequences = [int(record["sequence"]) for record in records]
        epochs = [int(record["epoch_index"]) for record in records]
        generations = [int(record["command_generation"]) for record in records]
        _need(all(right == left + 1 for left, right in zip(sequences, sequences[1:])), "record sequence gap/duplicate")
        _need(all(right == left + 1 for left, right in zip(epochs, epochs[1:])), "epoch index gap/duplicate")
        _need(all(right >= left for left, right in zip(generations, generations[1:])), "command generation decreases")
        _need(footer["first_sequence"] == sequences[0] and footer["last_sequence"] == sequences[-1], "footer sequence bounds mismatch")

        command_fields = (
            "requested_command_permille",
            "requested_motor_mode_value",
            "applied_command_permille",
            "applied_motor_mode_value",
            "command_apply_result",
            "command_apply_timestamp_us",
        )
        for left, right in zip(records, records[1:]):
            _need(
                int(right["release_timestamp_us"])
                > int(left["release_timestamp_us"])
                and int(right["logger_snapshot_timestamp_us"])
                > int(left["logger_snapshot_timestamp_us"]),
                "record release/snapshot timestamps do not increase",
            )
            if left["command_generation"] == right["command_generation"]:
                _need(
                    all(left[field] == right[field] for field in command_fields),
                    "command evidence changed without a new generation",
                )
            else:
                _need(
                    int(right["command_apply_timestamp_us"])
                    > int(left["command_apply_timestamp_us"])
                    and int(right["command_apply_timestamp_us"])
                    >= int(right["epoch_start_timestamp_us"]),
                    "new command generation has an invalid apply timestamp",
                )
    else:
        _need(footer["first_sequence"] == 0 and footer["last_sequence"] == 0, "empty capture has nonzero sequence bounds")

    raw_samples = [
        sample
        for record in records
        for sample in record["raw_samples"]
        if sample is not None
    ]
    for field in ("generation", "scheduled_timestamp_us", "capture_timestamp_us"):
        values = [int(sample[field]) for sample in raw_samples]
        _need(all(right > left for left, right in zip(values, values[1:])), f"raw {field} is not strictly increasing")
    _need(
        all(
            int(record["first_error"]) == int(footer["first_error"])
            for record in records
            if int(record["first_error"]) != 0
        ),
        "record/footer first error mismatch",
    )

    derived = {
        "repeated": sum(int(record["repeated_raw_sample_count"]) for record in records),
        "skipped": sum(int(record["skipped_raw_sample_count"]) for record in records),
        "invalid": sum(int(record["invalid_raw_sample_count"]) for record in records),
        "startup_incomplete": sum(bool(int(record["epoch_flags"]) & EPOCH_STARTUP_INCOMPLETE) for record in records),
        "steady_state_incomplete": sum(bool(int(record["epoch_flags"]) & EPOCH_INCOMPLETE and not int(record["epoch_flags"]) & EPOCH_STARTUP_INCOMPLETE) for record in records),
        "consumer_deadline_miss": sum(bool(int(record["epoch_flags"]) & EPOCH_DEADLINE) for record in records),
    }
    for name, value in derived.items():
        if name == "consumer_deadline_miss":
            continue
        _need(footer["counters"][name] == value, f"footer {name} counter mismatch")
    persisted_deadlines = derived["consumer_deadline_miss"]
    reported_deadlines = footer["counters"]["consumer_deadline_miss"]
    _need(reported_deadlines >= persisted_deadlines, "footer consumer deadline counter is too small")
    _need(
        reported_deadlines == persisted_deadlines
        or footer["completion"] in ("aborted", "unsupported"),
        "footer consumer deadline surplus requires aborted/unsupported completion",
    )
    late_records = sum(bool(int(record["epoch_flags"]) & EPOCH_LATE) for record in records)
    _need(
        footer["counters"]["late_after_release"] >= late_records,
        "footer late counter is smaller than flagged epochs",
    )
    aggregate_valid_epochs = sum(
        bool(int(record["epoch_flags"]) & EPOCH_AGGREGATE_VALID)
        for record in records
    )
    if header["run_kind"] == "rate-check":
        _need(footer["rate_check_total_epochs"] == record_count, "rate-check denominator does not match record count")
        _need(footer["rate_check_valid_epochs"] == aggregate_valid_epochs, "rate-check numerator mismatch")
    else:
        _need(footer["rate_check_total_epochs"] == footer["rate_check_valid_epochs"] == 0, "full capture reports rate-check counts")
    derived_vbus_invalid = sum(not bool(record["vbus_valid"]) for record in records)
    _need(footer["vbus_invalid"] >= derived_vbus_invalid, "footer Vbus invalid counter is too small")
    _need(
        footer["vbus_invalid"] == derived_vbus_invalid
        or footer["completion"] == "aborted",
        "footer Vbus invalid surplus requires aborted completion",
    )
    _need(
        footer["completion"] != "unsupported" or footer["vbus_invalid"] == 0,
        "unsupported capture reports Vbus invalid evidence",
    )
    raw_transport_errors = sum(
        sample is not None and int(sample["read_result"]) != 0
        for record in records
        for sample in record["raw_samples"]
    )
    raw_status_errors = sum(
        sample is not None and int(sample["diagnostic_flags"]) & 0x38 != 0
        for record in records
        for sample in record["raw_samples"]
    )
    _need(footer["encoder_transport_errors"] >= raw_transport_errors, "footer encoder transport counter is too small")
    _need(footer["encoder_status_errors"] >= raw_status_errors, "footer encoder status counter is too small")

    fatal_counters = (
        footer["counters"]["trigger_coalesced_missed"],
        footer["counters"]["pre_epoch"],
        # repeated/skippedがstartup epochだけならsteady_state_incompleteは増えない。
        footer["counters"]["invalid"],
        footer["counters"]["late_after_release"],
        footer["counters"]["raw_queue_overflow"],
        footer["counters"]["writer_queue_overflow"],
        footer["encoder_transport_errors"],
        footer["encoder_status_errors"],
        footer["vbus_invalid"],
    )
    normal = footer["completion"] == "normal"
    unsupported = footer["completion"] == "unsupported"
    _need(not unsupported or header["run_kind"] == "rate-check", "only a rate-check may be classified unsupported")
    if header["run_kind"] == "rate-check":
        _need(
            all(
                record["profile_phase_value"] == 1
                and record["episode_id"] == 1
                and record["approach_branch_value"] == 0
                for record in records
            ),
            "rate-check records are not stationary baseline episode 1",
        )
    if normal and header["run_kind"] == "full":
        phases = {int(record["profile_phase_value"]) for record in records}
        _need(REQUIRED_FULL_PHASES <= phases, "normal full capture omits required profile phases")

    unsupported_evidence = {
        1: footer["counters"]["trigger_coalesced_missed"] != 0,
        2: any(
            footer["counters"][name] != 0
            for name in (
                "pre_epoch",
                "repeated",
                "late_after_release",
                "steady_state_incomplete",
            )
        ),
        3: footer["counters"]["invalid"] != 0
        or footer["encoder_transport_errors"] != 0,
        4: footer["counters"]["consumer_deadline_miss"] != 0,
        5: footer["counters"]["raw_queue_overflow"] != 0
        or footer["counters"]["writer_queue_overflow"] != 0,
        6: footer["first_error"] != 0,
        7: footer["encoder_status_errors"] != 0,
        8: True,
    }
    _need(
        not unsupported
        or unsupported_evidence[int(footer["unsupported_reason_value"])],
        "unsupported reason lacks matching acquisition evidence",
    )
    shutdown_mask = int(footer["shutdown_step_mask"])
    _need(
        shutdown_mask <= COMPLETE_SHUTDOWN_MASK
        and shutdown_mask & (shutdown_mask + 1) == 0,
        "shutdown step mask is not a known ordered prefix",
    )
    _need(
        footer["completion"] == "aborted" or shutdown_mask == COMPLETE_SHUTDOWN_MASK,
        "completed capture lacks the required shutdown prefix",
    )
    _need(not normal or record_count != 0, "normal capture contains no epoch records")
    _need(not normal or (footer["first_error"] == 0 and not any(fatal_counters)), "normal capture reports a fatal acquisition error")
    _need(not normal or footer["counters"]["steady_state_incomplete"] == 0, "normal capture has steady-state incomplete epochs")
    _need(not normal or footer["counters"]["consumer_deadline_miss"] == 0, "normal capture has consumer deadline misses")
    _need(not normal or all(record["abort_reason_value"] == 0 and record["first_error"] == 0 and record["command_apply_result"] == 0 for record in records), "normal capture contains an aborted/error epoch")

    integrity = {
        "schema_version": SCHEMA_VERSION,
        "source": source,
        "sha256": hashlib.sha256(data).hexdigest(),
        "byte_count": len(data),
        "record_count": record_count,
        "header_bytes": header["header_bytes"],
        "record_bytes": header["record_bytes"],
        "footer_bytes": footer["footer_bytes"],
        "session_id": header["session_id"],
        "session_consistent": True,
        "stage": header["stage"],
        "encoder_rate_hz": header["encoder_rate_hz"],
        "profile_seed": header["profile_seed"],
        "run_kind": header["run_kind"],
        "raw_expected_count": sum(int(record["expected_raw_sample_count"]) for record in records),
        "raw_actual_count": sum(int(record["actual_raw_sample_count"]) for record in records),
        "raw_valid_count": sum(int(record["valid_raw_sample_count"]) for record in records),
        "sequence_first": footer["first_sequence"],
        "sequence_last": footer["last_sequence"],
        "sequence_gaps": 0,
        "crc_errors": 0,
        "command_mode_mismatches": 0,
        "vbus_timestamp_errors": 0,
        "classification": footer["completion"],
        "completion": footer["completion"],
        "completion_value": footer["completion_value"],
        "rate_supported": footer["rate_supported"],
        "unsupported_reason": footer["unsupported_reason"],
        "unsupported_reason_value": footer["unsupported_reason_value"],
        "rate_check_valid_epochs": footer["rate_check_valid_epochs"],
        "rate_check_total_epochs": footer["rate_check_total_epochs"],
        "first_error": footer["first_error"],
        "shutdown_step_mask": footer["shutdown_step_mask"],
        "strict_pass": footer["completion"] in ("normal", "unsupported"),
        **footer["counters"],
        "encoder_transport_errors": footer["encoder_transport_errors"],
        "encoder_status_errors": footer["encoder_status_errors"],
        "vbus_invalid": footer["vbus_invalid"],
    }
    return {"metadata": header, "epochs": records, "footer": footer, "integrity": integrity}


def verify_file(path: Path) -> dict[str, object]:
    return verify_bytes(path.read_bytes(), str(path))


def _put_text(buffer: bytearray, offset: int, size: int, value: str, *, pad: bool = True) -> None:
    encoded = value.encode("ascii")
    _need(len(encoded) <= size if not pad else len(encoded) < size, "fixture text is too long")
    buffer[offset : offset + len(encoded)] = encoded


def build_golden_fixture(
    *,
    incomplete_first: bool = False,
    stage_value: int = 1,
    encoder_rate_hz: int = 5000,
    run_kind: int = 1,
    sequence_base: int = 52_764,
    session_id: str = "golden-session",
) -> bytes:
    """Python/MATLAB相互運用試験用の決定的V5 fixtureを生成する。"""
    epoch_zero = 1_000_000
    header = bytearray(HEADER_SIZE)
    header[:8] = HEADER_MAGIC
    struct.pack_into(
        "<HHHHBBHIIIIIfffffIQ",
        header,
        8,
        5,
        256,
        320,
        192,
        stage_value,
        0,
        1000,
        encoder_rate_hz,
        1,
        0x1234_5678,
        30_000,
        1,
        176.175,
        15.0,
        8.0,
        10.0,
        0.344,
        0,
        epoch_zero,
    )
    _put_text(header, 72, 32, session_id)
    _put_text(header, 104, 40, "a" * 40, pad=False)
    _put_text(header, 144, 40, "b" * 40, pad=False)
    _put_text(header, 184, 64, "avi_99l_missionboard_characterization")
    struct.pack_into("<I", header, 248, 1 if run_kind == 1 else 2)
    struct.pack_into("<I", header, 252, crc32(header[:252]))

    episode_specs = (
        [(1, 1, 0), (1, 1, 0)]
        if run_kind == 1
        else [
            (1, 1, 0),
            (2, 2, 1),
            (2, 3, 2),
            *((phase, phase + 1, 0) for phase in range(3, 15)),
            (1, 16, 0),
        ]
    )
    records: list[bytes] = []
    repeated_total = skipped_total = invalid_total = startup_total = steady_total = deadline_total = 0
    for epoch, (phase, episode, branch) in enumerate(episode_specs):
        raw = bytearray(RECORD_SIZE)
        raw[:4] = RECORD_MAGIC
        sequence = sequence_base + epoch
        start = epoch_zero + epoch * 1000
        end = start + 1000
        release = end + 50
        snapshot = release + 50
        vbus_capture = release + 10
        struct.pack_into("<QQ", raw, 4, sequence, epoch)
        struct.pack_into("<QQQQQQQ", raw, 20, start, end, release, start + 10, snapshot, vbus_capture, epoch + 1)
        struct.pack_into("<I", raw, 76, episode)
        struct.pack_into("<iiiiii", raw, 80, 0, 0, 0, 50, 0, 0)
        struct.pack_into("<hhHH", raw, 104, 0, 0, 9000, 0)
        struct.pack_into("<BBBBBBBB", raw, 112, stage_value, phase, branch, 0, 0, 0, 1, 0)
        expected_slots = encoder_rate_hz // 1000
        missing_slot = expected_slots // 2 if incomplete_first and epoch == 0 else -1
        valid_count = expected_slots - (1 if missing_slot >= 0 else 0)
        skipped = 1 if missing_slot >= 0 else 0
        flags = EPOCH_AGGREGATE_VALID if skipped == 0 else EPOCH_INCOMPLETE | EPOCH_SKIPPED | EPOCH_STARTUP_INCOMPLETE
        struct.pack_into("<BBBBBB", raw, 120, expected_slots, valid_count, valid_count, 0, skipped, 0)
        struct.pack_into("<H", raw, 126, flags)
        struct.pack_into("<BBBBi", raw, 128, 1 if stage_value == 4 else 0, run_kind, 0 if run_kind == 1 else 1, 0, 0)
        for slot in range(expected_slots):
            if slot == missing_slot:
                continue
            scheduled = start + ((2 * slot + 1) * 1000) // (2 * expected_slots)
            capture = scheduled + 10
            struct.pack_into("<QQQHHiBBH", raw, 136 + slot * 36, epoch * 5 + slot + 1, scheduled, capture, 1000 + epoch * 5 + slot, 0, 0, 1, slot, 0)
        struct.pack_into("<I", raw, 316, crc32(raw[:316]))
        records.append(bytes(raw))
        skipped_total += skipped
        startup_total += int(skipped != 0)

    body = bytes(header) + b"".join(records)
    footer = bytearray(FOOTER_SIZE)
    footer[:8] = FOOTER_MAGIC
    struct.pack_into("<HHBBH", footer, 8, 5, 192, 1, 1, 0)
    struct.pack_into("<QQQ", footer, 16, len(records), sequence_base, sequence_base + len(records) - 1)
    counters = (0, 0, repeated_total, skipped_total, invalid_total, 0, startup_total, steady_total, deadline_total, 0, 0)
    struct.pack_into("<11Q", footer, 40, *counters)
    struct.pack_into("<QQQ", footer, 128, 0, 0, 0)
    rate_valid = len(records) - startup_total if run_kind == 1 else 0
    rate_total = len(records) if run_kind == 1 else 0
    struct.pack_into("<iIIII", footer, 152, 0, rate_valid, rate_total, 0x3F, crc32(body))
    struct.pack_into("<I", footer, 188, crc32(footer[:188]))
    return body + bytes(footer)


def self_test() -> None:
    def refresh_mutation(data: bytearray, *record_indexes: int) -> bytes:
        footer_offset = len(data) - FOOTER_SIZE
        for record_index in record_indexes:
            record_offset = HEADER_SIZE + record_index * RECORD_SIZE
            struct.pack_into(
                "<I", data, record_offset + 316, crc32(data[record_offset : record_offset + 316])
            )
        struct.pack_into("<I", data, footer_offset + 168, crc32(data[:footer_offset]))
        struct.pack_into("<I", data, footer_offset + 188, crc32(data[footer_offset : footer_offset + 188]))
        return bytes(data)

    def expect_rejected(data: bytes, message: str) -> None:
        try:
            verify_bytes(data)
        except ValidationError as error:
            _need(message in str(error), f"mutation rejected for the wrong reason: {error}")
        else:
            raise ValidationError("invalid mutation was accepted")

    _need(crc32(b"123456789") == 0xCBF4_3926, "CRC32 check vector failed")
    valid = build_golden_fixture(incomplete_first=True)
    _need(
        hashlib.sha256(valid).hexdigest()
        == "85a889f6e445b5d0f4a8fd176184ddf25c292dadacafb2ebd2059ac2062c7660",
        "golden fixture bytes changed",
    )
    decoded = verify_bytes(valid)
    _need(len(decoded["epochs"]) == 2, "golden fixture record count failed")
    _need(decoded["epochs"][0]["skipped_raw_sample_count"] == 1, "4-sample epoch was not preserved")
    _need(decoded["epochs"][1]["valid_raw_sample_count"] == 5, "next epoch borrowed/lost a sample")

    first_record = HEADER_SIZE
    second_record = HEADER_SIZE + RECORD_SIZE
    first_apply = struct.unpack_from("<Q", valid, first_record + 44)[0]
    first_generation = struct.unpack_from("<Q", valid, first_record + 68)[0]
    second_epoch_start = struct.unpack_from("<Q", valid, second_record + 20)[0]
    apply_at_budget = bytearray(valid)
    struct.pack_into("<Q", apply_at_budget, second_record + 44, second_epoch_start + 100)
    _need(
        verify_bytes(refresh_mutation(apply_at_budget, 1))["epochs"][1]["command_apply_timestamp_us"]
        == second_epoch_start + 100,
        "100 us command-apply budget boundary failed",
    )

    # 同じgeneration/stateが継続する場合、apply時刻は現在epochより前でよい。
    stable_command = bytearray(valid)
    struct.pack_into("<Q", stable_command, second_record + 44, first_apply)
    struct.pack_into("<Q", stable_command, second_record + 68, first_generation)
    stable_decoded = verify_bytes(refresh_mutation(stable_command, 1))
    _need(
        stable_decoded["epochs"][1]["command_apply_timestamp_us"] == first_apply,
        "stable command did not preserve its original apply timestamp",
    )

    # 新generationなのに現在epochより前のapply時刻は不正。
    apply_before_epoch = bytearray(valid)
    struct.pack_into("<Q", apply_before_epoch, second_record + 44, second_epoch_start - 1)
    expect_rejected(
        refresh_mutation(apply_before_epoch, 1),
        "new command generation has an invalid apply timestamp",
    )

    unflagged_late_apply = bytearray(valid)
    struct.pack_into("<Q", unflagged_late_apply, second_record + 44, second_epoch_start + 101)
    expect_rejected(refresh_mutation(unflagged_late_apply, 1), "deadline flag")

    diagnosed_rate_late = bytearray(valid)
    diagnosed_footer = len(diagnosed_rate_late) - FOOTER_SIZE
    struct.pack_into("<Q", diagnosed_rate_late, second_record + 44, second_epoch_start + 101)
    struct.pack_into("<H", diagnosed_rate_late, second_record + 126, EPOCH_AGGREGATE_VALID | EPOCH_DEADLINE)
    struct.pack_into("<BBH", diagnosed_rate_late, diagnosed_footer + 12, 3, 0, 4)
    struct.pack_into("<Q", diagnosed_rate_late, diagnosed_footer + 104, 1)
    _need(
        verify_bytes(refresh_mutation(diagnosed_rate_late, 1))["footer"]["completion"]
        == "unsupported",
        "rate-check late-command diagnostic failed",
    )

    full_late = bytearray(build_golden_fixture(run_kind=2))
    full_footer = len(full_late) - FOOTER_SIZE
    full_start = struct.unpack_from("<Q", full_late, HEADER_SIZE + 20)[0]
    struct.pack_into("<Q", full_late, HEADER_SIZE + 44, full_start + 101)
    struct.pack_into("<H", full_late, HEADER_SIZE + 126, EPOCH_AGGREGATE_VALID | EPOCH_DEADLINE)
    struct.pack_into("<H", full_late, HEADER_SIZE + 110, 6)
    struct.pack_into("<i", full_late, HEADER_SIZE + 132, -1)
    struct.pack_into("<B", full_late, full_footer + 12, 2)
    struct.pack_into("<Q", full_late, full_footer + 104, 1)
    struct.pack_into("<i", full_late, full_footer + 152, -1)
    _need(
        verify_bytes(refresh_mutation(full_late, 0))["footer"]["completion"]
        == "aborted",
        "full-run late-command abort evidence failed",
    )

    full_late_without_abort = bytearray(build_golden_fixture(run_kind=2))
    full_footer = len(full_late_without_abort) - FOOTER_SIZE
    struct.pack_into("<Q", full_late_without_abort, HEADER_SIZE + 44, full_start + 101)
    struct.pack_into(
        "<H", full_late_without_abort, HEADER_SIZE + 126, EPOCH_AGGREGATE_VALID | EPOCH_DEADLINE
    )
    struct.pack_into("<B", full_late_without_abort, full_footer + 12, 2)
    struct.pack_into("<Q", full_late_without_abort, full_footer + 104, 1)
    expect_rejected(
        refresh_mutation(full_late_without_abort, 0),
        "late full-run command lacks deadline abort evidence",
    )

    inflated_aborted = bytearray(valid)
    inflated_footer = len(inflated_aborted) - FOOTER_SIZE
    struct.pack_into("<B", inflated_aborted, inflated_footer + 12, 2)
    struct.pack_into("<Q", inflated_aborted, inflated_footer + 104, 2)
    struct.pack_into("<Q", inflated_aborted, inflated_footer + 144, 3)
    _need(
        verify_bytes(refresh_mutation(inflated_aborted))["footer"]["completion"]
        == "aborted",
        "aborted pre-record counter evidence failed",
    )

    inflated_unsupported = bytearray(valid)
    inflated_footer = len(inflated_unsupported) - FOOTER_SIZE
    struct.pack_into("<BBH", inflated_unsupported, inflated_footer + 12, 3, 0, 4)
    struct.pack_into("<Q", inflated_unsupported, inflated_footer + 104, 2)
    _need(
        verify_bytes(refresh_mutation(inflated_unsupported))["footer"]["completion"]
        == "unsupported",
        "unsupported pre-record deadline evidence failed",
    )

    normal_deadline_surplus = bytearray(valid)
    normal_footer = len(normal_deadline_surplus) - FOOTER_SIZE
    struct.pack_into("<Q", normal_deadline_surplus, normal_footer + 104, 1)
    expect_rejected(
        refresh_mutation(normal_deadline_surplus),
        "deadline surplus",
    )

    normal_vbus_surplus = bytearray(valid)
    normal_footer = len(normal_vbus_surplus) - FOOTER_SIZE
    struct.pack_into("<Q", normal_vbus_surplus, normal_footer + 144, 1)
    expect_rejected(refresh_mutation(normal_vbus_surplus), "Vbus invalid surplus")

    unsupported_vbus = bytearray(valid)
    unsupported_footer = len(unsupported_vbus) - FOOTER_SIZE
    struct.pack_into("<BBH", unsupported_vbus, unsupported_footer + 12, 3, 0, 8)
    struct.pack_into("<Q", unsupported_vbus, unsupported_footer + 144, 1)
    expect_rejected(refresh_mutation(unsupported_vbus), "Vbus invalid surplus")

    startup_spoof = bytearray(build_golden_fixture())
    second = HEADER_SIZE + RECORD_SIZE
    startup_spoof[second + 136 + 2 * 36 : second + 136 + 3 * 36] = bytes(36)
    struct.pack_into("<BBBBBB", startup_spoof, second + 120, 5, 4, 4, 0, 1, 0)
    struct.pack_into(
        "<H",
        startup_spoof,
        second + 126,
        EPOCH_INCOMPLETE | EPOCH_SKIPPED | EPOCH_STARTUP_INCOMPLETE,
    )
    struct.pack_into("<I", startup_spoof, second + 316, crc32(startup_spoof[second : second + 316]))
    footer_offset = len(startup_spoof) - FOOTER_SIZE
    struct.pack_into("<Q", startup_spoof, footer_offset + 64, 1)
    struct.pack_into("<Q", startup_spoof, footer_offset + 88, 1)
    struct.pack_into("<II", startup_spoof, footer_offset + 156, 1, 2)
    struct.pack_into("<I", startup_spoof, footer_offset + 168, crc32(startup_spoof[:footer_offset]))
    struct.pack_into("<I", startup_spoof, footer_offset + 188, crc32(startup_spoof[footer_offset : footer_offset + 188]))
    try:
        verify_bytes(bytes(startup_spoof))
    except ValidationError as error:
        _need("startup flag" in str(error), "steady-state startup spoof rejected for the wrong reason")
    else:
        raise ValidationError("steady-state incomplete epoch was accepted as startup")

    unsupported = bytearray(valid)
    footer_offset = len(unsupported) - FOOTER_SIZE
    struct.pack_into("<BBH", unsupported, footer_offset + 12, 3, 0, 7)
    struct.pack_into("<I", unsupported, footer_offset + 188, crc32(unsupported[footer_offset : footer_offset + 188]))
    try:
        verify_bytes(bytes(unsupported))
    except ValidationError as error:
        _need("lacks matching" in str(error), "unsupported evidence rejected for the wrong reason")
    else:
        raise ValidationError("unsupported sensor-health result without evidence was accepted")
    struct.pack_into("<H", unsupported, footer_offset + 14, 8)
    struct.pack_into("<I", unsupported, footer_offset + 188, crc32(unsupported[footer_offset : footer_offset + 188]))
    _need(
        verify_bytes(bytes(unsupported))["footer"]["unsupported_reason"]
        == "OperatorMarkedUnsupported",
        "operator-marked unsupported result failed",
    )

    zero_aborted = bytearray(valid[:HEADER_SIZE] + valid[-FOOTER_SIZE:])
    footer_start = HEADER_SIZE
    struct.pack_into("<BBH", zero_aborted, footer_start + 12, 2, 0, 0)
    struct.pack_into("<QQQ", zero_aborted, footer_start + 16, 0, 0, 0)
    struct.pack_into("<11Q", zero_aborted, footer_start + 40, *([0] * 11))
    struct.pack_into("<II", zero_aborted, footer_start + 156, 0, 0)
    struct.pack_into("<I", zero_aborted, footer_start + 168, crc32(zero_aborted[:HEADER_SIZE]))
    struct.pack_into("<I", zero_aborted, footer_start + 188, crc32(zero_aborted[footer_start : footer_start + 188]))
    _need(not verify_bytes(bytes(zero_aborted))["epochs"], "zero-record aborted capture failed")

    zero_normal = bytearray(zero_aborted)
    struct.pack_into("<BB", zero_normal, footer_start + 12, 1, 1)
    struct.pack_into("<I", zero_normal, footer_start + 188, crc32(zero_normal[footer_start : footer_start + 188]))
    try:
        verify_bytes(bytes(zero_normal))
    except ValidationError as error:
        _need("no epoch records" in str(error), "normal zero-record capture failed for the wrong reason")
    else:
        raise ValidationError("normal zero-record capture was accepted")

    bad_session = bytearray(valid)
    bad_session[72:104] = b"x" * 32
    struct.pack_into("<I", bad_session, 252, crc32(bad_session[:252]))
    body_end = len(bad_session) - FOOTER_SIZE
    struct.pack_into("<I", bad_session, body_end + 168, crc32(bad_session[:body_end]))
    struct.pack_into("<I", bad_session, len(bad_session) - 4, crc32(bad_session[-FOOTER_SIZE:-4]))
    try:
        verify_bytes(bytes(bad_session))
    except ValidationError as error:
        _need("NUL terminated" in str(error), "unterminated session rejected for the wrong reason")
    else:
        raise ValidationError("unterminated session ID was accepted")

    corrupted = bytearray(valid)
    corrupted[HEADER_SIZE + 104] = 0
    corrupted[HEADER_SIZE + 115] = 2
    struct.pack_into("<I", corrupted, HEADER_SIZE + 316, crc32(corrupted[HEADER_SIZE : HEADER_SIZE + 316]))
    body_end = len(corrupted) - FOOTER_SIZE
    struct.pack_into("<I", corrupted, body_end + 168, crc32(corrupted[:body_end]))
    struct.pack_into("<I", corrupted, len(corrupted) - 4, crc32(corrupted[-FOOTER_SIZE:-4]))
    try:
        verify_bytes(bytes(corrupted))
    except ValidationError as error:
        _need("requested command/mode mismatch" in str(error), "historical mismatch rejected for the wrong reason")
    else:
        raise ValidationError("historical command=0/DriveIn2 mismatch was accepted")

    for malformed in (valid[:-1], valid + b"\0"):
        try:
            verify_bytes(malformed)
        except ValidationError:
            pass
        else:
            raise ValidationError("truncation/trailing bytes were accepted")

    bad_crc = bytearray(valid)
    bad_crc[HEADER_SIZE + 200] ^= 1
    try:
        verify_bytes(bytes(bad_crc))
    except ValidationError as error:
        _need("CRC32 mismatch" in str(error), "CRC corruption rejected for the wrong reason")
    else:
        raise ValidationError("record CRC corruption was accepted")

    unknown_enum = bytearray(valid)
    unknown_enum[HEADER_SIZE + 113] = 0xFF
    struct.pack_into("<I", unknown_enum, HEADER_SIZE + 316, crc32(unknown_enum[HEADER_SIZE : HEADER_SIZE + 316]))
    body_end = len(unknown_enum) - FOOTER_SIZE
    struct.pack_into("<I", unknown_enum, body_end + 168, crc32(unknown_enum[:body_end]))
    struct.pack_into("<I", unknown_enum, len(unknown_enum) - 4, crc32(unknown_enum[-FOOTER_SIZE:-4]))
    try:
        verify_bytes(bytes(unknown_enum))
    except ValidationError as error:
        _need("unknown profile phase" in str(error), "unknown enum rejected for the wrong reason")
    else:
        raise ValidationError("unknown enum was accepted")


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as output:
            temporary_name = output.name
            output.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, path)
    except Exception:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="?", type=Path)
    parser.add_argument("--integrity", type=Path, help="integrity.json output path")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--write-golden", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        if args.self_test:
            self_test()
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "golden.bin"
                path.write_bytes(build_golden_fixture(incomplete_first=True))
                verify_file(path)
            print("verify_characterization self-test: PASS")
            return 0
        if args.write_golden:
            args.write_golden.parent.mkdir(parents=True, exist_ok=True)
            golden = build_golden_fixture(incomplete_first=True)
            with args.write_golden.open("xb") as output:
                output.write(golden)
            print(hashlib.sha256(golden).hexdigest(), args.write_golden)
            return 0
        _need(args.capture is not None, "capture path is required")
        output = args.integrity or args.capture.with_name("integrity.json")
        _need(args.capture.resolve() != output.resolve(), "integrity output aliases the capture input")
        try:
            result = verify_file(args.capture)
        except ValidationError as error:
            data = args.capture.read_bytes()
            _write_json(
                output,
                {
                    "schema_version": SCHEMA_VERSION,
                    "source": str(args.capture),
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "byte_count": len(data),
                    "classification": "invalid",
                    "strict_pass": False,
                    "error": str(error),
                },
            )
            print(f"FAIL: {error}; integrity={output}", file=sys.stderr)
            return 1
        _write_json(output, result["integrity"])
        if not result["integrity"]["strict_pass"]:
            print(f"REJECT records={len(result['epochs'])} classification={result['integrity']['classification']} integrity={output}", file=sys.stderr)
            return 1
        print(f"PASS records={len(result['epochs'])} classification={result['integrity']['classification']} integrity={output}")
        return 0
    except (OSError, ValidationError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
