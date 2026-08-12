#!/usr/bin/env python3
"""Mission Board bring-upのバイナリstreamを保存し、CSVへ変換する。"""

from __future__ import annotations

import argparse
import csv
import errno
import io
import json
import math
import os
from pathlib import Path
import re
import select
import socket
import stat
import struct
import sys
import tempfile
import termios
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import BinaryIO, Callable, Iterable


MAGIC = b"\xA5\x5A"
PROTOCOL_VERSION = 1
HEADER_SIZE = 10
TRAILER_SIZE = 2
MAX_PAYLOAD_SIZE = 96
MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + TRAILER_SIZE
MAX_FEED_SIZE = 65_536
SERIAL_BAUD = termios.B115200
MOTOR_CAPTURE_COMMANDS = frozenset(
    {
        "motor-polarity",
        "motor-step",
        "motor-prbs",
        "motor-coast",
        "motor-brake-test",
        "combined-motor-imu-test",
    }
)
DISARM_REQUEST_PATTERN = re.compile(rb"motor-disarm request: ([A-Z0-9_]+)")


def crc16_ccitt(data: bytes | bytearray | memoryview) -> int:
    """初期値0xffffのCRC-16/CCITT-FALSEを計算する。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    version: int
    record_type: int
    sequence: int
    payload: bytes
    raw_offset: int


class FrameParser:
    """任意位置で分割されたbyte列から、上限長付きframeを復元する。"""

    def __init__(self, on_frame: Callable[[Frame], None]) -> None:
        self._on_frame = on_frame
        self._buffer = bytearray()
        self._stream_offset = 0
        self.valid_frames = 0
        self.crc_errors = 0
        self.malformed_headers = 0
        self.discarded_bytes = 0
        self.incomplete_bytes = 0

    def _discard(self, count: int) -> None:
        del self._buffer[:count]
        self._stream_offset += count
        self.discarded_bytes += count

    def feed(self, data: bytes) -> None:
        if len(data) > MAX_FEED_SIZE:
            for offset in range(0, len(data), MAX_FEED_SIZE):
                self.feed(data[offset : offset + MAX_FEED_SIZE])
            return
        self._buffer.extend(data)
        while True:
            magic_at = self._buffer.find(MAGIC)
            if magic_at < 0:
                keep = 1 if self._buffer.endswith(MAGIC[:1]) else 0
                self._discard(len(self._buffer) - keep)
                return
            if magic_at:
                self._discard(magic_at)
            if len(self._buffer) < HEADER_SIZE:
                return

            version = self._buffer[2]
            payload_size = int.from_bytes(self._buffer[4:6], "little")
            if version != PROTOCOL_VERSION or payload_size > MAX_PAYLOAD_SIZE:
                self.malformed_headers += 1
                self._discard(1)
                continue

            frame_size = HEADER_SIZE + payload_size + TRAILER_SIZE
            if len(self._buffer) < frame_size:
                return
            expected_crc = int.from_bytes(self._buffer[frame_size - 2 : frame_size], "little")
            actual_crc = crc16_ccitt(memoryview(self._buffer)[2 : frame_size - 2])
            if actual_crc != expected_crc:
                self.crc_errors += 1
                self._discard(1)
                continue

            frame = Frame(
                version=version,
                record_type=self._buffer[3],
                sequence=int.from_bytes(self._buffer[6:10], "little"),
                payload=bytes(self._buffer[HEADER_SIZE : frame_size - TRAILER_SIZE]),
                raw_offset=self._stream_offset,
            )
            del self._buffer[:frame_size]
            self._stream_offset += frame_size
            self.valid_frames += 1
            self._on_frame(frame)

    def finish(self) -> None:
        self.incomplete_bytes = len(self._buffer)
        self._discard(len(self._buffer))


@dataclass(frozen=True)
class RecordSchema:
    name: str
    layout: struct.Struct
    fields: tuple[str, ...]
    decorate: Callable[[dict[str, object]], dict[str, object]] | None = None

    def decode(self, payload: bytes) -> dict[str, object] | None:
        if len(payload) != self.layout.size:
            return None
        row = dict(zip(self.fields, self.layout.unpack(payload), strict=True))
        return self.decorate(row) if self.decorate else row


def _decorate_adc(row: dict[str, object]) -> dict[str, object]:
    flags = int(row["valid_flags"])
    row.update(
        logic_raw_valid=bool(flags & (1 << 0)),
        logic_calibrated_valid=bool(flags & (1 << 1)),
        motor_raw_valid=bool(flags & (1 << 2)),
        motor_calibrated_valid=bool(flags & (1 << 3)),
    )
    return row


def _decorate_encoder(row: dict[str, object]) -> dict[str, object]:
    flags = int(row["flags"])
    row["valid"] = bool(flags & (1 << 0))
    return row


def _decorate_imu(row: dict[str, object]) -> dict[str, object]:
    flags = int(row["flags"])
    names = (
        "acceleration_valid",
        "angular_velocity_valid",
        "temperature_valid",
        "accel_odr_changed",
        "gyro_odr_changed",
        "fifo_threshold",
        "fifo_full",
        "fifo_fault",
    )
    row.update({name: bool(flags & (1 << bit)) for bit, name in enumerate(names)})
    return row


def _decorate_motor(row: dict[str, object]) -> dict[str, object]:
    drive_mode = int(row["drive_mode"])
    row["drive_mode_name"] = {0: "coast", 1: "in1", 2: "in2", 3: "brake"}.get(
        drive_mode, "unknown"
    )
    encoder_flags = int(row["encoder_status_flags"])
    row.update(
        encoder_magnetic_too_low=bool(encoder_flags & (1 << 0)),
        encoder_magnetic_too_high=bool(encoder_flags & (1 << 1)),
        encoder_cordic_overflow=bool(encoder_flags & (1 << 2)),
        encoder_offset_compensation_finished=bool(encoder_flags & (1 << 3)),
        encoder_status_snapshot_valid=bool(encoder_flags & (1 << 14)),
        encoder_valid=bool(encoder_flags & (1 << 15)),
    )
    imu_flags = int(row["imu_flags"])
    imu_names = (
        "imu_acceleration_valid",
        "imu_angular_velocity_valid",
        "imu_temperature_valid",
        "imu_accel_odr_changed",
        "imu_gyro_odr_changed",
    )
    row.update({name: bool(imu_flags & (1 << bit)) for bit, name in enumerate(imu_names)})
    fifo_flags = int(row["fifo_flags"])
    row.update(
        fifo_threshold=bool(fifo_flags & (1 << 0)),
        fifo_full=bool(fifo_flags & (1 << 1)),
        fifo_fault=bool(fifo_flags & (1 << 2)),
    )
    return row


def _decorate_calibration(row: dict[str, object]) -> dict[str, object]:
    flags = int(row["valid_flags"])
    row.update(
        gyro_valid=bool(flags & (1 << 0)),
        gravity_valid=bool(flags & (1 << 1)),
        launcher_tilt_valid=bool(flags & (1 << 2)),
        ssc_valid=bool(flags & (1 << 3)),
    )
    return row


SCHEMAS: dict[int, RecordSchema] = {
    1: RecordSchema(
        "encoder",
        struct.Struct("<QHffIB"),
        (
            "host_timestamp_us",
            "angle_raw",
            "angle_degrees",
            "angle_radians",
            "read_latency_us",
            "flags",
        ),
        _decorate_encoder,
    ),
    2: RecordSchema(
        "imu",
        struct.Struct("<QQH7h7fIHHB"),
        (
            "host_timestamp_us",
            "sensor_timestamp_us",
            "timestamp_ticks",
            "accel_raw_x",
            "accel_raw_y",
            "accel_raw_z",
            "gyro_raw_x",
            "gyro_raw_y",
            "gyro_raw_z",
            "temperature_raw",
            "accel_g_x",
            "accel_g_y",
            "accel_g_z",
            "gyro_dps_x",
            "gyro_dps_y",
            "gyro_dps_z",
            "temperature_celsius",
            "read_latency_us",
            "fifo_records_available",
            "fifo_lost_packets",
            "flags",
        ),
        _decorate_imu,
    ),
    3: RecordSchema(
        "adc",
        struct.Struct("<QBiffiiffi"),
        (
            "timestamp_us",
            "valid_flags",
            "logic_raw",
            "logic_pin_voltage_v",
            "logic_source_voltage_v",
            "logic_status",
            "motor_raw",
            "motor_pin_voltage_v",
            "motor_source_voltage_v",
            "motor_status",
        ),
        _decorate_adc,
    ),
    4: RecordSchema(
        "motor",
        struct.Struct("<QIfBifHffHBH3h3hHQBHBiii"),
        (
            "host_timestamp_us",
            "sample_index",
            "command_duty",
            "drive_mode",
            "motor_adc_raw",
            "motor_bus_voltage_v",
            "encoder_raw",
            "encoder_unwrapped_rad",
            "encoder_speed_rad_s",
            "encoder_status_flags",
            "encoder_agc",
            "encoder_magnitude",
            "accel_raw_x",
            "accel_raw_y",
            "accel_raw_z",
            "gyro_raw_x",
            "gyro_raw_y",
            "gyro_raw_z",
            "imu_timestamp_ticks",
            "imu_sensor_timestamp_us",
            "imu_flags",
            "imu_lost_packets",
            "fifo_flags",
            "adc_result",
            "encoder_result",
            "imu_result",
        ),
        _decorate_motor,
    ),
    5: RecordSchema(
        "calibration",
        struct.Struct("<QIBI3f3ffffI"),
        (
            "completed_at_us",
            "attempt_id",
            "valid_flags",
            "imu_samples",
            "gyro_bias_dps_x",
            "gyro_bias_dps_y",
            "gyro_bias_dps_z",
            "gravity_sensor_g_x",
            "gravity_sensor_g_y",
            "gravity_sensor_g_z",
            "acceleration_norm_g",
            "launcher_tilt_deg",
            "ssc_zero_pa",
            "error_count",
        ),
        _decorate_calibration,
    ),
}


AXES = ("x", "y", "z")


class RunningStatistics:
    """値を保持せず、Welford法で平均と標本標準偏差を更新する。"""

    def __init__(self) -> None:
        self.count = 0
        self.mean = 0.0
        self._m2 = 0.0

    def observe(self, value: float) -> None:
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self._m2 += delta * (value - self.mean)

    def summary(self) -> dict[str, object]:
        stddev = math.sqrt(max(0.0, self._m2 / (self.count - 1))) if self.count > 1 else None
        return {"count": self.count, "mean": self.mean if self.count else None, "stddev": stddev}


class VectorStatistics:
    def __init__(self) -> None:
        self._axes = tuple(RunningStatistics() for _ in AXES)

    def observe(self, values: tuple[float, float, float]) -> None:
        for statistics, value in zip(self._axes, values, strict=True):
            statistics.observe(value)

    def summary(self) -> dict[str, object]:
        return {
            "sample_count": self._axes[0].count,
            "axes": {axis: statistics.summary() for axis, statistics in zip(AXES, self._axes, strict=True)},
        }


class LinearDrift:
    """時刻と3軸値の最小二乗直線を逐次計算する。"""

    def __init__(self) -> None:
        self.count = 0
        self._origin_us: int | None = None
        self._last_seconds = 0.0
        self._mean_seconds = 0.0
        self._mean_values = [0.0, 0.0, 0.0]
        self._time_m2 = 0.0
        self._covariance = [0.0, 0.0, 0.0]

    def observe(self, timestamp_us: int, values: tuple[float, float, float]) -> None:
        if self._origin_us is None:
            self._origin_us = timestamp_us
        seconds = (timestamp_us - self._origin_us) / 1_000_000.0
        self._last_seconds = seconds
        next_count = self.count + 1
        time_delta = seconds - self._mean_seconds
        self._mean_seconds += time_delta / next_count
        self._time_m2 += time_delta * (seconds - self._mean_seconds)
        for index, value in enumerate(values):
            value_delta = value - self._mean_values[index]
            self._mean_values[index] += value_delta / next_count
            self._covariance[index] += time_delta * (value - self._mean_values[index])
        self.count = next_count

    def summary(self) -> dict[str, object]:
        slopes = (
            [value / self._time_m2 for value in self._covariance]
            if self.count > 1 and self._time_m2 > 0.0
            else [None, None, None]
        )
        return {
            "method": "gyro_dpsを時刻に対して最小二乗直線fit",
            "sample_count": self.count,
            "duration_s": self._last_seconds if self.count else None,
            "axes": {
                axis: {
                    "slope_dps_per_s": slope,
                    "estimated_change_dps": slope * self._last_seconds if slope is not None else None,
                }
                for axis, slope in zip(AXES, slopes, strict=True)
            },
        }


class WindowBiasStatistics:
    """1 kHz時刻列を固定幅の非重複windowへ分ける。"""

    def __init__(self, seconds: int) -> None:
        self.seconds = seconds
        self._duration_us = seconds * 1_000_000
        self._origin_us: int | None = None
        self._bucket: int | None = None
        self._sum = [0.0, 0.0, 0.0]
        self._count = 0
        self._first_us = 0
        self._last_us = 0
        self._means = VectorStatistics()
        self._sample_counts = RunningStatistics()
        self.skipped_empty_windows = 0
        self.rejected_trailing_windows = 0
        self._finished = False

    def _accept_current(self) -> None:
        if self._count == 0:
            return
        self._means.observe(tuple(value / self._count for value in self._sum))
        self._sample_counts.observe(float(self._count))

    def _start(self, bucket: int, timestamp_us: int) -> None:
        self._bucket = bucket
        self._sum = [0.0, 0.0, 0.0]
        self._count = 0
        self._first_us = timestamp_us
        self._last_us = timestamp_us

    def observe(self, timestamp_us: int, values: tuple[float, float, float]) -> None:
        if self._finished:
            raise RuntimeError("集計完了後にsampleは追加できません")
        if self._origin_us is None:
            self._origin_us = timestamp_us
            self._start(0, timestamp_us)
        assert self._origin_us is not None and self._bucket is not None
        bucket = (timestamp_us - self._origin_us) // self._duration_us
        if bucket != self._bucket:
            self._accept_current()
            self.skipped_empty_windows += max(0, bucket - self._bucket - 1)
            self._start(bucket, timestamp_us)
        for index, value in enumerate(values):
            self._sum[index] += value
        self._count += 1
        self._last_us = timestamp_us

    def finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        if self._count == 0:
            return
        # 末尾windowは1 kHzの1周期を加えて全期間を覆う場合だけ採用する。
        coverage_us = self._last_us - self._first_us + 1_000
        if coverage_us >= self._duration_us:
            self._accept_current()
        else:
            self.rejected_trailing_windows += 1

    def summary(self) -> dict[str, object]:
        self.finish()
        means = self._means.summary()
        return {
            "duration_s": self.seconds,
            "window_count": means["sample_count"],
            "axes": means["axes"],
            "valid_samples_per_window": self._sample_counts.summary(),
            "skipped_empty_windows": self.skipped_empty_windows,
            "rejected_trailing_windows": self.rejected_trailing_windows,
        }


class ImuStatistics:
    """IMU CSVと同時にstationary解析値をonline集計する。"""

    def __init__(self) -> None:
        self.records = 0
        self.invalid_gyro_samples = 0
        self.invalid_accel_samples = 0
        self.nonfinite_gyro_samples = 0
        self.nonfinite_accel_samples = 0
        self.nonmonotonic_time_samples = 0
        self._time_field = "sensor_timestamp_us"
        self._last_time_us: int | None = None
        self._gyro = VectorStatistics()
        self._accel = VectorStatistics()
        self._accel_norm = RunningStatistics()
        self._drift = LinearDrift()
        self._windows = tuple(WindowBiasStatistics(seconds) for seconds in (1, 3, 10))

    def observe(self, row: dict[str, object]) -> None:
        self.records += 1
        accel = tuple(float(row[f"accel_g_{axis}"]) for axis in AXES)
        gyro = tuple(float(row[f"gyro_dps_{axis}"]) for axis in AXES)

        if bool(row["acceleration_valid"]) and all(math.isfinite(value) for value in accel):
            self._accel.observe(accel)
            self._accel_norm.observe(math.sqrt(sum(value * value for value in accel)))
        else:
            self.invalid_accel_samples += 1
            if bool(row["acceleration_valid"]):
                self.nonfinite_accel_samples += 1

        if not bool(row["angular_velocity_valid"]) or not all(
            math.isfinite(value) for value in gyro
        ):
            self.invalid_gyro_samples += 1
            if bool(row["angular_velocity_valid"]):
                self.nonfinite_gyro_samples += 1
            return

        self._gyro.observe(gyro)
        timestamp_us = int(row[self._time_field])
        if self._last_time_us is not None and timestamp_us <= self._last_time_us:
            self.nonmonotonic_time_samples += 1
            return
        self._last_time_us = timestamp_us
        self._drift.observe(timestamp_us, gyro)
        for statistics in self._windows:
            statistics.observe(timestamp_us, gyro)

    def summary(self) -> dict[str, object]:
        return {
            "assumed_sample_rate_hz": 1_000,
            "time_source": self._time_field,
            "invalid_sample_policy": "valid flagがfalseまたは値がNaN/Infのsampleを除外",
            "standard_deviation": "標本標準偏差(n-1)",
            "record_count": self.records,
            "invalid_gyro_samples": self.invalid_gyro_samples,
            "invalid_acceleration_samples": self.invalid_accel_samples,
            "nonfinite_gyro_samples": self.nonfinite_gyro_samples,
            "nonfinite_acceleration_samples": self.nonfinite_accel_samples,
            "nonmonotonic_time_samples": self.nonmonotonic_time_samples,
            "gyro_dps": self._gyro.summary(),
            "gyro_bias_drift": self._drift.summary(),
            "acceleration_g": self._accel.summary(),
            "acceleration_norm_g": self._accel_norm.summary(),
            "gyro_bias_windows": {
                f"{statistics.seconds}_s": statistics.summary() for statistics in self._windows
            },
        }


class CsvOutput:
    """record種別ごとにCSVを遅延作成し、全frameを書き出す。"""

    def __init__(self, output_dir: Path, stem: str) -> None:
        self._output_dir = output_dir
        self._stem = stem
        self._files: dict[str, object] = {}
        self._writers: dict[str, csv.DictWriter] = {}
        self._headers: dict[str, tuple[str, ...]] = {}
        self.paths: dict[str, Path] = {}
        self.record_counts: dict[str, int] = {}
        self.decode_errors = 0
        self.imu_statistics = ImuStatistics()

    def _writer(self, name: str, fields: Iterable[str]) -> csv.DictWriter:
        field_tuple = tuple(fields)
        if name in self._writers:
            if self._headers[name] != field_tuple:
                raise RuntimeError(f"CSV schemaがcapture中に変化しました: {name}")
            return self._writers[name]
        path = self._output_dir / f"{self._stem}_{name}.csv"
        handle = path.open("x", newline="", encoding="utf-8")
        writer = csv.DictWriter(handle, fieldnames=field_tuple)
        writer.writeheader()
        self._files[name] = handle
        self._writers[name] = writer
        self._headers[name] = field_tuple
        self.paths[name] = path
        return writer

    def write(self, frame: Frame) -> None:
        schema = SCHEMAS.get(frame.record_type)
        decoded = schema.decode(frame.payload) if schema else None
        common: dict[str, object] = {
            "raw_offset": frame.raw_offset,
            "sequence": frame.sequence,
            "version": frame.version,
            "record_type": frame.record_type,
            "payload_length": len(frame.payload),
        }
        if decoded is None:
            if schema is not None:
                self.decode_errors += 1
            name = "unknown"
            row = common | {
                "known_name": schema.name if schema else "",
                "decode_error": "payload_length" if schema else "unknown_type",
                "payload_hex": frame.payload.hex(),
            }
        else:
            name = schema.name
            row = common | decoded
            if name == "imu":
                self.imu_statistics.observe(decoded)
        self._writer(name, row.keys()).writerow(row)
        self.record_counts[name] = self.record_counts.get(name, 0) + 1

    def close(self) -> None:
        for handle in self._files.values():
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()


class SequenceTracker:
    """32 bit wrapを許容して欠落・重複・逆行を数える。"""

    def __init__(self, expected_first: int | None = None) -> None:
        self.expected_first = expected_first
        self.first: int | None = None
        self.last: int | None = None
        self.gap_events = 0
        self.missing_frames = 0
        self.duplicates = 0
        self.out_of_order = 0
        self.examples: list[dict[str, int]] = []

    def observe(self, sequence: int) -> None:
        if self.first is None:
            self.first = self.last = sequence
            if self.expected_first is not None:
                delta = (sequence - self.expected_first) & 0xFFFFFFFF
                if 0 < delta < 0x80000000:
                    self.gap_events += 1
                    self.missing_frames += delta
                    self.examples.append(
                        {
                            "after": (self.expected_first - 1) & 0xFFFFFFFF,
                            "before": sequence,
                            "missing": delta,
                        }
                    )
                elif delta >= 0x80000000:
                    self.out_of_order += 1
            return
        assert self.last is not None
        delta = (sequence - self.last) & 0xFFFFFFFF
        if delta == 0:
            self.duplicates += 1
            return
        elif delta == 1:
            self.last = sequence
            return
        elif delta < 0x80000000:
            self.gap_events += 1
            self.missing_frames += delta - 1
            if len(self.examples) < 20:
                self.examples.append({"after": self.last, "before": sequence, "missing": delta - 1})
        else:
            self.out_of_order += 1
            return
        self.last = sequence


def _configure_serial(fd: int) -> list[object]:
    original = termios.tcgetattr(fd)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = SERIAL_BAUD
    attrs[5] = SERIAL_BAUD
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return original


def _write_command(fd: int, command: str, timeout_s: float = 2.0) -> None:
    if "\n" in command or "\r" in command or "\x00" in command:
        raise ValueError("commandに改行またはNULは指定できません")
    pending = memoryview((command + "\n").encode("utf-8"))
    deadline = time.monotonic() + timeout_s
    while pending:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("command送信がtimeoutしました")
        _, writable, _ = select.select([], [fd], [], remaining)
        if not writable:
            continue
        written = os.write(fd, pending)
        pending = pending[written:]


@dataclass
class MotorDisarmResult:
    attempted: bool = False
    write_completed: bool = False
    confirmed: bool = False
    confirmation: str | None = None
    firmware_result: str | None = None
    response_bytes: int = 0
    raw_bytes_saved: int = 0
    raw_save_error: str | None = None
    error: str | None = None

    def successful(self) -> bool:
        return self.confirmed and self.firmware_result == "ESP_OK"


def _motor_command_requires_disarm(
    command: str | None, *, abnormal_exit: bool = False
) -> bool:
    """駆動試験commandだけを終了時の自動disarm対象にする。"""
    if command is None:
        return False
    tokens = command.strip().split(maxsplit=1)
    if not tokens:
        return False
    return tokens[0] in MOTOR_CAPTURE_COMMANDS or (
        abnormal_exit and tokens[0] == "motor-arm"
    )


def _parse_disarm_response(data: bytes | bytearray) -> tuple[str | None, str | None]:
    request_match = DISARM_REQUEST_PATTERN.search(data)
    if request_match is not None:
        return "request_result", request_match.group(1).decode("ascii")
    return None, None


def _attempt_motor_disarm(
    fd: int, raw_file: BinaryIO | None, timeout_s: float = 1.0
) -> MotorDisarmResult:
    """受信を止めずにdisarmを送り、firmware応答を有限時間だけ待つ。"""
    result = MotorDisarmResult(attempted=True)
    deadline = time.monotonic() + timeout_s

    def save(chunk: bytes) -> None:
        result.response_bytes += len(chunk)
        if raw_file is None or result.raw_save_error is not None:
            return
        try:
            written = raw_file.write(chunk)
            if written != len(chunk):
                raise OSError("raw fileへの書込みが途中で終了しました")
            result.raw_bytes_saved += written
        except Exception as error:
            result.raw_save_error = f"{type(error).__name__}: {error}"

    # command送信前にkernelへ到着済みのbyteをrawへ移し、古い応答と区別する。
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0)
        if not readable:
            break
        try:
            chunk = os.read(fd, MAX_FEED_SIZE)
        except OSError as error:
            if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                break
            result.error = f"{type(error).__name__}: {error}"
            return result
        if not chunk:
            break
        save(chunk)

    try:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("motor-disarm送信前に確認時間を超過しました")
        _write_command(fd, "motor-disarm", timeout_s=remaining)
        result.write_completed = True
    except Exception as error:
        result.error = f"{type(error).__name__}: {error}"
        return result

    response = bytearray()
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        readable, _, _ = select.select(
            [fd], [], [], max(0.0, min(0.05, remaining))
        )
        if not readable:
            continue
        try:
            chunk = os.read(fd, MAX_FEED_SIZE)
        except OSError as error:
            if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                continue
            result.error = f"{type(error).__name__}: {error}"
            return result
        if not chunk:
            result.error = "firmware応答待機中にserial portがEOFになりました"
            return result
        save(chunk)
        response.extend(chunk)
        confirmation, firmware_result = _parse_disarm_response(response)
        if confirmation is not None:
            result.confirmed = True
            result.confirmation = confirmation
            result.firmware_result = firmware_result
            if firmware_result is not None and firmware_result != "ESP_OK":
                result.error = f"firmware motor-disarm result: {firmware_result}"
            return result
        if len(response) > 8_192:
            del response[:-8_192]

    result.error = "motor-disarmのfirmware応答を1秒以内に確認できませんでした"
    return result


def _validate_port(value: str) -> Path:
    path = Path(value)
    resolved = path.resolve(strict=True)
    if not re.fullmatch(r"/dev/ttyACM\d+", str(resolved)):
        raise ValueError("portには明示的な/dev/ttyACM<N>を指定してください")
    if not stat.S_ISCHR(resolved.stat().st_mode):
        raise ValueError(f"character deviceではありません: {resolved}")
    return resolved


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def capture(port_value: str, output_value: str, duration_s: float, command: str | None) -> int:
    if not math.isfinite(duration_s) or duration_s <= 0 or duration_s > 86_400:
        raise ValueError("durationは0より大きく86400秒以下で指定してください")
    port = _validate_port(port_value)
    output_dir = Path(output_value).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if not output_dir.is_dir():
        raise ValueError(f"output directoryではありません: {output_dir}")

    started_wall = datetime.now().astimezone()
    stem = "bringup_" + started_wall.strftime("%Y%m%dT%H%M%S_%f")
    raw_path = output_dir / f"{stem}.raw"
    summary_path = output_dir / f"{stem}_summary.json"
    csv_output = CsvOutput(output_dir, stem)
    sequences = SequenceTracker(expected_first=0 if command is not None else None)
    parser = FrameParser(lambda frame: (sequences.observe(frame.sequence), csv_output.write(frame)))
    byte_count = 0
    capture_error: str | None = None
    interrupted = False
    motor_disarm_required = _motor_command_requires_disarm(command)
    motor_disarm_reason: str | None = None
    motor_disarm = MotorDisarmResult()
    fd: int | None = None
    raw_file: BinaryIO | None = None
    original_attrs: list[object] | None = None
    monotonic_started = time.monotonic()
    capture_ended_monotonic = monotonic_started

    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        original_attrs = _configure_serial(fd)
        # command開始直後のframeも失わないよう、raw fileを先に開く。
        raw_file = raw_path.open("xb")
        if command is not None:
            _write_command(fd, command)
        monotonic_started = time.monotonic()
        deadline = monotonic_started + duration_s
        while time.monotonic() < deadline:
            timeout = min(0.2, max(0.0, deadline - time.monotonic()))
            readable, _, _ = select.select([fd], [], [], timeout)
            if not readable:
                continue
            try:
                chunk = os.read(fd, MAX_FEED_SIZE)
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    continue
                raise
            if not chunk:
                continue
            written = raw_file.write(chunk)
            if written != len(chunk):
                raise OSError("raw fileへの書込みが途中で終了しました")
            byte_count += written
    except KeyboardInterrupt:
        interrupted = True
    except Exception as error:
        capture_error = f"{type(error).__name__}: {error}"
    finally:
        capture_ended_monotonic = time.monotonic()
        if interrupted or capture_error is not None:
            # arm commandの正常終了は次の試験へ状態を渡すが、異常終了時は解除する。
            motor_disarm_required = _motor_command_requires_disarm(
                command, abnormal_exit=True
            )
        if motor_disarm_required:
            motor_disarm_reason = (
                "keyboard_interrupt"
                if interrupted
                else "capture_error"
                if capture_error is not None
                else "normal_completion"
            )
            if fd is None:
                motor_disarm.error = "serial portを開けなかったためmotor-disarmを送信できません"
            else:
                try:
                    motor_disarm = _attempt_motor_disarm(fd, raw_file, timeout_s=1.0)
                    byte_count += motor_disarm.raw_bytes_saved
                except Exception as error:
                    motor_disarm = MotorDisarmResult(
                        attempted=True,
                        error=f"{type(error).__name__}: {error}",
                    )
            if not motor_disarm.successful() and capture_error is None:
                capture_error = (
                    "MotorDisarmError: "
                    + (motor_disarm.error or "firmware応答を確認できませんでした")
                )
            if motor_disarm.raw_save_error is not None and capture_error is None:
                capture_error = "RawSaveError: " + motor_disarm.raw_save_error

        if fd is not None:
            if original_attrs is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSANOW, original_attrs)
                except termios.error:
                    pass
            try:
                os.close(fd)
            except Exception as error:
                if capture_error is None:
                    capture_error = f"{type(error).__name__}: {error}"

        if raw_file is not None:
            try:
                raw_file.flush()
                os.fsync(raw_file.fileno())
            except Exception as error:
                if capture_error is None:
                    capture_error = f"{type(error).__name__}: {error}"
            try:
                raw_file.close()
            except Exception as error:
                if capture_error is None:
                    capture_error = f"{type(error).__name__}: {error}"

        # serial取得中はraw保存だけにし、host解析の停止でTTYを
        # overflowさせない。CSV変換と統計はportを閉じた後に行う。
        if raw_path.exists():
            try:
                with raw_path.open("rb") as raw_input:
                    while chunk := raw_input.read(MAX_FEED_SIZE):
                        parser.feed(chunk)
            except Exception as error:
                if capture_error is None:
                    capture_error = f"{type(error).__name__}: {error}"
        parser.finish()
        try:
            csv_output.close()
        except Exception as error:
            if capture_error is None:
                capture_error = f"{type(error).__name__}: {error}"

    ended_wall = datetime.now().astimezone()
    summary = {
        "format": "avi_99l_bringup_capture_v1",
        "started_at": started_wall.isoformat(),
        "ended_at": ended_wall.isoformat(),
        "serial_port": str(port),
        "baud": 115200,
        "command": command,
        "requested_duration_s": duration_s,
        "actual_duration_s": capture_ended_monotonic - monotonic_started,
        "interrupted": interrupted,
        "motor_disarm_required": motor_disarm_required,
        "motor_disarm_reason": motor_disarm_reason,
        "motor_disarm_attempted": motor_disarm.attempted,
        "motor_disarm_write_completed": motor_disarm.write_completed,
        "motor_disarm_confirmed": motor_disarm.confirmed,
        "motor_disarm_confirmation": motor_disarm.confirmation,
        "motor_disarm_firmware_result": motor_disarm.firmware_result,
        "motor_disarm_response_bytes": motor_disarm.response_bytes,
        "motor_disarm_raw_bytes_saved": motor_disarm.raw_bytes_saved,
        "motor_disarm_raw_save_error": motor_disarm.raw_save_error,
        "motor_disarm_error": motor_disarm.error,
        "emergency_disarm_attempted": interrupted and motor_disarm.attempted,
        "emergency_disarm_confirmed": interrupted and motor_disarm.confirmed,
        "emergency_disarm_error": motor_disarm.error if interrupted else None,
        "capture_error": capture_error,
        "raw_file": str(raw_path),
        "raw_bytes": byte_count,
        "csv_files": {name: str(path) for name, path in csv_output.paths.items()},
        "record_counts": csv_output.record_counts,
        "valid_frames": parser.valid_frames,
        "crc_errors": parser.crc_errors,
        "malformed_headers": parser.malformed_headers,
        "decode_errors": csv_output.decode_errors,
        "discarded_bytes": parser.discarded_bytes,
        "incomplete_bytes": parser.incomplete_bytes,
        "expected_first_sequence": sequences.expected_first,
        "first_sequence": sequences.first,
        "last_sequence": sequences.last,
        "sequence_gap_events": sequences.gap_events,
        "missing_frames": sequences.missing_frames,
        "duplicate_frames": sequences.duplicates,
        "out_of_order_frames": sequences.out_of_order,
        "sequence_gap_examples": sequences.examples,
        "imu_statistics": csv_output.imu_statistics.summary(),
    }
    temporary_summary = summary_path.with_suffix(summary_path.suffix + ".tmp")
    temporary_summary.write_text(
        json.dumps(_json_safe(summary), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary_summary, summary_path)
    print(json.dumps(_json_safe(summary), ensure_ascii=False, indent=2))
    if interrupted:
        return 130
    return 1 if capture_error else 0


def _make_frame(record_type: int, sequence: int, payload: bytes) -> bytes:
    header_without_magic = struct.pack("<BBHI", PROTOCOL_VERSION, record_type, len(payload), sequence)
    crc = crc16_ccitt(header_without_magic + payload)
    return MAGIC + header_without_magic + payload + struct.pack("<H", crc)


def self_test() -> None:
    assert crc16_ccitt(b"123456789") == 0x29B1
    assert SCHEMAS[1].layout.size == 23
    assert SCHEMAS[2].layout.size == 69
    assert SCHEMAS[3].layout.size == 41
    assert SCHEMAS[4].layout.size == 78
    assert SCHEMAS[5].layout.size == 57
    encoder_payload = SCHEMAS[1].layout.pack(100, 42, 1.0, 0.1, 9, 1)
    imu_payload = SCHEMAS[2].layout.pack(
        100,
        90,
        1,
        *range(7),
        *(float(value) for value in range(7)),
        10,
        1,
        0,
        0x83,
    )
    adc_payload = SCHEMAS[3].layout.pack(123, 0x0F, 100, 0.1, 0.57, 0, 200, 0.2, 1.14, 0)
    motor_payload = SCHEMAS[4].layout.pack(
        1,
        2,
        0.05,
        1,
        100,
        12.3,
        45,
        1.1,
        2.2,
        0x800D,
        7,
        1_000,
        1,
        2,
        3,
        4,
        5,
        6,
        30,
        99,
        0x1B,
        0,
        0x01,
        0,
        0,
        0,
    )
    calibration_payload = SCHEMAS[5].layout.pack(
        10,
        3,
        0x07,
        3_000,
        0.1,
        0.2,
        0.3,
        0.0,
        0.0,
        -1.0,
        1.0,
        2.0,
        float("nan"),
        0,
    )
    frame0 = _make_frame(3, 0xFFFFFFFF, adc_payload)
    frame2 = _make_frame(99, 1, b"\x01\x02")
    damaged = bytearray(_make_frame(99, 0, b"bad"))
    damaged[-1] ^= 0x80
    frames: list[Frame] = []
    parser = FrameParser(frames.append)
    stream = b"shell output\n" + frame0 + bytes(damaged) + frame2
    for split in (stream[:3], stream[3:17], stream[17:53], stream[53:]):
        parser.feed(split)
    parser.finish()
    assert [frame.sequence for frame in frames] == [0xFFFFFFFF, 1]
    assert parser.crc_errors == 1
    assert parser.valid_frames == 2
    bulk_frames: list[Frame] = []
    bulk_parser = FrameParser(bulk_frames.append)
    bulk_parser.feed(
        b"shell\n".join(_make_frame(99, sequence, b"x" * 96)
                         for sequence in range(3))
    )
    bulk_parser.finish()
    assert [frame.sequence for frame in bulk_frames] == [0, 1, 2]
    tracker = SequenceTracker()
    for frame in frames:
        tracker.observe(frame.sequence)
    assert tracker.gap_events == 1 and tracker.missing_frames == 1
    reorder_tracker = SequenceTracker()
    for sequence in (10, 9, 11):
        reorder_tracker.observe(sequence)
    assert reorder_tracker.out_of_order == 1 and reorder_tracker.gap_events == 0
    initial_gap_tracker = SequenceTracker(expected_first=0)
    initial_gap_tracker.observe(3)
    assert initial_gap_tracker.gap_events == 1
    assert initial_gap_tracker.missing_frames == 3
    assert _motor_command_requires_disarm("motor-prbs")
    assert _motor_command_requires_disarm("combined-motor-imu-test")
    assert not _motor_command_requires_disarm("motor-arm")
    assert _motor_command_requires_disarm("motor-arm", abnormal_exit=True)
    assert not _motor_command_requires_disarm("imu-static 300")
    assert _parse_disarm_response(b"motor-disarm request: ESP_OK\n") == (
        "request_result",
        "ESP_OK",
    )
    assert _parse_disarm_response(b"command result: ESP_OK\n") == (None, None)
    assert MotorDisarmResult(
        attempted=True,
        write_completed=True,
        confirmed=True,
        confirmation="request_result",
        firmware_result="ESP_OK",
    ).successful()

    host_socket, firmware_socket = socket.socketpair()
    request_received: list[bytes] = []

    def respond_to_disarm() -> None:
        request = bytearray()
        while not request.endswith(b"\n"):
            request.extend(firmware_socket.recv(64))
        request_received.append(bytes(request))
        firmware_socket.sendall(b"motor-disarm request: ESP_OK\n")

    responder = threading.Thread(target=respond_to_disarm)
    responder.start()
    captured_response = io.BytesIO()
    try:
        helper_result = _attempt_motor_disarm(
            host_socket.fileno(), captured_response, timeout_s=0.5
        )
    finally:
        responder.join(timeout=1.0)
        host_socket.close()
        firmware_socket.close()
    assert not responder.is_alive()
    assert request_received == [b"motor-disarm\n"]
    assert helper_result.successful()
    assert captured_response.getvalue() == b"motor-disarm request: ESP_OK\n"
    decoded = SCHEMAS[3].decode(frames[0].payload)
    assert decoded is not None and decoded["logic_calibrated_valid"] is True
    assert SCHEMAS[1].decode(encoder_payload)["valid"] is True
    imu_decoded = SCHEMAS[2].decode(imu_payload)
    assert imu_decoded is not None and imu_decoded["fifo_fault"] is True
    motor_decoded = SCHEMAS[4].decode(motor_payload)
    assert motor_decoded is not None
    assert motor_decoded["drive_mode_name"] == "in1" and motor_decoded["encoder_valid"] is True
    calibration_decoded = SCHEMAS[5].decode(calibration_payload)
    assert calibration_decoded is not None
    assert calibration_decoded["launcher_tilt_valid"] is True
    assert calibration_decoded["ssc_valid"] is False

    imu_statistics = ImuStatistics()
    for index in range(10_000):
        timestamp_us = index * 1_000
        seconds = timestamp_us / 1_000_000.0
        imu_statistics.observe(
            {
                "host_timestamp_us": timestamp_us,
                "sensor_timestamp_us": timestamp_us,
                "acceleration_valid": True,
                "angular_velocity_valid": True,
                "accel_g_x": 0.0,
                "accel_g_y": 0.0,
                "accel_g_z": -1.0,
                "gyro_dps_x": 1.0 + seconds,
                "gyro_dps_y": -2.0 + 2.0 * seconds,
                "gyro_dps_z": 5.0,
            }
        )
    imu_statistics.observe(
        {
            "host_timestamp_us": 10_000_000,
            "sensor_timestamp_us": 10_000_000,
            "acceleration_valid": False,
            "angular_velocity_valid": False,
            "accel_g_x": 999.0,
            "accel_g_y": 999.0,
            "accel_g_z": 999.0,
            "gyro_dps_x": 999.0,
            "gyro_dps_y": 999.0,
            "gyro_dps_z": 999.0,
        }
    )
    statistics_summary = imu_statistics.summary()
    assert statistics_summary["invalid_gyro_samples"] == 1
    assert statistics_summary["gyro_dps"]["sample_count"] == 10_000
    assert math.isclose(
        statistics_summary["gyro_bias_drift"]["axes"]["x"]["slope_dps_per_s"],
        1.0,
        rel_tol=1e-12,
    )
    assert math.isclose(statistics_summary["acceleration_norm_g"]["mean"], 1.0)
    assert statistics_summary["gyro_bias_windows"]["1_s"]["window_count"] == 10
    assert statistics_summary["gyro_bias_windows"]["3_s"]["window_count"] == 3
    assert statistics_summary["gyro_bias_windows"]["10_s"]["window_count"] == 1
    json.dumps(_json_safe(statistics_summary), allow_nan=False)

    with tempfile.TemporaryDirectory() as directory:
        sink = CsvOutput(Path(directory), "test")
        for frame in frames:
            sink.write(frame)
        for sequence, (record_type, payload) in enumerate(
            (
                (1, encoder_payload),
                (2, imu_payload),
                (4, motor_payload),
                (5, calibration_payload),
            ),
            start=10,
        ):
            sink.write(Frame(PROTOCOL_VERSION, record_type, sequence, payload, 0))
        sink.close()
        assert set(sink.record_counts) == {
            "encoder",
            "imu",
            "adc",
            "motor",
            "calibration",
            "unknown",
        }
    print("self-test: PASS")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="明示的な/dev/ttyACM<N>")
    parser.add_argument("output_directory", nargs="?", help="保存先directory")
    parser.add_argument("--duration", type=float, help="取得時間[秒]")
    parser.add_argument("--command", help="取得開始前に送るbring-up shell command")
    parser.add_argument("--self-test", action="store_true", help="parserの内蔵試験を実行")
    args = parser.parse_args(argv)
    if args.self_test:
        if args.port or args.output_directory or args.duration is not None or args.command:
            parser.error("--self-testは他の引数と同時に指定できません")
        return args
    if args.port is None or args.output_directory is None or args.duration is None:
        parser.error("port、output_directory、--durationが必要です")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.self_test:
        self_test()
        return 0
    try:
        return capture(args.port, args.output_directory, args.duration, args.command)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
