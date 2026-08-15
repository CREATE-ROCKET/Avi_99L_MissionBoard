#!/usr/bin/env python3
"""Characterization consoleをtimestamp付きで保存し、安全なcommandだけ送信する。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import select
import stat
import sys
import termios
import time


BAUD = termios.B115200
FORBIDDEN_AUTOMATION = {"arm", "run"}


class CaptureError(RuntimeError):
    pass


class LineFramer:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[str]:
        self.buffer.extend(data)
        lines: list[str] = []
        while (newline := self.buffer.find(b"\n")) >= 0:
            raw = bytes(self.buffer[:newline]).rstrip(b"\r")
            del self.buffer[: newline + 1]
            lines.append(raw.decode("utf-8", errors="backslashreplace"))
        return lines

    def finish(self) -> list[str]:
        if not self.buffer:
            return []
        raw = bytes(self.buffer).rstrip(b"\r")
        self.buffer.clear()
        return [raw.decode("utf-8", errors="backslashreplace")]


def command_verb(command: str, *, allow_manual_motion: bool = False) -> str:
    if not command or any(not 0x20 <= ord(character) <= 0x7E for character in command):
        raise CaptureError("command must be one printable ASCII line")
    parts = command.strip().split()
    if len(parts) < 2 or parts[0] != "char":
        raise CaptureError("command must begin with 'char <verb>'")
    if not allow_manual_motion and parts[1] in FORBIDDEN_AUTOMATION:
        raise CaptureError(f"automatic '{parts[1]}' is prohibited; issue it manually while supervising hardware")
    return parts[1]


def response_result(line: str, verb: str) -> bool | None:
    parts = line.split()
    if len(parts) < 2 or parts[1] != verb:
        return None
    if parts[0] == "CHAR_OK":
        return True
    if parts[0] == "CHAR_ERROR":
        return False
    return None


def match_pending_response(line: str, pending: list[str]) -> tuple[str, bool] | None:
    for index, verb in enumerate(pending):
        result = response_result(line, verb)
        if result is not None:
            del pending[index]
            return verb, result
    return None


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _open_port(path: Path) -> int:
    metadata = path.stat()
    if not stat.S_ISCHR(metadata.st_mode):
        raise CaptureError(f"serial port is not a character device: {path}")
    descriptor = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attributes = termios.tcgetattr(descriptor)
    attributes[0] = 0
    attributes[1] = 0
    attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attributes[3] = 0
    attributes[4] = BAUD
    attributes[5] = BAUD
    attributes[6][termios.VMIN] = 0
    attributes[6][termios.VTIME] = 0
    termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
    return descriptor


def _write_line(log, direction: str, line: str) -> None:
    log.write(f"{_timestamp()} {direction} {line}\n")
    log.flush()


def _write_all(descriptor: int, payload: bytes, timeout: float) -> None:
    view = memoryview(payload)
    deadline = time.monotonic() + timeout
    while view:
        if time.monotonic() >= deadline:
            raise CaptureError("timed out writing serial command")
        try:
            written = os.write(descriptor, view)
        except BlockingIOError:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CaptureError("timed out writing serial command")
            select.select([], [descriptor], [], min(0.1, remaining))
            continue
        if written <= 0:
            raise CaptureError("serial command write made no progress")
        view = view[written:]


def _read_lines(descriptor: int, framer: LineFramer) -> list[str]:
    data = os.read(descriptor, 4096)
    if not data:
        raise CaptureError("serial port closed")
    return framer.feed(data)


def _record_interactive_rx(log, line: str, pending: list[str]) -> None:
    _write_line(log, "RX", line)
    print(line, flush=True)
    matched = match_pending_response(line, pending)
    if matched is None:
        return
    verb, accepted = matched
    _write_line(log, "MATCH", f"{verb} {'OK' if accepted else 'ERROR'}")
    if not accepted:
        print(f"firmware rejected '{verb}'", file=sys.stderr, flush=True)


def _best_effort_stop(
    descriptor: int,
    log,
    framer: LineFramer,
    pending: list[str],
    response_timeout: float,
) -> None:
    command = "char stop"
    try:
        _write_line(log, "TX", command)
    except OSError as error:
        # log filesystemが故障しても、安全停止要求は必ず試行する。
        print(f"could not record stop request: {error}", file=sys.stderr, flush=True)
    try:
        _write_all(descriptor, b"char stop\n", min(response_timeout, 0.5))
        pending.append("stop")
        deadline = time.monotonic() + min(response_timeout, 0.5)
        while "stop" in pending and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select(
                [descriptor], [], [], min(0.05, remaining)
            )
            if readable:
                for line in _read_lines(descriptor, framer):
                    _record_interactive_rx(log, line, pending)
    except (CaptureError, OSError) as error:
        try:
            _write_line(log, "HOST_ERROR", f"stop send failed: {error}")
        except OSError:
            pass
        print(f"stop request failed: {error}", file=sys.stderr, flush=True)


def _interactive_relay(
    descriptor: int,
    log,
    framer: LineFramer,
    response_timeout: float,
    deadline: float | None,
) -> None:
    pending: list[str] = []
    stdin_descriptor = sys.stdin.fileno()
    print("interactive console: enter characterization commands; Ctrl-C sends 'char stop'", flush=True)
    try:
        while deadline is None or time.monotonic() < deadline:
            timeout = (
                0.2
                if deadline is None
                else min(0.2, max(0.0, deadline - time.monotonic()))
            )
            readable, _, _ = select.select([descriptor, stdin_descriptor], [], [], timeout)
            if descriptor in readable:
                for line in _read_lines(descriptor, framer):
                    _record_interactive_rx(log, line, pending)
            if stdin_descriptor in readable:
                manual = sys.stdin.readline()
                if manual == "":
                    break
                command = manual.rstrip("\r\n")
                try:
                    verb = command_verb(command, allow_manual_motion=True)
                except CaptureError as error:
                    _write_line(log, "HOST_REJECT", str(error))
                    print(f"rejected: {error}", file=sys.stderr, flush=True)
                    continue
                _write_line(log, "TX", command)
                _write_all(descriptor, (command + "\n").encode("ascii"), response_timeout)
                pending.append(verb)
    except KeyboardInterrupt:
        print("\ninterrupt: requesting characterization stop", file=sys.stderr, flush=True)
    finally:
        # operatorがarm/run済みの可能性があるため、Ctrl-C、EOF、時間切れ、
        # host errorの全終了経路で停止を要求する。
        _best_effort_stop(descriptor, log, framer, pending, response_timeout)
    if pending:
        message = "unmatched responses: " + ", ".join(pending)
        _write_line(log, "HOST_PENDING", message)
        print(message, file=sys.stderr, flush=True)


def capture(
    port: Path,
    output: Path,
    commands: list[str],
    response_timeout: float,
    duration: float | None,
    interactive: bool = False,
) -> None:
    planned = [(command.strip(), command_verb(command)) for command in commands]
    if interactive and not sys.stdin.isatty():
        raise CaptureError("--interactive requires stdin to be a TTY")
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor = _open_port(port)
    framer = LineFramer()
    started = time.monotonic()
    try:
        with output.open("x", encoding="utf-8", newline="\n") as log:
            for command, verb in planned:
                _write_line(log, "TX", command)
                _write_all(descriptor, (command + "\n").encode("ascii"), response_timeout)
                deadline = time.monotonic() + response_timeout
                matched: bool | None = None
                while time.monotonic() < deadline and matched is None:
                    remaining = max(0.0, deadline - time.monotonic())
                    readable, _, _ = select.select([descriptor], [], [], min(0.1, remaining))
                    if not readable:
                        continue
                    for line in _read_lines(descriptor, framer):
                        _write_line(log, "RX", line)
                        if interactive:
                            print(line, flush=True)
                        result = response_result(line, verb)
                        if result is not None:
                            matched = result
                if matched is None:
                    raise CaptureError(f"timed out waiting for response to '{verb}'")
                if not matched:
                    raise CaptureError(f"firmware rejected '{verb}'")

            deadline = None if duration is None else started + duration
            if interactive:
                _interactive_relay(descriptor, log, framer, response_timeout, deadline)
            else:
                assert deadline is not None
                while time.monotonic() < deadline:
                    remaining = max(0.0, deadline - time.monotonic())
                    readable, _, _ = select.select([descriptor], [], [], min(0.2, remaining))
                    if readable:
                        for line in _read_lines(descriptor, framer):
                            _write_line(log, "RX", line)
            for line in framer.finish():
                _write_line(log, "RX", line)
                if interactive:
                    print(line, flush=True)
    finally:
        os.close(descriptor)


def self_test() -> None:
    framer = LineFramer()
    assert framer.feed(b"boot\r\nCHAR_OK sta") == ["boot"]
    assert framer.feed(b"tus ready\n") == ["CHAR_OK status ready"]
    assert command_verb("char status") == "status"
    assert response_result("CHAR_OK status ready", "status") is True
    assert response_result("CHAR_ERROR status ESP_FAIL", "status") is False
    assert response_result("CHAR_OK rate-check", "status") is None
    pending = ["status", "rate-check"]
    assert match_pending_response("CHAR_OK rate-check accepted", pending) == ("rate-check", True)
    assert pending == ["status"]
    assert match_pending_response("CHAR_ERROR status ESP_FAIL", pending) == ("status", False)
    assert not pending
    for command in ("char arm session", "char run full 1000"):
        try:
            command_verb(command)
        except CaptureError:
            pass
        else:
            raise AssertionError(f"unsafe automation was accepted: {command}")
    assert command_verb("char arm session", allow_manual_motion=True) == "arm"
    assert command_verb("char run full 1000", allow_manual_motion=True) == "run"
    for command in ("char status\nchar arm session", "char status\rchar run full 1000", "char status\0"):
        try:
            command_verb(command)
        except CaptureError:
            pass
        else:
            raise AssertionError(f"multi-line/control-byte command was accepted: {command!r}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", type=Path)
    parser.add_argument("output", nargs="?", type=Path)
    parser.add_argument("--command", action="append", default=[])
    parser.add_argument("--response-timeout", type=float, default=5.0)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.self_test:
            self_test()
            print("capture_characterization self-test: PASS")
            return 0
        if args.port is None or args.output is None:
            raise CaptureError("port and output are required")
        if args.response_timeout <= 0 or (args.duration is not None and args.duration < 0):
            raise CaptureError("timeouts must be positive")
        duration = args.duration if args.duration is not None else (None if args.interactive else 10.0)
        capture(
            args.port,
            args.output,
            args.command,
            args.response_timeout,
            duration,
            args.interactive,
        )
        print(f"capture saved: {args.output}")
        return 0
    except (CaptureError, OSError, UnicodeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
