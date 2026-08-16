#!/usr/bin/env python3
"""1 kHz motor-ID用99L characterization V5を厳密検証する。"""

from __future__ import annotations

import verify_characterization as _base


# V5 base validatorがmotor-IDの新しいtiming契約を直接扱う。
# 100 usは診断target、full captureでは同一1 ms epoch内の遅延を許容し、
# epochを跨ぐcommand applyだけをfatalなDeadlineとして扱う。
ValidationError = _base.ValidationError
build_golden_fixture = _base.build_golden_fixture
verify_bytes = _base.verify_bytes
verify_file = _base.verify_file


def main(argv: list[str] | None = None) -> int:
    return _base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
