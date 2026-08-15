from pathlib import Path

path = Path("src/runtime/production_runtime.cpp")
text = path.read_text(encoding="utf-8")
old = "if (++flash_decimation >= 10U)"
new = "if (++flash_decimation >= 20U)"
if text.count(old) != 1:
    raise SystemExit("flash backup decimation anchor was not unique")
path.write_text(text.replace(old, new, 1), encoding="utf-8")
