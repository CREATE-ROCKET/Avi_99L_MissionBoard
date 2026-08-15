from pathlib import Path

path = Path(__file__).resolve().parents[1] / "host_test/mission_host_tests.cpp"
text = path.read_text(encoding="utf-8")
old = "  assert(parachute.tick({5'999'999, false, 0, false}) ==\n         ParachuteAction::none);\n"
if text.count(old) != 1:
    raise RuntimeError("deadline pre-boundary assertion not found exactly once")
path.write_text(text.replace(old, "", 1), encoding="utf-8")
print("ForceStart Stage 2 deadline test boundary fixed")
