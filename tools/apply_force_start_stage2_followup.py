from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def replace_once(path: str, old: str, new: str) -> None:
    p = ROOT / path
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old[:100]!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


# MotorProfileValidだけはForceでbypass可能にし、それ以外のcompile-time/runtime
# configuration不整合はresources/runtime invariantとしてStart/Force共通で拒否する。
p = ROOT / "src/config/flight_config.hpp"
text = p.read_text(encoding="utf-8")
pattern = re.compile(
    r"\[\[nodiscard\]\] inline bool productionFlightConfigurationReady\(\) \{\n"
    r"  return motorProfileValid\(\) &&\n"
    r"(?P<body>.*?)\n\}\n\n\} // namespace flight_config",
    re.S,
)
match = pattern.search(text)
if match is None:
    raise RuntimeError("flight_config production readiness function not found")
body = match.group("body")
replacement = (
    "[[nodiscard]] inline bool nonBypassFlightConfigurationReady() {\n"
    "  return\n" + body + "\n}\n\n"
    "[[nodiscard]] inline bool productionFlightConfigurationReady() {\n"
    "  return motorProfileValid() && nonBypassFlightConfigurationReady();\n"
    "}\n\n} // namespace flight_config"
)
text = text[: match.start()] + replacement + text[match.end() :]
p.write_text(text, encoding="utf-8")

# persistence corruptは既存persistence_error event/Healthへ残すが、deployment failureの
# first-failure slotを消費しない。Open等の実deployment failureを後から記録可能にする。
replace_once(
    "src/runtime/production_runtime.cpp",
    "        if (persistence_response.corruption_detected)\n"
    "          {\n"
    "            uint8_t expected = 0;\n"
    "            (void)parachute_deployment_failure.compare_exchange_strong(\n"
    "                expected, static_cast<uint8_t>(\n"
    "                              ParachuteDeploymentFailure::persistence_corrupt));\n"
    "          }\n",
    "",
)

# preallocation/static config gateをForceでもbypassしない。
replace_once(
    "src/runtime/production_runtime.cpp",
    "      context.resources_preallocated = true;",
    "      context.resources_preallocated =\n"
    "          flight_config::nonBypassFlightConfigurationReady();",
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "        readiness.resources_preallocated = true;",
    "        readiness.resources_preallocated =\n"
    "            flight_config::nonBypassFlightConfigurationReady();",
)

# wireには一度だけ出すが、内部logでは発生したfailure種別を毎回区別する。
replace_once(
    "src/runtime/production_runtime.cpp",
    "    auto recordParachuteFailure = [&](ParachuteDeploymentFailure failure,\n"
    "                                      uint16_t detail = 0) {\n"
    "      uint8_t expected = 0;",
    "    auto recordParachuteFailure = [&](ParachuteDeploymentFailure failure,\n"
    "                                      uint16_t detail = 0) {\n"
    "      std::printf(\"parachute deployment failure: code=%u detail=%u\\n\",\n"
    "                  static_cast<unsigned>(failure),\n"
    "                  static_cast<unsigned>(detail));\n"
    "      uint8_t expected = 0;",
)

# persistence corruption stateはHealth/telemetryへ残すためatomic自体は維持する。
# recovery-only bootではParachuteTaskを起動しないのでCommandReceive Hold既定値は影響しない。

print("ForceStart/Parachute Stage 2 follow-up transform applied")
