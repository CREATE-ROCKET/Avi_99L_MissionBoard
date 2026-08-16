from pathlib import Path
import re

runtime_path = Path("src/runtime/production_runtime.cpp")
s = runtime_path.read_text()

# 旧endpoint readiness代入を機械置換残骸ごと除去する。
s = s.replace(
    "        readiness.fin_zero_configured =\n"
    "            fin_zero_configured.load(std::memory_order_acquire);\n"
    "            parachute_open_configured.load(std::memory_order_acquire);\n"
    "            parachute_close_configured.load(std::memory_order_acquire);\n",
    "        readiness.fin_zero_configured =\n"
    "            fin_zero_configured.load(std::memory_order_acquire);\n")

# CommandContextから削除済みのNVS gate代入残骸を除去する。
s = s.replace(
    "      context.resources_preallocated =\n"
    "          flight_config::nonBypassFlightConfigurationReady();\n"
    "          parachute_config_load_complete.load(std::memory_order_acquire);\n"
    "          parachute_persistence_ready.load(std::memory_order_acquire);\n",
    "      context.resources_preallocated =\n"
    "          flight_config::nonBypassFlightConfigurationReady();\n")

# queue初期化の連結残骸を削除する。
s, n = re.subn(
    r"(&parachute_start_response_queue_storage\);)\s*"
    r"parachute_persistence_response_queue = xQueueCreateStatic\(\s*"
    r"4, sizeof\(ParachutePersistenceResponse\),\s*"
    r"parachute_persistence_response_queue_buffer\.data\(\),\s*"
    r"&parachute_persistence_response_queue_storage\);",
    r"\1",
    s,
    count=1,
    flags=re.S,
)
if n != 1:
    raise RuntimeError(f"parachute persistence queue residual: {n}")

# 廃止したNVS/absolute endpoint経路がproduction runtimeに残らないことを保証する。
for forbidden in [
    "ParachutePersistenceRequest",
    "ParachutePersistenceResponse",
    "parachute_persistence_",
    "parachute_config_load_complete",
    "parachute_open_configured",
    "parachute_close_configured",
    "kParachuteNvsNamespace",
    "nvs_open(",
    "nvs_set_blob(",
    "nvs_get_blob(",
    "nvs_flash_init(",
    "configureMultiTurnPositionMode",
    "servo.disableTorque()",
    "loadFlightParachuteConfiguration",
    "storeFlightParachuteConfiguration",
    "clearFlightParachuteConfiguration",
]:
    if forbidden in s:
        raise RuntimeError(f"runtime forbidden token remains: {forbidden}")

runtime_path.write_text(s)

# PreflightReadinessSnapshotはendpoint 2 bit削除後の5 bit readinessへ移行した。
old_return = "return {1, 1234, true, true, true, true, true, true, true, true};"
new_return = "return {1, 1234, true, true, true, true, true, true};"
old_const = (
    "const PreflightReadinessSnapshot ready{1, 1234, true, true, true, true,\n"
    "                                         true, true, true, true};"
)
new_const = (
    "const PreflightReadinessSnapshot ready{1, 1234, true, true, true, true,\n"
    "                                         true, true};"
)

for path in Path("host_test").glob("*.cpp"):
    t = path.read_text()
    t = t.replace(old_return, new_return)
    t = t.replace(old_const, new_const)
    t = t.replace("  context.persistence_load_complete = true;\n", "")
    t = t.replace("  context.persistence_runtime_available = true;\n", "")
    path.write_text(t)

mission = Path("host_test/mission_host_tests.cpp")
t = mission.read_text()
t = t.replace("  missing.parachute_close_configured = false;\n", "")
t = t.replace(
    "  assert(missing.missingMask() == ((1U << 2U) | (1U << 4U)));",
    "  assert(missing.missingMask() == (1U << 2U));")

# retired command testは専用executorを使う。
t = t.replace(
    "  // 旧パラシュートcommandはstate/argsより先にNotSupportedとし、\n"
    "  // 同一transaction replayでもservo操作へ到達させない。\n"
    "  for (uint8_t raw = static_cast<uint8_t>(CommandCode::para_free);",
    "  // 旧パラシュートcommandはstate/argsより先にNotSupportedとし、\n"
    "  // 同一transaction replayでもservo操作へ到達させない。\n"
    "  CommandExecutor retired_executor;\n"
    "  for (uint8_t raw = static_cast<uint8_t>(CommandCode::para_free);")
t = t.replace("executor.begin(retired, retired_context)",
              "retired_executor.begin(retired, retired_context)")

# endpoint NVS load/runtime failureでStartを拒否する旧testは削除する。
t, n_load = re.subn(
    r"\n  CommandExecutor load_gate;\n.*?CommandReason::busy\);\n",
    "\n", t, count=1, flags=re.S)
t, n_persist = re.subn(
    r"\n  CommandExecutor persistence_gate;\n.*?CommandReason::persistence_error\);\n",
    "\n", t, count=1, flags=re.S)
if n_load != 1 or n_persist != 1:
    raise RuntimeError(
        f"old persistence gate tests not removed: load={n_load} persist={n_persist}")
mission.write_text(t)

# fin overtravel testのCommandContextから旧NVS gateを除去する。
fin = Path("host_test/fin_overtravel_tests.cpp")
t = fin.read_text()
t = t.replace("  context.persistence_load_complete = true;\n", "")
t = t.replace("  context.persistence_runtime_available = true;\n", "")
fin.write_text(t)

# 旧readiness field参照がhost testに残っていないことを確認する。
for path in Path("host_test").glob("*.cpp"):
    t = path.read_text()
    for forbidden in [
        "parachute_open_configured",
        "parachute_close_configured",
        "persistence_load_complete",
        "persistence_runtime_available",
        "ParachuteBlob",
        "ParachuteEndpoint",
        "ParachuteConfigurationState",
        "FlightParachuteConfiguration",
    ]:
        if forbidden in t:
            raise RuntimeError(f"{path}: old test token remains: {forbidden}")
