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

# ActuatorEmergencyStopはfinをFreeへ倒すが、パラシュート電源は落とさずHoldを維持する。
s = s.replace(
    "std::atomic<bool> emergency_power_safe_requested{false};\n", "")
s = s.replace(
    "  if (xQueueSendToFront(emergency_queue, &envelope, 0) == pdTRUE) {\n"
    "    if (!liftoff)\n"
    "      emergency_power_safe_requested.store(true, std::memory_order_release);\n"
    "    return;\n"
    "  }",
    "  if (xQueueSendToFront(emergency_queue, &envelope, 0) == pdTRUE)\n"
    "    return;")
s = s.replace(
    "  if (actuator_emergency_latch.signal(transaction_id))\n"
    "    emergency_metadata_overflow.fetch_add(1, std::memory_order_relaxed);\n"
    "  emergency_power_safe_requested.store(true, std::memory_order_release);\n",
    "  if (actuator_emergency_latch.signal(transaction_id))\n"
    "    emergency_metadata_overflow.fetch_add(1, std::memory_order_relaxed);\n")
s, emergency_block_count = re.subn(
    r"\n    if \(emergency_power_safe_requested\.exchange\(false,\n"
    r"\s*std::memory_order_acq_rel\)\) \{.*?\n    \}\n",
    "\n", s, count=1, flags=re.S)
if emergency_block_count != 1:
    raise RuntimeError(
        f"emergency parachute power-off block: {emergency_block_count}")

# CommandReceiveでは1秒、flight中は既存20ms retry intervalを使う。
reconnect_anchor = (
    "  constexpr uint64_t kTelemetryIntervalUs = 500'000;\n"
    "  constexpr uint64_t kMotionTimeoutUs = 3'000'000;\n")
reconnect_code = reconnect_anchor + '''\n  auto reconnectIntervalMs = [&]() {
    protocol::MissionState state = protocol::MissionState::unknown;
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      state = state_machine.snapshot().state;
      xSemaphoreGive(state_mutex);
    }
    return state == protocol::MissionState::command_receive
               ? flight_config::kParachuteCommandReceiveReconnectMs
               : flight_config::kParachute.retry_interval_ms;
  };
'''
if reconnect_anchor not in s:
    raise RuntimeError("parachute reconnect anchor not found")
s = s.replace(reconnect_anchor, reconnect_code, 1)
s = s.replace(
    "static_cast<uint64_t>(\n                       flight_config::kParachuteCommandReceiveReconnectMs)",
    "static_cast<uint64_t>(reconnectIntervalMs())", 1)
s = s.replace(
    "static_cast<uint64_t>(\n                     flight_config::kParachuteCommandReceiveReconnectMs)",
    "static_cast<uint64_t>(reconnectIntervalMs())", 2)

# 相対move前に必ず現在位置Holdを成立させ、Torque ON状態から1回だけmoveを発行する。
old_move = '''    // 相対移動は1 operationにつき1回だけ発行する。retryで再送しない。
    const esp_err_t result =
        servo.moveRelativeDegrees(delta_degrees, motion());
    if (result != ESP_OK) {'''
new_move = '''    // 相対移動は1 operationにつき1回だけ発行する。retryで再送しない。
    // fresh接続直後でもTorque OFFのままtargetだけ書かないよう、先にHoldを確立する。
    esp_err_t result = establishHold();
    if (result == ESP_OK)
      result = servo.moveRelativeDegrees(delta_degrees, motion());
    if (result != ESP_OK) {'''
if old_move not in s:
    raise RuntimeError("relative move anchor not found")
s = s.replace(old_move, new_move, 1)

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
    "emergency_power_safe_requested",
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
