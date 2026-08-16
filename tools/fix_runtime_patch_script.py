from pathlib import Path

p = Path("tools/apply_runtime_v2.py")
s = p.read_text()
marker = 'm = re.search(r"\\nconstexpr char kParachuteNvsNamespace'
pos = s.find(marker)
if pos < 0:
    raise RuntimeError("runtime cleanup tail marker not found")

new_tail = r'''
# NVS endpoint helper群を丸ごと削除する。
helper_start = s.find("constexpr char kParachuteNvsNamespace[]")
if helper_start >= 0:
    task_start = s.find("void internalFlashTask(void *) {", helper_start)
    if task_start < 0:
        raise RuntimeError("internalFlashTask marker not found")
    s = s[:helper_start] + s[task_start:]

# InternalFlashTaskはflight log/recoveryだけを所有し、para endpoint NVSを扱わない。
internal_anchor = "void internalFlashTask(void *) {\n  addWatchdog();"
internal_pos = s.find(internal_anchor)
if internal_pos < 0:
    raise RuntimeError("internalFlashTask anchor not found")
body_start = internal_pos + len(internal_anchor)
recovery_pos = s.find("\n  mission::RecoveryRuntime recovery;", body_start)
if recovery_pos < 0:
    raise RuntimeError("RecoveryRuntime anchor not found")
s = s[:body_start] + s[recovery_pos:]

persist_loop = s.find("\n    ParachutePersistenceRequest persistence_request{};", internal_pos)
if persist_loop >= 0:
    recovery_loop = s.find("\n    RecoveryRequest request{};", persist_loop)
    if recovery_loop < 0:
        raise RuntimeError("RecoveryRequest loop anchor not found")
    s = s[:persist_loop] + s[recovery_loop:]

# Queue storage / buffer / handle / initializationを除去する。
for exact in [
    "StaticQueue_t parachute_persistence_request_queue_storage;\n",
    "StaticQueue_t parachute_persistence_response_queue_storage;\n",
    "QueueHandle_t parachute_persistence_request_queue{};\n",
    "QueueHandle_t parachute_persistence_response_queue{};\n",
    "std::atomic<bool> parachute_config_load_complete{false};\n",
    "std::atomic<bool> parachute_persistence_ready{false};\n",
    "std::atomic<bool> parachute_persistence_corrupt{false};\n",
    "std::atomic<bool> parachute_open_configured{false};\n",
    "std::atomic<bool> parachute_close_configured{false};\n",
]:
    s = s.replace(exact, "")

for typename, varname in [
    ("ParachutePersistenceRequest", "parachute_persistence_request_queue_buffer"),
    ("ParachutePersistenceResponse", "parachute_persistence_response_queue_buffer"),
]:
    block = (f"std::array<uint8_t, sizeof({typename}) * 4>\n"
             f"    {varname}{{}};\n")
    s = s.replace(block, "")

for qname in ["parachute_persistence_request_queue", "parachute_persistence_response_queue"]:
    begin = s.find(f"\n  {qname} = xQueueCreateStatic(")
    if begin >= 0:
        end = s.find(");\n", begin)
        if end < 0:
            raise RuntimeError(f"{qname} initialization terminator not found")
        s = s[:begin] + s[end + 3:]
    s = s.replace(f"      {qname} == nullptr ||\n", "")

# endpoint readinessはStart条件から除外する。
s = s.replace(
    "        readiness.parachute_open_configured =\n"
    "            parachute_open_configured.load(std::memory_order_acquire);\n",
    "")
s = s.replace(
    "        readiness.parachute_close_configured =\n"
    "            parachute_close_configured.load(std::memory_order_acquire);\n",
    "")
s = s.replace(
    "      context.persistence_load_complete =\n"
    "          parachute_config_load_complete.load(std::memory_order_acquire);\n",
    "")
s = s.replace(
    "      context.persistence_runtime_available =\n"
    "          parachute_persistence_ready.load(std::memory_order_acquire);\n",
    "")

# MissionStatus.config_flags bit2は旧endpoint configured。予約0へ戻す。
s = s.replace(
    "        (parachute_open_configured.load(std::memory_order_acquire) &&\n"
    "                 parachute_close_configured.load(std::memory_order_acquire)\n"
    "             ? (1U << 2U)\n"
    "             : 0U) |\n",
    "")

# Descent status bit4は旧NVS corruption。予約0へ戻す。
s = s.replace(
    "          const uint16_t persistence_corrupt =\n"
    "              parachute_persistence_corrupt.load(std::memory_order_acquire)\n"
    "                  ? uint16_t{1U << 4U}\n"
    "                  : uint16_t{0};\n",
    "")
s = s.replace("static_cast<uint16_t>(failure | persistence_corrupt)", "failure")

# PowerTime persistence_flags bit0..2は予約0。SD healthy bit3以降は維持する。
s = s.replace(
    "        if (parachute_config_load_complete.load(std::memory_order_acquire))\n"
    "          persistence_flags |= 1U << 0U;\n"
    "        if (parachute_persistence_ready.load(std::memory_order_acquire))\n"
    "          persistence_flags |= 1U << 1U;\n"
    "        if (parachute_persistence_corrupt.load(std::memory_order_acquire))\n"
    "          persistence_flags |= 1U << 2U;\n",
    "")

for forbidden in [
    "ParachutePersistenceRequest",
    "ParachutePersistenceResponse",
    "parachute_persistence_",
    "parachute_config_load_complete",
    "parachute_open_configured",
    "parachute_close_configured",
    "kParachuteNvsNamespace",
    "nvs_open(",
    "nvs_flash_init(",
    "configureMultiTurnPositionMode",
    "servo.disableTorque()",
    "loadFlightParachuteConfiguration",
    "storeFlightParachuteConfiguration",
    "clearFlightParachuteConfiguration",
]:
    if forbidden in s:
        raise RuntimeError(f"forbidden token remains: {forbidden}")

path.write_text(s)
'''

p.write_text(s[:pos] + new_tail)
