from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def replace_once(path: str, old: str, new: str) -> None:
    p = ROOT / path
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old[:120]!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_once(
    "src/runtime/recovery_boot.hpp",
    "namespace runtime::recovery_boot {\n\n[[nodiscard]] bool markerValid();",
    "namespace runtime::recovery_boot {\n\n"
    "struct StartReadinessAudit {\n"
    "  bool valid{};\n"
    "  bool forced{};\n"
    "  uint32_t generation{};\n"
    "  uint64_t captured_at_us{};\n"
    "  uint8_t ready_mask{};\n"
    "  uint8_t missing_mask{};\n"
    "};\n\n"
    "[[nodiscard]] bool loadStartReadinessAudit(StartReadinessAudit &audit);\n"
    "void storeStartReadinessAudit(\n"
    "    const mission::PreflightReadinessSnapshot &readiness, bool forced);\n"
    "void clearStartReadinessAudit();\n\n"
    "[[nodiscard]] bool markerValid();",
)

replace_once(
    "src/runtime/recovery_boot.cpp",
    "RTC_DATA_ATTR ParachuteCheckpointRecord parachute_checkpoint{};\n\n"
    "uint16_t parachuteCheckpointChecksum",
    "RTC_DATA_ATTR ParachuteCheckpointRecord parachute_checkpoint{};\n\n"
    "constexpr uint32_t kStartReadinessAuditMagic = 0x39395341;\n"
    "constexpr uint8_t kStartReadinessAuditVersion = 1;\n"
    "struct StartReadinessAuditRecord {\n"
    "  uint32_t magic{};\n"
    "  uint8_t version{};\n"
    "  uint8_t ready_mask{};\n"
    "  uint8_t missing_mask{};\n"
    "  bool forced{};\n"
    "  uint32_t generation{};\n"
    "  uint64_t captured_at_us{};\n"
    "  uint16_t checksum{};\n"
    "};\n"
    "RTC_DATA_ATTR StartReadinessAuditRecord start_readiness_audit{};\n\n"
    "uint16_t startReadinessAuditChecksum(\n"
    "    const StartReadinessAuditRecord &record) {\n"
    "  uint64_t value = record.magic ^\n"
    "                   (static_cast<uint32_t>(record.version) << 24U) ^\n"
    "                   (static_cast<uint32_t>(record.ready_mask) << 16U) ^\n"
    "                   (static_cast<uint32_t>(record.missing_mask) << 8U) ^\n"
    "                   record.generation ^ record.captured_at_us ^\n"
    "                   (record.captured_at_us >> 32U) ^\n"
    "                   (record.forced ? 0x5AA5U : 0xA55AU);\n"
    "  value ^= value >> 32U;\n"
    "  value ^= value >> 16U;\n"
    "  return static_cast<uint16_t>(value);\n"
    "}\n\n"
    "bool validStartReadinessAudit(const StartReadinessAuditRecord &record) {\n"
    "  const uint8_t masks = static_cast<uint8_t>(\n"
    "      (record.ready_mask | record.missing_mask) &\n"
    "      mission::kPreflightReadinessMask);\n"
    "  return record.magic == kStartReadinessAuditMagic &&\n"
    "         record.version == kStartReadinessAuditVersion &&\n"
    "         record.generation != 0 &&\n"
    "         masks == mission::kPreflightReadinessMask &&\n"
    "         (record.ready_mask & record.missing_mask) == 0 &&\n"
    "         record.checksum == startReadinessAuditChecksum(record);\n"
    "}\n\n"
    "uint16_t parachuteCheckpointChecksum",
)

replace_once(
    "src/runtime/recovery_boot.cpp",
    "bool markerValid() { return mission::validRecoveryMarker(recovery_marker); }",
    "bool loadStartReadinessAudit(StartReadinessAudit &audit) {\n"
    "  audit = {};\n"
    "  if (!resetPreservesRtcMemory() ||\n"
    "      !validStartReadinessAudit(start_readiness_audit))\n"
    "    return false;\n"
    "  audit.valid = true;\n"
    "  audit.forced = start_readiness_audit.forced;\n"
    "  audit.generation = start_readiness_audit.generation;\n"
    "  audit.captured_at_us = start_readiness_audit.captured_at_us;\n"
    "  audit.ready_mask = start_readiness_audit.ready_mask;\n"
    "  audit.missing_mask = start_readiness_audit.missing_mask;\n"
    "  return true;\n"
    "}\n\n"
    "void storeStartReadinessAudit(\n"
    "    const mission::PreflightReadinessSnapshot &readiness, bool forced) {\n"
    "  StartReadinessAuditRecord record{};\n"
    "  record.magic = kStartReadinessAuditMagic;\n"
    "  record.version = kStartReadinessAuditVersion;\n"
    "  record.ready_mask = readiness.readyMask();\n"
    "  record.missing_mask = readiness.missingMask();\n"
    "  record.forced = forced;\n"
    "  record.generation = readiness.generation;\n"
    "  record.captured_at_us = readiness.captured_at_us;\n"
    "  record.checksum = startReadinessAuditChecksum(record);\n"
    "  start_readiness_audit = record;\n"
    "}\n\n"
    "void clearStartReadinessAudit() { start_readiness_audit = {}; }\n\n"
    "bool markerValid() { return mission::validRecoveryMarker(recovery_marker); }",
)

replace_once(
    "src/runtime/production_runtime.cpp",
    "        command_envelope.readiness = readiness;\n"
    "        const ParachuteCommandRequest preparation{",
    "        command_envelope.readiness = readiness;\n"
    "        const bool forced_start =\n"
    "            code == mission::CommandCode::force_start_sequence;\n"
    "        recovery_boot::storeStartReadinessAudit(readiness, forced_start);\n"
    "        std::printf(\n"
    "            \"preflight readiness accepted: forced=%u generation=%lu captured_us=%llu ready=0x%02X missing=0x%02X\\n\",\n"
    "            forced_start ? 1U : 0U,\n"
    "            static_cast<unsigned long>(readiness.generation),\n"
    "            static_cast<unsigned long long>(readiness.captured_at_us),\n"
    "            static_cast<unsigned>(readiness.readyMask()),\n"
    "            static_cast<unsigned>(readiness.missingMask()));\n"
    "        const ParachuteCommandRequest preparation{",
)

replace_once(
    "src/runtime/production_runtime.cpp",
    "        if (transition == mission::TransitionResult::completed) {\n"
    "          post_transition_para = {ParaRequest::Kind::discard_snapshot,",
    "        if (transition == mission::TransitionResult::completed) {\n"
    "          recovery_boot::clearStartReadinessAudit();\n"
    "          post_transition_para = {ParaRequest::Kind::discard_snapshot,",
)

replace_once(
    "src/runtime/production_runtime.cpp",
    "  if (!recovery_only_) {\n"
    "    mission::ResetCheckpoint checkpoint{};",
    "  if (!recovery_only_) {\n"
    "    recovery_boot::StartReadinessAudit readiness_audit{};\n"
    "    if (recovery_boot::loadStartReadinessAudit(readiness_audit)) {\n"
    "      std::printf(\n"
    "          \"RTC preflight audit restored: forced=%u generation=%lu captured_us=%llu ready=0x%02X missing=0x%02X\\n\",\n"
    "          readiness_audit.forced ? 1U : 0U,\n"
    "          static_cast<unsigned long>(readiness_audit.generation),\n"
    "          static_cast<unsigned long long>(readiness_audit.captured_at_us),\n"
    "          static_cast<unsigned>(readiness_audit.ready_mask),\n"
    "          static_cast<unsigned>(readiness_audit.missing_mask));\n"
    "    }\n"
    "    mission::ResetCheckpoint checkpoint{};",
)

replace_once(
    "src/runtime/production_runtime.cpp",
    "        const protocol::PowerTimeTelemetry power{\n"
    "            sequences.next(protocol::CanId::power_time_telemetry), 0xFF, 0xFF,",
    "        uint8_t persistence_flags = 0;\n"
    "        if (parachute_config_load_complete.load(std::memory_order_acquire))\n"
    "          persistence_flags |= 1U << 0U;\n"
    "        if (parachute_persistence_ready.load(std::memory_order_acquire))\n"
    "          persistence_flags |= 1U << 1U;\n"
    "        if (parachute_persistence_corrupt.load(std::memory_order_acquire))\n"
    "          persistence_flags |= 1U << 2U;\n"
    "        if (result_queue_overflow.load(std::memory_order_relaxed) != 0)\n"
    "          persistence_flags |= 1U << 7U;\n"
    "        const protocol::PowerTimeTelemetry power{\n"
    "            sequences.next(protocol::CanId::power_time_telemetry), 0xFF, 0xFF,",
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "            0xFFF1,\n"
    "            static_cast<uint8_t>(result_queue_overflow.load(\n"
    "                                     std::memory_order_relaxed) != 0\n"
    "                                     ? 0x80\n"
    "                                     : 0)};",
    "            0xFFF1, persistence_flags};",
)

p = ROOT / "README.md"
text = p.read_text(encoding="utf-8")
needle = "- `forced_start`とpreflight snapshot/missing mask、optional parachute snapshotはsoftware/watchdog reset時にRTC checkpointから復元します。\n"
replacement = needle + (
    "- Start/Force受理時の同一`PreflightReadinessSnapshot`はRTC auditにも保存し、software/watchdog reset後に検証して内部logへ復元します。\n"
    "- `PowerTimeTelemetry.persistence_flags`のbit0/1/2でparachute NVS load完了/runtime ready/corruptを公開します。\n"
)
if text.count(needle) != 1:
    raise RuntimeError("README Stage 2 audit insertion point not found")
p.write_text(text.replace(needle, replacement, 1), encoding="utf-8")

print("ForceStart Stage 2 accepted audit and persistence health applied")
