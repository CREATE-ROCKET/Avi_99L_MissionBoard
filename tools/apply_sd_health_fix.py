from pathlib import Path


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        print(f"{label}: already applied")
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: insertion point count={count}, expected=1")
    path.write_text(text.replace(old, new, 1))
    print(f"{label}: applied")


runtime = Path("src/runtime/production_runtime.cpp")
replace_once(
    runtime,
    """        if (parachute_persistence_corrupt.load(std::memory_order_acquire))\n          persistence_flags |= 1U << 2U;\n        if (result_queue_overflow.load(std::memory_order_relaxed) != 0)\n""",
    """        if (parachute_persistence_corrupt.load(std::memory_order_acquire))\n          persistence_flags |= 1U << 2U;\n        if (sd_log_ready.load(std::memory_order_acquire) &&\n            !sd_log_failed.load(std::memory_order_acquire))\n          persistence_flags |= 1U << 3U;\n        if (result_queue_overflow.load(std::memory_order_relaxed) != 0)\n""",
    "publish Mission SD health",
)
replace_once(
    runtime,
    """      context.resources_preallocated =\n          flight_config::nonBypassFlightConfigurationReady() &&\n          flash_log_ready.load(std::memory_order_acquire) &&\n          sd_log_ready.load(std::memory_order_acquire);\n""",
    """      // loggingはflight sequence/recoveryの必須資源ではない。\n      context.resources_preallocated =\n          flight_config::nonBypassFlightConfigurationReady();\n""",
    "remove logging from StartSequence gate",
)

protocol = Path("src/protocol/can_protocol.cpp")
replace_once(
    protocol,
    "message.persistence_flags & 0x87U",
    "message.persistence_flags & 0x8FU",
    "allow PowerTime Mission SD bit",
)
replace_once(
    protocol,
    "(input.data[7] & 0x78U) != 0",
    "(input.data[7] & 0x70U) != 0",
    "reserve only PowerTime bits4..6",
)

tests = Path("host_test/mission_host_tests.cpp")
replace_once(
    tests,
    "encode(PowerTimeTelemetry{0xFC, 0xA0, 0xDC, 0xFFFA, 0x000C,\n                                        0x85})",
    "encode(PowerTimeTelemetry{0xFC, 0xA0, 0xDC, 0xFFFA, 0x000C,\n                                        0x89})",
    "PowerTime golden health flags",
)
replace_once(
    tests,
    "auto invalid_power_time = encode(PowerTimeTelemetry{1, 2, 3, 4, 5, 0x85});\n  invalid_power_time.data[7] |= 1U << 3U;",
    "auto invalid_power_time = encode(PowerTimeTelemetry{1, 2, 3, 4, 5, 0x89});\n  invalid_power_time.data[7] |= 1U << 4U;",
    "PowerTime reserved-bit test",
)
replace_once(
    tests,
    "assert(masked_power_time.data[7] == 0x85);",
    "assert(masked_power_time.data[7] == 0x8D);",
    "PowerTime mask test",
)

golden = Path("testdata/99l_protocol_golden_vectors.txt")
replace_once(
    golden,
    "CAN_103=FCA0DCFAFF0C0085",
    "CAN_103=FCA0DCFAFF0C0089",
    "PowerTime golden vector",
)

docs = Path("docs/04_通信仕様.md")
replace_once(
    docs,
    "bit2=`ParachutePersistenceCorruptLatched`、bit3..6=0 reserved、bit7=`CommandResultQueueOverflow`",
    "bit2=`ParachutePersistenceCorruptLatched`、bit3=`MissionMicroSdHealthy`、bit4..6=0 reserved、bit7=`CommandResultQueueOverflow`",
    "PowerTime Mission SD specification",
)

print("Mission SD health patch complete")
