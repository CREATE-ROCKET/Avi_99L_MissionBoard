from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path}: 置換対象が一意ではありません: count={count}\n{old[:200]}")
    target.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_once(
    "src/runtime/production_runtime.cpp",
    """        recovery_entry_deadline_us =
            static_cast<uint64_t>(esp_timer_get_time()) + 2'000'000;
""",
    """        recovery_entry_deadline_us =
            static_cast<uint64_t>(esp_timer_get_time()) + 1'000'000;
""",
)

replace_once(
    "src/protocol/can_protocol.cpp",
    """  result.data[7] = message.persistence_flags;
""",
    """  result.data[7] = static_cast<uint8_t>(message.persistence_flags & 0x87U);
""",
)

replace_once(
    "testdata/99l_protocol_golden_vectors.txt",
    """# Vault source commit: 2a6fa974a9b7a50a9b9d574174262068e2e5b8bf
""",
    """# Vault source commit: cbf6e6128f963a36a94941a3d9770988a6027356
""",
)
replace_once(
    "testdata/99l_protocol_golden_vectors.txt",
    """CAN_013=070278563412E703
CAN_020=FF614003D204EFBE
""",
    """CAN_013=070278563412E703
CAN_014=440101
CAN_020=FF614003D204EFBE
""",
)
replace_once(
    "testdata/99l_protocol_golden_vectors.txt",
    """CAN_103=FCA0DCFAFF0C0065
CAN_104=FB551AF7
""",
    """CAN_103=FCA0DCFAFF0C0085
CAN_104=FB1500F7
""",
)

replace_once(
    "host_test/mission_host_tests.cpp",
    """  expectFrame(golden, \"CAN_013\",
              encode(TimeResponse{7, TimeSource::ground, 0x12345678, 999}));
  expectFrame(golden, \"CAN_020\",
""",
    """  expectFrame(golden, \"CAN_013\",
              encode(TimeResponse{7, TimeSource::ground, 0x12345678, 999}));
  expectFrame(golden, \"CAN_014\",
              encode(RecoveryModeCommand{0x44,
                                         RecoveryMode::enter_recovery_beacon,
                                         RecoveryModeReason::ground_requested}));
  expectFrame(golden, \"CAN_020\",
""",
)
replace_once(
    "host_test/mission_host_tests.cpp",
    """  expectFrame(golden, \"CAN_103\",
              encode(PowerTimeTelemetry{0xFC, 0xA0, 0xDC, 0xFFFA, 0x000C,
                                        0x65}));
  expectFrame(golden, \"CAN_104\",
              encode(DescentCoreTelemetry{0xFB, 0x1A55, 0xF7}));
""",
    """  expectFrame(golden, \"CAN_103\",
              encode(PowerTimeTelemetry{0xFC, 0xA0, 0xDC, 0xFFFA, 0x000C,
                                        0x85}));
  expectFrame(golden, \"CAN_104\",
              encode(DescentCoreTelemetry{0xFB, 0x0015, 0xF7}));
""",
)
replace_once(
    "host_test/mission_host_tests.cpp",
    """  invalid = encode(ControlTelemetry{1, 2, 3});
  invalid.extended = true;
  assert(decode(invalid, control) == CodecError::unsupported_frame);

  TelemetrySequences sequences;
""",
    """  invalid = encode(ControlTelemetry{1, 2, 3});
  invalid.extended = true;
  assert(decode(invalid, control) == CodecError::unsupported_frame);

  RecoveryModeCommand recovery_mode{};
  auto recovery_mode_frame = encode(RecoveryModeCommand{
      0x44, RecoveryMode::enter_recovery_beacon,
      RecoveryModeReason::ground_requested});
  assert(decode(recovery_mode_frame, recovery_mode) == CodecError::none);
  assert(recovery_mode.sequence == 0x44);
  assert(recovery_mode.mode == RecoveryMode::enter_recovery_beacon);
  assert(recovery_mode.reason == RecoveryModeReason::ground_requested);
  recovery_mode_frame.data[1] = 0;
  assert(decode(recovery_mode_frame, recovery_mode) == CodecError::invalid_enum);
  recovery_mode_frame = encode(RecoveryModeCommand{
      0x44, RecoveryMode::enter_recovery_beacon,
      RecoveryModeReason::ground_requested});
  recovery_mode_frame.data[2] = 4;
  assert(decode(recovery_mode_frame, recovery_mode) == CodecError::invalid_enum);

  PowerTimeTelemetry power_time{};
  auto invalid_power_time = encode(PowerTimeTelemetry{1, 2, 3, 4, 5, 0x85});
  invalid_power_time.data[7] |= 1U << 3U;
  assert(decode(invalid_power_time, power_time) == CodecError::reserved_bits);
  const auto masked_power_time =
      encode(PowerTimeTelemetry{1, 2, 3, 4, 5, 0xFD});
  assert(masked_power_time.data[7] == 0x85);

  DescentCoreTelemetry descent_core{};
  auto invalid_descent = encode(DescentCoreTelemetry{1, 0x0015, 2});
  invalid_descent.data[1] |= 1U << 5U;
  assert(decode(invalid_descent, descent_core) == CodecError::reserved_bits);
  invalid_descent = encode(DescentCoreTelemetry{1, 0x0015, 2});
  invalid_descent.data[2] = 1;
  assert(decode(invalid_descent, descent_core) == CodecError::reserved_bits);

  RecoveryControl recovery_control{};
  auto retired_entry = encode(RecoveryControl{
      RecoveryOpcode::wake, RecoverySource::internal_flash, 1, 0, 0});
  retired_entry.data[0] = 0;
  assert(decode(retired_entry, recovery_control) == CodecError::invalid_enum);

  TelemetrySequences sequences;
""",
)
replace_once(
    "host_test/mission_host_tests.cpp",
    """  CommandExecutor cache;
  for (uint8_t id = 1; id <= CommandExecutor::kResultCacheSize; ++id) {
""",
    """  CommandExecutor retired_profile;
  assert(retired_profile.begin(command(30, 0x32), context).result.reason ==
         CommandReason::not_supported);

  CommandExecutor recovery_gate;
  auto descent_without_cutoff = context;
  descent_without_cutoff.state = MissionState::descent;
  descent_without_cutoff.deployment_power_cutoff_done = false;
  auto recovery_request =
      command(31, static_cast<uint8_t>(CommandCode::enter_recovery));
  assert(recovery_gate.begin(recovery_request, descent_without_cutoff)
             .result.reason == CommandReason::safety_interlock);

  CommandExecutor recovery_accept;
  auto descent_after_cutoff = descent_without_cutoff;
  descent_after_cutoff.deployment_power_cutoff_done = true;
  assert(recovery_accept.begin(recovery_request, descent_after_cutoff).execute);
  assert(recovery_accept.finish(31, CommandPhase::completed).phase ==
         CommandPhase::completed);

  CommandExecutor recovery_wrong_state;
  auto control_after_cutoff = descent_after_cutoff;
  control_after_cutoff.state = MissionState::control;
  assert(recovery_wrong_state.begin(recovery_request, control_after_cutoff)
             .result.reason == CommandReason::invalid_state);

  CommandExecutor recovery_arguments;
  auto invalid_recovery_arguments =
      command(32, static_cast<uint8_t>(CommandCode::enter_recovery));
  invalid_recovery_arguments.arguments[0] = 1;
  assert(recovery_arguments.begin(invalid_recovery_arguments,
                                  descent_after_cutoff)
             .result.reason == CommandReason::invalid_argument);

  CommandExecutor cache;
  for (uint8_t id = 1; id <= CommandExecutor::kResultCacheSize; ++id) {
""",
)
