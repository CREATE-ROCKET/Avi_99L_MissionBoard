from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def replace_between(path: str, start: str, end: str, replacement: str) -> None:
    p = ROOT / path
    text = p.read_text(encoding="utf-8")
    begin = text.find(start)
    finish = text.find(end, begin)
    if begin < 0 or finish < 0:
        raise RuntimeError(f"markers not found in {path}: {start!r} .. {end!r}")
    p.write_text(text[:begin] + replacement + "\n\n" + text[finish:],
                 encoding="utf-8")


replace_between(
    "host_test/mission_host_tests.cpp",
    "void testMissionStateMachine() {",
    "protocol::GenericCommandRequest command",
    r'''void testMissionStateMachine() {
  using mission::FinDirective;
  using mission::MissionStateMachine;
  using mission::PreflightReadinessSnapshot;
  using mission::StartMode;
  using mission::TransitionResult;
  using protocol::MissionState;

  const PreflightReadinessSnapshot ready{1, 1234, true, true, true, true,
                                         true, true, true, true};

  MissionStateMachine cancelled;
  assert(cancelled.startSequence(0, ready, StartMode::normal) ==
         TransitionResult::completed);
  const uint32_t first_epoch = cancelled.snapshot().flight_epoch;
  assert(cancelled.snapshot().state == MissionState::liftoff_detection);
  assert(cancelled.snapshot().fin == FinDirective::zero_hold);
  assert(!cancelled.snapshot().forced_start);
  assert(cancelled.snapshot().preflight_missing_mask == 0);
  assert(cancelled.cancelSequence() == TransitionResult::completed);
  assert(cancelled.snapshot().state == MissionState::command_receive);
  assert(cancelled.snapshot().fin == FinDirective::brake);
  assert(!cancelled.snapshot().forced_start);
  assert(cancelled.snapshot().preflight_generation == 0);

  auto missing = ready;
  missing.generation = 2;
  missing.parachute_close_configured = false;
  missing.gyro_bias_valid = false;
  assert(missing.missingMask() == ((1U << 2U) | (1U << 4U)));
  MissionStateMachine normal_missing;
  assert(normal_missing.startSequence(0, missing, StartMode::normal) ==
         TransitionResult::not_configured);
  assert(normal_missing.snapshot().state == MissionState::command_receive);

  MissionStateMachine forced;
  assert(forced.startSequence(0, missing, StartMode::forced) ==
         TransitionResult::completed);
  assert(forced.snapshot().forced_start);
  assert(forced.snapshot().preflight_missing_mask == missing.missingMask());
  assert(forced.snapshot().preflight_ready_mask == missing.readyMask());
  assert(forced.snapshot().preflight_generation == 2);
  assert(forced.cancelSequence() == TransitionResult::completed);
  assert(!forced.snapshot().forced_start);
  assert(forced.snapshot().preflight_missing_mask == 0);

  auto runtime_unavailable = ready;
  runtime_unavailable.resources_preallocated = false;
  MissionStateMachine runtime_gate;
  assert(runtime_gate.startSequence(0, runtime_unavailable, StartMode::forced) ==
         TransitionResult::runtime_unavailable);

  MissionStateMachine emergency;
  assert(emergency.startSequence(0, missing, StartMode::forced) ==
         TransitionResult::completed);
  const uint8_t emergency_missing = emergency.snapshot().preflight_missing_mask;
  emergency.tick({2'000'000, true, false, readyControl()});
  assert(emergency.snapshot().state == MissionState::engine_burn);
  assert(emergency.snapshot().liftoff_time_us == 1'000'000);
  assert(emergency.liftoffDetectionEmergencyStop() ==
         TransitionResult::completed);
  assert(emergency.snapshot().state == MissionState::liftoff_detection);
  assert(emergency.snapshot().flight_epoch != first_epoch);
  assert(!emergency.snapshot().liftoff_time_valid);
  assert(emergency.snapshot().forced_start);
  assert(emergency.snapshot().preflight_missing_mask == emergency_missing);

  MissionStateMachine control;
  assert(control.startSequence(0, ready, StartMode::normal) ==
         TransitionResult::completed);
  control.tick({2'000'000, true, false, readyControl()});
  control.tick({8'999'999, false, false, readyControl()});
  assert(control.snapshot().state == MissionState::engine_burn);
  auto entry_tick = mission::MissionTickInput{
      9'000'000, false, false, readyControl(), 9'000};
  control.tick(entry_tick);
  assert(control.snapshot().state == MissionState::control);
  assert(control.snapshot().control_roll_reference_valid);
  assert(control.snapshot().control_roll_reference_unwrapped_rad == 12.75);
  assert(control.snapshot().control_roll_reference_estimator_timestamp_us ==
         8'999'900);
  assert(control.snapshot().control_roll_reference_capture_tick == 9'000);
  const uint8_t capture_sequence =
      control.snapshot().control_roll_reference_capture_event_sequence;
  auto changed_roll = readyControl();
  changed_roll.roll_estimate_liftoff_relative_unwrapped_rad +=
      2.0 * 3.14159265358979323846;
  changed_roll.roll_estimator_timestamp_us = 9'000'900;
  control.tick({9'001'000, false, false, changed_roll, 9'001});
  assert(control.snapshot().state == MissionState::control);
  assert(control.snapshot().control_roll_reference_unwrapped_rad == 12.75);
  assert(control.snapshot().control_roll_reference_capture_event_sequence ==
         capture_sequence);
  auto unavailable = readyControl();
  unavailable.ssc_available = false;
  control.tick({9'002'000, false, false, unavailable});
  assert(control.snapshot().state == MissionState::engine_burn);
  assert(control.snapshot().control_reentry_inhibited);
  assert(control.snapshot().fin == FinDirective::zero_hold);
  auto fin_unavailable = unavailable;
  fin_unavailable.fin_control_available = false;
  control.tick({9'003'000, false, false, fin_unavailable});
  assert(control.snapshot().fin == FinDirective::brake);
  control.tick({10'000'000, false, false, readyControl()});
  assert(control.snapshot().state == MissionState::engine_burn);
  assert(control.snapshot().control_roll_reference_capture_event_sequence ==
         capture_sequence);
  assert(control.liftoffDetectionEmergencyStop() ==
         TransitionResult::completed);
  assert(!control.snapshot().control_roll_reference_valid);

  MissionStateMachine stale_reference;
  assert(stale_reference.startSequence(0, ready, StartMode::normal) ==
         TransitionResult::completed);
  stale_reference.tick({2'000'000, true, false, readyControl()});
  auto stale_control = readyControl();
  stale_control.roll_estimator_timestamp_us = 8'996'000;
  stale_reference.tick({9'000'000, false, false, stale_control, 9'000});
  assert(stale_reference.snapshot().state == MissionState::engine_burn);
  assert(stale_reference.snapshot().control_reentry_inhibited);
  assert(!stale_reference.snapshot().control_roll_reference_valid);

  MissionStateMachine disabled;
  assert(disabled.startSequence(0, ready, StartMode::normal) ==
         TransitionResult::completed);
  assert(disabled.disableFinControl() == TransitionResult::completed);
  assert(disabled.snapshot().state == MissionState::liftoff_detection);
  assert(disabled.snapshot().fin == FinDirective::brake);
  disabled.tick({2'000'000, true, false, readyControl()});
  disabled.tick({9'000'000, false, false, readyControl()});
  assert(disabled.snapshot().state == MissionState::engine_burn);

  MissionStateMachine deadlines;
  assert(deadlines.startSequence(0, ready, StartMode::normal) ==
         TransitionResult::completed);
  deadlines.tick({2'000'000, true, false, readyControl()});
  const uint32_t deadline_epoch = deadlines.snapshot().flight_epoch;
  deadlines.tick({17'999'999, false, false, readyControl()},
                 {deadline_epoch - 1, true, true});
  assert(deadlines.snapshot().state != MissionState::descent);
  assert(!deadlines.snapshot().deployment_power_cutoff_latched);
  deadlines.tick({18'000'000, false, false, readyControl()});
  assert(deadlines.snapshot().state == MissionState::descent);
  assert(deadlines.snapshot().deployment_started);
  assert(deadlines.snapshot().parachute == mission::ParaDirective::open);
  deadlines.tick({25'999'999, false, false, readyControl()});
  assert(!deadlines.snapshot().deployment_power_cutoff_latched);
  deadlines.tick({26'000'000, false, false, readyControl()});
  assert(deadlines.snapshot().deployment_power_cutoff_latched);
  assert(deadlines.snapshot().parachute == mission::ParaDirective::powered_off);
  assert(deadlines.snapshot().fin == FinDirective::brake);

  MissionStateMachine pressure;
  assert(pressure.startSequence(0, ready, StartMode::normal) ==
         TransitionResult::completed);
  pressure.tick({2'000'000, true, false, readyControl()});
  pressure.tick({10'999'999, false, true, readyControl()});
  assert(pressure.snapshot().state != MissionState::descent);
  pressure.tick({11'000'000, false, true, readyControl()});
  assert(pressure.snapshot().state == MissionState::descent);

  mission::ResetCheckpoint reset_checkpoint{};
  reset_checkpoint.valid = true;
  reset_checkpoint.state = MissionState::control;
  reset_checkpoint.flight_epoch = 7;
  reset_checkpoint.elapsed_valid = true;
  reset_checkpoint.elapsed_us = 20'000'000;
  reset_checkpoint.forced_start = true;
  reset_checkpoint.preflight_generation = missing.generation;
  reset_checkpoint.preflight_captured_at_us = missing.captured_at_us;
  reset_checkpoint.preflight_ready_mask = missing.readyMask();
  reset_checkpoint.preflight_missing_mask = missing.missingMask();
  MissionStateMachine reset;
  assert(reset.restoreAfterReset(100'000'000, reset_checkpoint) ==
         TransitionResult::completed);
  assert(reset.snapshot().state == MissionState::engine_burn);
  assert(reset.snapshot().reset_invalidated);
  assert(reset.snapshot().forced_start);
  assert(reset.snapshot().preflight_missing_mask == missing.missingMask());
  assert(!reset.snapshot().control_roll_reference_valid);
  assert(reset.snapshot().fin == FinDirective::brake);
  reset.tick({100'000'000, false, false, readyControl()});
  assert(reset.snapshot().state == MissionState::descent);
  reset.tick({105'000'000, false, false, readyControl()});
  assert(reset.snapshot().deployment_power_cutoff_latched);
  assert(reset.snapshot().elapsed_us == 25'000'000);

  mission::ResetCheckpoint pre_liftoff_checkpoint{};
  pre_liftoff_checkpoint.valid = true;
  pre_liftoff_checkpoint.state = MissionState::liftoff_detection;
  pre_liftoff_checkpoint.flight_epoch = 9;
  pre_liftoff_checkpoint.elapsed_valid = false;
  pre_liftoff_checkpoint.forced_start = true;
  pre_liftoff_checkpoint.preflight_generation = missing.generation;
  pre_liftoff_checkpoint.preflight_captured_at_us = missing.captured_at_us;
  pre_liftoff_checkpoint.preflight_ready_mask = missing.readyMask();
  pre_liftoff_checkpoint.preflight_missing_mask = missing.missingMask();
  MissionStateMachine pre_liftoff_reset;
  assert(pre_liftoff_reset.restoreAfterReset(5'000'000,
                                             pre_liftoff_checkpoint) ==
         TransitionResult::completed);
  assert(pre_liftoff_reset.snapshot().state == MissionState::liftoff_detection);
  assert(pre_liftoff_reset.snapshot().forced_start);
  assert(!pre_liftoff_reset.snapshot().liftoff_time_valid);

  mission::ResetCheckpoint invalid_checkpoint = pre_liftoff_checkpoint;
  invalid_checkpoint.valid = false;
  MissionStateMachine invalid_reset;
  assert(invalid_reset.restoreAfterReset(0, invalid_checkpoint) ==
         TransitionResult::not_configured);
  assert(!invalid_reset.snapshot().forced_start);
}''',
)

replace_between(
    "host_test/mission_host_tests.cpp",
    "void testCommandExecutor() {",
    "void testSensors() {",
    r'''void testCommandExecutor() {
  using mission::CommandCode;
  using mission::CommandContext;
  using mission::CommandExecutor;
  using protocol::CommandPhase;
  using protocol::CommandReason;
  using protocol::MissionState;

  CommandContext context{};
  context.resources_preallocated = true;
  context.persistence_load_complete = true;
  context.persistence_runtime_available = true;
  context.fin_safe_commands_supported = true;
  context.calibration_supported = true;

  CommandExecutor executor;
  auto decision = executor.begin(
      command(0, static_cast<uint8_t>(CommandCode::start_sequence)), context);
  assert(!decision.execute);
  assert(decision.result.reason == CommandReason::invalid_argument);

  auto invalid_args =
      command(1, static_cast<uint8_t>(CommandCode::force_start_sequence));
  invalid_args.arguments[5] = 1;
  decision = executor.begin(invalid_args, context);
  assert(decision.result.reason == CommandReason::invalid_argument);
  decision = executor.begin(invalid_args, context);
  assert(decision.replay && !decision.execute);

  CommandContext wrong_state = context;
  wrong_state.state = MissionState::engine_burn;
  decision = executor.begin(
      command(2, static_cast<uint8_t>(CommandCode::force_start_sequence)),
      wrong_state);
  assert(decision.result.reason == CommandReason::invalid_state);

  CommandExecutor resource_gate;
  auto no_resources = context;
  no_resources.resources_preallocated = false;
  assert(resource_gate
             .begin(command(3,
                            static_cast<uint8_t>(
                                CommandCode::force_start_sequence)),
                    no_resources)
             .result.reason == CommandReason::internal_error);

  CommandExecutor load_gate;
  auto loading = context;
  loading.persistence_load_complete = false;
  assert(load_gate
             .begin(command(4,
                            static_cast<uint8_t>(
                                CommandCode::force_start_sequence)),
                    loading)
             .result.reason == CommandReason::busy);

  CommandExecutor persistence_gate;
  auto persistence_failed = context;
  persistence_failed.persistence_runtime_available = false;
  assert(persistence_gate
             .begin(command(5,
                            static_cast<uint8_t>(
                                CommandCode::force_start_sequence)),
                    persistence_failed)
             .result.reason == CommandReason::persistence_error);

  CommandExecutor force_accept;
  auto force = command(6, static_cast<uint8_t>(CommandCode::force_start_sequence));
  decision = force_accept.begin(force, context);
  assert(decision.execute && decision.result.phase == CommandPhase::accepted);
  const auto forced_complete = force_accept.finish(
      6, CommandPhase::completed, CommandReason::none, 0x55);
  assert(forced_complete.command ==
         static_cast<uint8_t>(CommandCode::force_start_sequence));
  assert(forced_complete.detail == 0x55);

  auto fin = command(7, static_cast<uint8_t>(CommandCode::fin_move_relative));
  fin.arguments[0] = 10;
  decision = executor.begin(fin, context);
  assert(decision.execute && decision.result.phase == CommandPhase::accepted);
  decision = executor.begin(fin, context);
  assert(decision.replay && !decision.execute &&
         decision.result.phase == CommandPhase::accepted);
  auto conflicting = fin;
  conflicting.arguments[0] = 11;
  decision = executor.begin(conflicting, context);
  assert(decision.result.reason == CommandReason::protocol_error);
  decision = executor.begin(
      command(8, static_cast<uint8_t>(CommandCode::fin_free)), context);
  assert(decision.result.reason == CommandReason::busy);
  const auto completed =
      executor.finish(7, CommandPhase::completed, CommandReason::none, 7);
  assert(completed.phase == CommandPhase::completed && completed.detail == 7);

  auto para = command(9, static_cast<uint8_t>(CommandCode::para_open));
  assert(executor.begin(para, context).execute);
  auto fin_two = command(10, static_cast<uint8_t>(CommandCode::fin_free));
  assert(executor.begin(fin_two, context).execute);
  const auto emergency =
      executor.actuatorEmergency(11, MissionState::command_receive);
  assert(emergency.execute && emergency.interrupted_count == 2);
  for (std::size_t index = 0; index < emergency.interrupted_count; ++index)
    assert(emergency.interrupted[index].reason ==
           CommandReason::interrupted_by_emergency);
  assert(!executor.actuatorEmergency(0, MissionState::command_receive).execute);
  assert(executor.liftoffEmergencyResult(0, true).reason ==
         CommandReason::invalid_argument);
  assert(executor.liftoffEmergencyResult(12, false).reason ==
         CommandReason::invalid_state);

  auto para_limit =
      command(13, static_cast<uint8_t>(CommandCode::para_move_relative));
  para_limit.arguments[0] = 0x08;
  para_limit.arguments[1] = 0x07;
  assert(executor.begin(para_limit, context).result.reason ==
         CommandReason::invalid_argument);

  CommandExecutor parachute_arguments;
  auto set_open =
      command(14, static_cast<uint8_t>(CommandCode::set_para_open));
  assert(parachute_arguments.begin(set_open, context).execute);
  (void)parachute_arguments.finish(14, CommandPhase::completed);
  set_open = command(15, static_cast<uint8_t>(CommandCode::set_para_open));
  set_open.arguments[0] = 1;
  assert(parachute_arguments.begin(set_open, context).result.reason ==
         CommandReason::invalid_argument);
  auto negative_relative =
      command(16, static_cast<uint8_t>(CommandCode::para_move_relative));
  negative_relative.arguments[0] = 0xFF;
  negative_relative.arguments[1] = 0xFF;
  assert(parachute_arguments.begin(negative_relative, context).execute);

  CommandExecutor start_busy;
  assert(start_busy
             .begin(command(17,
                            static_cast<uint8_t>(CommandCode::para_hold)),
                    context)
             .execute);
  assert(start_busy
             .begin(command(18,
                            static_cast<uint8_t>(
                                CommandCode::force_start_sequence)),
                    context)
             .result.reason == CommandReason::busy);

  CommandExecutor parachute_busy;
  assert(parachute_busy
             .begin(command(19,
                            static_cast<uint8_t>(CommandCode::start_sequence)),
                    context)
             .execute);
  assert(parachute_busy
             .begin(command(20,
                            static_cast<uint8_t>(CommandCode::para_free)),
                    context)
             .result.reason == CommandReason::busy);

  CommandExecutor calibration;
  const auto calibration_decision = calibration.begin(
      command(21,
              static_cast<uint8_t>(CommandCode::run_preflight_calibration)),
      context);
  assert(calibration_decision.execute &&
         calibration_decision.result.phase == CommandPhase::accepted);
  assert(calibration.finish(21, CommandPhase::completed).phase ==
         CommandPhase::completed);

  CommandExecutor cache;
  for (uint8_t id = 1; id <= CommandExecutor::kResultCacheSize; ++id) {
    const auto result = cache.begin(command(id, 0x7F), context);
    assert(result.result.reason == CommandReason::not_supported);
  }
  assert(cache.cachedCount() == CommandExecutor::kResultCacheSize);
  assert(cache.begin(command(1, 0x7F), context).replay);
  assert(cache.begin(command(17, 0x7F), context).result.reason ==
         CommandReason::not_supported);
  assert(cache.cachedCount() == CommandExecutor::kResultCacheSize);
}''',
)

replace_between(
    "host_test/mission_host_tests.cpp",
    "void testParachuteAndRecovery() {",
    "void testRuntimeQueue() {",
    r'''void testParachuteAndRecovery() {
  using namespace actuators;
  PowerArbiter power;
  assert(power.requestAuxiliary5v(true));
  assert(power.requestParachutePower(true));
  assert(power.state().parachute_power);

  ParachuteController parachute;
  assert(parachute.startOpen(1'000'000, 4095) ==
         ParachuteAction::command_open);
  assert(parachute.tick({1'499'999, true, 4095, false}) ==
         ParachuteAction::none);
  assert(parachute.tick({1'500'000, true, 4095, false}) ==
         ParachuteAction::retry_open);
  assert(parachute.status().retry_count == 1);
  assert(parachute.tick({2'000'000, true, 30, false}) ==
         ParachuteAction::none);
  assert(parachute.tick({5'999'999, false, 0, false}) ==
         ParachuteAction::none);
  assert(parachute.tick({6'000'000, false, 0, false}) ==
         ParachuteAction::stop_retrying);
  assert(parachute.status().state == ParachuteOpenState::retry_exhausted);
  assert(parachute.status().open_attempt_finished);
  // 5秒deadlineは電源遮断要求ではない。PowerArbiterは明示cutoffまでONのまま。
  assert(power.state().parachute_power && !power.state().cutoff_latched);

  ParachuteController positive_wrap;
  assert(positive_wrap.startOpen(0, 4095) ==
         ParachuteAction::command_open);
  assert(positive_wrap.tick({500'000, true, 30, false}) ==
         ParachuteAction::none);
  ParachuteController negative_wrap;
  assert(negative_wrap.startOpen(0, 0) == ParachuteAction::command_open);
  assert(negative_wrap.tick({500'000, true, 4060, false}) ==
         ParachuteAction::none);
  ParachuteController half_turn_progress;
  assert(half_turn_progress.startOpen(0, 0) ==
         ParachuteAction::command_open);
  assert(half_turn_progress.tick({500'000, true, 2048, false}) ==
         ParachuteAction::retry_open);

  ParachuteController confirmed;
  assert(confirmed.startOpen(0, 0) == ParachuteAction::command_open);
  assert(confirmed.tick({100'000, true, 30, true}) ==
         ParachuteAction::hold_open);
  assert(confirmed.status().servo_open_confirmed);
  assert(confirmed.status().open_attempt_finished);
  assert(power.state().parachute_power);

  power.latchDeploymentCutoff();
  assert(!power.state().auxiliary_5v && !power.state().parachute_power);
  assert(power.state().cutoff_latched);
  assert(!power.requestAuxiliary5v(true));
  assert(!power.requestParachutePower(true));
  confirmed.notifyPowerCutoff();
  assert(confirmed.status().state == ParachuteOpenState::powered_off);

  using namespace mission;
  const auto marker = makeRecoveryMarker(7);
  assert(validRecoveryMarker(marker));
  assert(selectBootPath({ResetKind::deep_sleep, false, false, false, false,
                         true, marker}) == BootPath::recovery_only);
  assert(selectBootPath({ResetKind::deep_sleep, false, false, false, false,
                         false, marker}) == BootPath::fail_safe);
  assert(selectBootPath({ResetKind::software_or_watchdog, true, true, false,
                         false, false, {}}) ==
         BootPath::resume_flight_safely);
  assert(selectBootPath({ResetKind::power_on, true, false, false, false,
                         false, {}}) == BootPath::await_absolute_time);
  RecoveryRuntime recovery;
  assert(recovery.requestEntry());
  assert(recovery.markResourcesSafeAndFlushed());
  assert(recovery.mayEnterDeepSleep());
  assert(recovery.wake(true, true));
  assert(recovery.beginTransfer());
  assert(!recovery.mayEnterDeepSleep());
  recovery.finishTransfer();
  assert(recovery.mayEnterDeepSleep());
}''',
)

(ROOT / "host_test/parachute_configuration_tests.cpp").write_text(
    r'''#include "actuators/parachute_configuration.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

using actuators::AbsoluteParachuteAngle;
using actuators::FlightParachutePreparationError;
using actuators::ParachuteBlobError;
using actuators::ParachuteConfiguration;
using actuators::ParachuteConfigurationState;
using actuators::ParachuteEndpoint;
using actuators::ParachuteEndpointBlob;
using actuators::ParachutePathError;

[[noreturn]] void fail(const char *message) {
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

void require(bool condition, const char *message) {
  if (!condition)
    fail(message);
}

AbsoluteParachuteAngle angle(uint16_t count) {
  const auto value = AbsoluteParachuteAngle::fromCount(count);
  require(value.has_value(), "test angle must be valid");
  return *value;
}

uint16_t nearestCount(double degrees) {
  return static_cast<uint16_t>(std::lround(
      degrees / actuators::kParachuteDegreesPerCount));
}

void refreshCrc(ParachuteEndpointBlob &blob) {
  actuators::writeLe32(blob.data() + 12,
                       actuators::parachuteCrc32(blob.data(), 12));
}

void requireCorruptNullopt(const ParachuteEndpointBlob &blob,
                           ParachuteEndpoint endpoint,
                           ParachuteBlobError expected,
                           const char *message) {
  const auto decoded = actuators::decodeParachuteEndpoint(
      blob.data(), blob.size(), endpoint);
  require(decoded.error == expected && !decoded.angle.has_value(), message);
}

void testAbsoluteAnglesAndConfiguration() {
  require(angle(0).count() == 0, "count zero must be valid");
  require(angle(4095).count() == 4095, "count 4095 must be valid");
  require(!AbsoluteParachuteAngle::fromCount(4096).has_value(),
          "count 4096 must not be normalized");

  ParachuteConfiguration configuration{};
  require(!configuration.openConfigured() && !configuration.closeConfigured(),
          "both endpoints must start unconfigured");
  configuration.open = angle(1);
  require(configuration.openConfigured() && !configuration.closeConfigured(),
          "open-only configuration must be represented");
  configuration = {};
  configuration.close = angle(2);
  require(!configuration.openConfigured() && configuration.closeConfigured(),
          "close-only configuration must be represented");
  configuration.open = angle(1);
  require(configuration.openConfigured() && configuration.closeConfigured(),
          "both endpoints must be represented independently");
}

void testShortestPath() {
  const auto forward =
      actuators::shortestParachuteDisplacement(angle(0), angle(2047));
  require(forward.valid() && forward.counts == 2047,
          "0 -> 2047 must be positive");
  const auto half =
      actuators::shortestParachuteDisplacement(angle(0), angle(2048));
  require(!half.valid() && half.error == ParachutePathError::exactly_half_turn,
          "0 -> 2048 must be rejected");
  const auto reverse =
      actuators::shortestParachuteDisplacement(angle(0), angle(2049));
  require(reverse.valid() && reverse.counts == -2047,
          "0 -> 2049 must be negative");
  require(actuators::shortestParachuteDisplacement(angle(2048), angle(0))
              .error == ParachutePathError::exactly_half_turn,
          "2048 -> 0 must be rejected");
  require(actuators::shortestParachuteDisplacement(angle(4095), angle(0))
              .counts == 1,
          "4095 -> 0 must be +1");
  require(actuators::shortestParachuteDisplacement(angle(0), angle(4095))
              .counts == -1,
          "0 -> 4095 must be -1");

  const auto path = [](double current, double target) {
    return actuators::shortestParachuteDisplacement(
        angle(nearestCount(current)), angle(nearestCount(target)));
  };
  require(std::abs(path(350.0, 10.0).degrees() - 20.0) < 0.1,
          "350 -> 10 must be about +20 degrees");
  require(std::abs(path(10.0, 350.0).degrees() + 20.0) < 0.1,
          "10 -> 350 must be about -20 degrees");
  require(path(0.0, 179.9).valid(), "179.9 degrees must remain valid");
  require(!path(0.0, 180.0).valid(), "180 degrees must be rejected");
}

void testBlobValidation() {
  const auto valid = actuators::encodeParachuteEndpoint(
      ParachuteEndpoint::open, angle(4095));
  const auto decoded = actuators::decodeParachuteEndpoint(
      valid.data(), valid.size(), ParachuteEndpoint::open);
  require(decoded.valid() && decoded.angle->count() == 4095,
          "valid blob must round-trip");
  const auto wrong_size = actuators::decodeParachuteEndpoint(
      valid.data(), valid.size() - 1, ParachuteEndpoint::open);
  require(wrong_size.error == ParachuteBlobError::wrong_size &&
              !wrong_size.angle.has_value(),
          "wrong size must be operational nullopt");
  const auto wrong_endpoint = actuators::decodeParachuteEndpoint(
      valid.data(), valid.size(), ParachuteEndpoint::close);
  require(wrong_endpoint.error == ParachuteBlobError::wrong_endpoint &&
              !wrong_endpoint.angle.has_value(),
          "wrong endpoint must be operational nullopt");

  auto blob = valid;
  blob[8] ^= 1U;
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::crc_mismatch,
                        "CRC mismatch must preserve corrupt reason");

  blob = valid;
  blob[0] = 'X';
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::wrong_magic,
                        "wrong magic must preserve corrupt reason");

  blob = valid;
  blob[4] = 2;
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::wrong_schema,
                        "wrong schema must preserve corrupt reason");

  blob = valid;
  actuators::writeLe16(blob.data() + 6, 3);
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::wrong_payload_size,
                        "wrong payload size must preserve corrupt reason");

  blob = valid;
  actuators::writeLe16(blob.data() + 10, 1);
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::reserved_nonzero,
                        "nonzero reserved must preserve corrupt reason");

  blob = valid;
  actuators::writeLe16(blob.data() + 8, 4096);
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::angle_out_of_range,
                        "out-of-range count must not be normalized");
}

void testTransactionAndRebootLoad() {
  ParachuteConfigurationState missing_keys;
  missing_keys.replaceLoadedConfiguration({});
  require(!missing_keys.active().open.has_value() &&
              !missing_keys.active().close.has_value(),
          "missing keys must load as independent nullopt endpoints");

  ParachuteConfigurationState state;
  ParachuteConfiguration loaded{};
  loaded.open = angle(100);
  loaded.close = angle(500);
  state.replaceLoadedConfiguration(loaded);
  const auto candidate =
      state.candidateWith(ParachuteEndpoint::open, angle(200));
  require(state.active().open->count() == 100,
          "candidate must not update active RAM");
  state.activatePersistedCandidate(candidate);
  require(state.active().open->count() == 200,
          "verified save may update active RAM");

  const auto open_blob = actuators::encodeParachuteEndpoint(
      ParachuteEndpoint::open, angle(321));
  auto close_blob = actuators::encodeParachuteEndpoint(
      ParachuteEndpoint::close, angle(654));
  close_blob[8] ^= 1U;
  ParachuteConfiguration rebooted_configuration{};
  const auto open = actuators::decodeParachuteEndpoint(
      open_blob.data(), open_blob.size(), ParachuteEndpoint::open);
  const auto close = actuators::decodeParachuteEndpoint(
      close_blob.data(), close_blob.size(), ParachuteEndpoint::close);
  if (open.valid())
    rebooted_configuration.open = open.angle;
  if (close.valid())
    rebooted_configuration.close = close.angle;
  ParachuteConfigurationState rebooted;
  rebooted.replaceLoadedConfiguration(rebooted_configuration);
  require(rebooted.active().open->count() == 321 &&
              !rebooted.active().close.has_value(),
          "one corrupt endpoint must not discard the other");
  require(close.error == ParachuteBlobError::crc_mismatch,
          "corrupt reason must remain observable separately from nullopt");
}

void testFlightSnapshot() {
  ParachuteConfigurationState state;
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::open_not_configured,
          "normal Start must require Open");

  ParachuteConfiguration open_only{};
  open_only.open = angle(100);
  state.replaceLoadedConfiguration(open_only);
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::close_not_configured,
          "normal Start must require Close");
  state.freezeFlightSnapshotForced();
  require(state.flightSnapshotValid() &&
              state.flightSnapshot()->open.has_value() &&
              !state.flightSnapshot()->close.has_value() &&
              state.flightSnapshot()->open->count() == 100,
          "Force snapshot must preserve Open-only configuration");

  ParachuteConfiguration close_only{};
  close_only.close = angle(200);
  state.replaceLoadedConfiguration(close_only);
  state.freezeFlightSnapshotForced();
  require(!state.flightSnapshot()->open.has_value() &&
              state.flightSnapshot()->close.has_value() &&
              state.flightSnapshot()->close->count() == 200,
          "Force snapshot must preserve Close-only configuration");

  state.replaceLoadedConfiguration({});
  state.freezeFlightSnapshotForced();
  require(!state.flightSnapshot()->open.has_value() &&
              !state.flightSnapshot()->close.has_value(),
          "Force snapshot must preserve both-null configuration");

  ParachuteConfiguration mutual_half_turn{};
  mutual_half_turn.open = angle(100);
  mutual_half_turn.close = angle(2148);
  state.replaceLoadedConfiguration(mutual_half_turn);
  require(state.freezeFlightSnapshot(angle(0)).ready(),
          "Open/Close mutual half-turn must not reject normal Start");
  require(state.flightSnapshot()->open->count() == 100 &&
              state.flightSnapshot()->close->count() == 2148,
          "normal snapshot must freeze both endpoints");

  require(state.freezeFlightSnapshot(angle(2148)).error ==
              FlightParachutePreparationError::current_open_exactly_half_turn,
          "actual current-to-Open half-turn must reject normal Start");

  ParachuteConfiguration valid{};
  valid.open = angle(300);
  valid.close = angle(100);
  state.replaceLoadedConfiguration(valid);
  require(state.freezeFlightSnapshot(angle(200)).ready(),
          "valid endpoints must prepare normal Start");
  require(state.flightSnapshot()->open->count() == 300,
          "snapshot must contain Start-time Open");
  state.activatePersistedCandidate(
      state.candidateWith(ParachuteEndpoint::open, angle(700)));
  require(state.active().open->count() == 700 &&
              state.flightSnapshot()->open->count() == 300,
          "active changes must not alter flight snapshot");
  state.discardFlightSnapshot();
  require(!state.flightSnapshotValid() && state.active().open->count() == 700,
          "Cancel must discard only flight snapshot");

  actuators::FlightParachuteConfiguration restored{};
  restored.open = angle(333);
  state.restoreFlightSnapshot(restored);
  ParachuteConfiguration reloaded{};
  reloaded.open = angle(900);
  reloaded.close = angle(800);
  state.replaceLoadedConfiguration(reloaded);
  require(state.flightSnapshot()->open->count() == 333 &&
              !state.flightSnapshot()->close.has_value() &&
              state.active().open->count() == 900,
          "RTC optional snapshot must remain independent of active NVS config");
}

} // 無名名前空間

int main() {
  testAbsoluteAnglesAndConfiguration();
  testShortestPath();
  testBlobValidation();
  testTransactionAndRebootLoad();
  testFlightSnapshot();
  std::cout << "parachute configuration tests: PASS\n";
  return 0;
}
''',
    encoding="utf-8",
)

print("ForceStart/Parachute Stage 2 host tests transformed")
