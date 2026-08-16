#include "control/control_pipeline.hpp"
#include "control/fin_overtravel_guard.hpp"
#include "mission/command_executor.hpp"
#include "mission/mission_state.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr double degrees(double value) { return value * kPi / 180.0; }

void resetGuard() {
  control::clearFinOvertravelFault();
  control::observeFinOvertravel(0.0, true);
}

mission::PreflightReadinessSnapshot readyReadiness() {
  return {1, 1234, true, true, true, true, true, true, true, true};
}

mission::ControlAvailability readyControl(uint64_t timestamp_us) {
  mission::ControlAvailability result{};
  result.fin_control_available = true;
  result.fin_zero_hold_valid = true;
  result.attitude_valid = true;
  result.airspeed_above_60 = true;
  result.lps_available = true;
  result.ssc_available = true;
  result.gyro_bias_valid = true;
  result.ssc_zero_valid = true;
  result.roll_estimate_liftoff_relative_unwrapped_rad = 12.75;
  result.roll_estimator_timestamp_us = timestamp_us;
  result.attitude_fresh = true;
  return result;
}

protocol::GenericCommandRequest command(uint8_t transaction_id,
                                        mission::CommandCode code) {
  return {transaction_id, static_cast<uint8_t>(code), {0, 0, 0, 0, 0, 0}};
}

mission::CommandContext readyCommandContext() {
  mission::CommandContext context{};
  context.state = protocol::MissionState::command_receive;
  context.resources_preallocated = true;
  context.persistence_load_complete = true;
  context.persistence_runtime_available = true;
  context.fin_available = true;
  context.parachute_available = true;
  context.calibration_supported = true;
  context.storage_export_supported = true;
  context.fin_safe_commands_supported = true;
  return context;
}

void testThresholdAndInvalidSamples() {
  resetGuard();
  control::observeFinOvertravel(degrees(20.0), true);
  assert(!control::finOvertravelFaultLatched());
  control::observeFinOvertravel(degrees(-20.0), true);
  assert(!control::finOvertravelFaultLatched());

  // invalid/stale相当のsampleは範囲外でもovertravelを新規成立させない。
  control::observeFinOvertravel(degrees(20.1), false);
  assert(!control::finOvertravelFaultLatched());
  control::observeFinOvertravel(degrees(-20.1), false);
  assert(!control::finOvertravelFaultLatched());
  control::observeFinOvertravel(NAN, true);
  assert(!control::finOvertravelFaultLatched());
  control::observeFinOvertravel(INFINITY, true);
  assert(!control::finOvertravelFaultLatched());

  control::observeFinOvertravel(degrees(20.1), true);
  assert(control::finOvertravelFaultLatched());
  control::observeFinOvertravel(0.0, false);
  assert(!control::clearFinOvertravelIfRecoverable());
  assert(control::finOvertravelFaultLatched());

  control::observeFinOvertravel(NAN, true);
  assert(!control::clearFinOvertravelIfRecoverable());
  assert(control::finOvertravelFaultLatched());
  control::observeFinOvertravel(INFINITY, true);
  assert(!control::clearFinOvertravelIfRecoverable());
  assert(control::finOvertravelFaultLatched());

  resetGuard();
  control::observeFinOvertravel(degrees(-20.1), true);
  assert(control::finOvertravelFaultLatched());
}

void testCommandReceiveRecoveryAndGating() {
  using mission::CommandCode;
  using protocol::CommandPhase;
  using protocol::CommandReason;

  const auto context = readyCommandContext();
  resetGuard();
  control::ZeroHoldController zero;
  (void)zero.updateValidity(degrees(20.1), 0.0, true);
  assert(control::finOvertravelFaultLatched());
  assert(!zero.compute(0.0, 0.0).valid);

  for (const auto code : {CommandCode::start_sequence,
                          CommandCode::force_start_sequence,
                          CommandCode::fin_hold_current}) {
    mission::CommandExecutor executor;
    const auto decision = executor.begin(command(1, code), context);
    assert(!decision.execute);
    assert(decision.result.reason == CommandReason::safety_interlock);
  }

  mission::CommandExecutor release_executor;
  const auto release_decision =
      release_executor.begin(command(2, CommandCode::fin_release_hold), context);
  assert(release_decision.execute);
  assert(release_decision.result.phase == CommandPhase::accepted);

  mission::CommandExecutor failed_zero_executor;
  const auto failed_zero =
      failed_zero_executor.begin(command(3, CommandCode::set_fin_zero), context);
  assert(failed_zero.execute);
  (void)failed_zero_executor.finish(3, CommandPhase::failed,
                                    CommandReason::device_unavailable);
  assert(control::finOvertravelFaultLatched());

  mission::CommandExecutor successful_zero_executor;
  const auto successful_zero = successful_zero_executor.begin(
      command(4, CommandCode::set_fin_zero), context);
  assert(successful_zero.execute);
  (void)successful_zero_executor.finish(4, CommandPhase::completed,
                                        CommandReason::none);
  assert(!control::finOvertravelFaultLatched());
  assert(zero.compute(0.0, 0.0).valid);

  mission::CommandExecutor legacy_executor;
  const auto legacy = legacy_executor.begin(
      command(5, CommandCode::start_fin_zero_hold), context);
  assert(!legacy.execute);
  assert(legacy.result.phase == CommandPhase::rejected);
  assert(legacy.result.reason == CommandReason::not_supported);

  mission::CommandExecutor hold_executor;
  const auto hold =
      hold_executor.begin(command(6, CommandCode::fin_hold_current), context);
  assert(hold.execute);
  assert(hold.result.phase == CommandPhase::accepted);

  mission::CommandExecutor argument_executor;
  auto invalid_hold = command(7, CommandCode::fin_hold_current);
  invalid_hold.arguments[0] = 1;
  const auto invalid = argument_executor.begin(invalid_hold, context);
  assert(!invalid.execute);
  assert(invalid.result.reason == CommandReason::invalid_argument);

  control::observeFinOvertravel(degrees(-20.1), true);
  assert(control::finOvertravelFaultLatched());
  control::observeFinOvertravel(degrees(-19.9), true);
  mission::CommandExecutor recovered_executor;
  const auto recovered = recovered_executor.begin(
      command(8, CommandCode::fin_hold_current), context);
  assert(recovered.execute);
  assert(!control::finOvertravelFaultLatched());
}

void testStartGateAndFlightNoReentry() {
  resetGuard();
  control::observeFinOvertravel(degrees(20.1), true);
  mission::MissionStateMachine blocked;
  assert(blocked.startSequence(0, readyReadiness(), mission::StartMode::forced) ==
         mission::TransitionResult::runtime_unavailable);

  control::observeFinOvertravel(degrees(19.0), true);
  assert(blocked.startSequence(0, readyReadiness(), mission::StartMode::normal) ==
         mission::TransitionResult::completed);
  assert(!control::finOvertravelFaultLatched());

  resetGuard();
  mission::MissionStateMachine machine;
  assert(machine.startSequence(0, readyReadiness(), mission::StartMode::normal) ==
         mission::TransitionResult::completed);
  machine.tick({2'000'000, true, false, readyControl(1'999'900)});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  machine.tick({9'000'000, false, false, readyControl(8'999'900), 9'000});
  assert(machine.snapshot().state == protocol::MissionState::control);
  assert(machine.snapshot().control_roll_reference_valid);

  control::observeFinOvertravel(degrees(20.1), true);
  machine.tick({9'001'000, false, false, readyControl(9'000'900), 9'001});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  assert(machine.snapshot().control_reentry_inhibited);
  assert(machine.snapshot().fin == mission::FinDirective::brake);
  assert(!machine.snapshot().control_roll_reference_valid);
  assert(control::finOvertravelFaultLatched());

  control::observeFinOvertravel(0.0, true);
  machine.tick({9'100'000, false, false, readyControl(9'099'900), 9'100});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  assert(machine.snapshot().control_reentry_inhibited);
  assert(machine.snapshot().fin == mission::FinDirective::brake);
  assert(control::finOvertravelFaultLatched());
}

void testCommandReceiveAutomaticRecovery() {
  resetGuard();
  control::observeFinOvertravel(degrees(-20.1), true);
  assert(control::finOvertravelFaultLatched());
  control::observeFinOvertravel(degrees(-19.9), true);

  mission::MissionStateMachine machine;
  machine.tick({1'000, false, false, {}});
  assert(machine.snapshot().state == protocol::MissionState::command_receive);
  assert(!control::finOvertravelFaultLatched());
}

} // 無名名前空間

int main() {
  testThresholdAndInvalidSamples();
  testCommandReceiveRecoveryAndGating();
  testStartGateAndFlightNoReentry();
  testCommandReceiveAutomaticRecovery();
  std::cout << "fin overtravel tests: PASS\n";
  return 0;
}
