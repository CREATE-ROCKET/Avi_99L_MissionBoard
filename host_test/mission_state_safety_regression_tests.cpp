#include "mission/mission_state.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>

namespace {

mission::PreflightReadinessSnapshot readyReadiness() {
  return {1, 1234, true, true, true, true, true, true, true, true};
}

mission::ControlAvailability readyControl() {
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
  result.roll_estimator_timestamp_us = 8'999'900;
  result.attitude_fresh = true;
  return result;
}

void startAndReachEngineBurn(mission::MissionStateMachine &machine) {
  assert(machine.startSequence(0, readyReadiness(), mission::StartMode::normal) ==
         mission::TransitionResult::completed);
  machine.tick({2'000'000, true, false, readyControl()});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
}

void testGateFailureRemainsInhibited() {
  mission::MissionStateMachine machine;
  startAndReachEngineBurn(machine);

  auto unavailable = readyControl();
  unavailable.ssc_available = false;
  machine.tick({9'000'000, false, false, unavailable, 9'000});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  assert(machine.snapshot().control_reentry_inhibited);

  machine.tick({9'100'000, false, false, readyControl(), 9'100});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  assert(!machine.snapshot().control_roll_reference_valid);
}

void testControlLossPreventsReentry() {
  for (int loss_case = 0; loss_case < 2; ++loss_case) {
    mission::MissionStateMachine machine;
    startAndReachEngineBurn(machine);
    machine.tick({9'000'000, false, false, readyControl(), 9'000});
    assert(machine.snapshot().state == protocol::MissionState::control);

    auto lost = readyControl();
    lost.roll_estimator_timestamp_us = 9'000'900;
    if (loss_case == 0)
      lost.lps_available = false;
    else
      lost.attitude_fresh = false;
    machine.tick({9'001'000, false, false, lost, 9'001});
    assert(machine.snapshot().state == protocol::MissionState::engine_burn);
    assert(machine.snapshot().control_reentry_inhibited);

    auto recovered = readyControl();
    recovered.roll_estimator_timestamp_us = 9'099'900;
    machine.tick({9'100'000, false, false, recovered, 9'100});
    assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  }
}

void testFutureReferenceIsRejected() {
  mission::MissionStateMachine machine;
  startAndReachEngineBurn(machine);

  auto future = readyControl();
  future.roll_estimator_timestamp_us = 9'000'001;
  machine.tick({9'000'000, false, false, future, 9'000});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  assert(machine.snapshot().control_reentry_inhibited);
  assert(!machine.snapshot().control_roll_reference_valid);
}

void testDisableAfterCaptureInvalidatesReference() {
  mission::MissionStateMachine machine;
  startAndReachEngineBurn(machine);
  machine.tick({9'000'000, false, false, readyControl(), 9'000});
  assert(machine.snapshot().state == protocol::MissionState::control);
  assert(machine.snapshot().control_roll_reference_valid);

  assert(machine.disableFinControl() == mission::TransitionResult::completed);
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  assert(machine.snapshot().control_reentry_inhibited);
  assert(!machine.snapshot().control_roll_reference_valid);
}

void testForceDoesNotMakeControlInputsValid() {
  auto readiness = readyReadiness();
  readiness.gyro_bias_valid = false;
  readiness.ssc_zero_valid = false;

  mission::MissionStateMachine machine;
  assert(machine.startSequence(0, readiness, mission::StartMode::forced) ==
         mission::TransitionResult::completed);
  assert(machine.snapshot().forced_start);
  machine.tick({2'000'000, true, false, readyControl()});

  auto control = readyControl();
  control.gyro_bias_valid = false;
  control.ssc_zero_valid = false;
  machine.tick({9'000'000, false, false, control, 9'000});
  assert(machine.snapshot().state == protocol::MissionState::engine_burn);
  assert(machine.snapshot().control_reentry_inhibited);
  assert(machine.snapshot().forced_start);
}

} // 無名名前空間

int main() {
  testGateFailureRemainsInhibited();
  testControlLossPreventsReentry();
  testFutureReferenceIsRejected();
  testDisableAfterCaptureInvalidatesReference();
  testForceDoesNotMakeControlInputsValid();
  std::cout << "mission state safety regression tests: PASS\n";
  return 0;
}
