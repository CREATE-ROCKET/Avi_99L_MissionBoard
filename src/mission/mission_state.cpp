#include "mission/mission_state.hpp"

#include <cmath>

#include "control/fin_overtravel_guard.hpp"

namespace mission {
namespace {
constexpr uint64_t kOneSecondUs = 1'000'000;
constexpr uint64_t kControlGateUs = 8 * kOneSecondUs;
constexpr uint64_t kPressureDeploymentGateUs = 10 * kOneSecondUs;
constexpr uint64_t kDeploymentDeadlineUs = 17 * kOneSecondUs;
constexpr uint64_t kPowerCutoffUs = 25 * kOneSecondUs;

uint32_t nextEpoch(uint32_t current) {
  ++current;
  return current == 0 ? 1 : current;
}

constexpr uint8_t bit(PreflightReadinessBit value) {
  return static_cast<uint8_t>(1U << static_cast<uint8_t>(value));
}

bool finFlightReadinessValid(const MissionSnapshot &snapshot) {
  constexpr uint8_t required =
      bit(PreflightReadinessBit::fin_zero_configured) |
      bit(PreflightReadinessBit::motor_profile_valid);
  return (snapshot.preflight_ready_mask & required) == required;
}
} // 無名名前空間

uint8_t PreflightReadinessSnapshot::readyMask() const {
  uint8_t result = 0;
  if (fin_zero_configured)
    result |= bit(PreflightReadinessBit::fin_zero_configured);
  if (motor_profile_valid)
    result |= bit(PreflightReadinessBit::motor_profile_valid);
  if (gyro_bias_valid)
    result |= bit(PreflightReadinessBit::gyro_bias_valid);
  if (gravity_reference_valid)
    result |= bit(PreflightReadinessBit::gravity_reference_valid);
  if (ssc_zero_valid)
    result |= bit(PreflightReadinessBit::ssc_zero_valid);
  return result;
}

uint8_t PreflightReadinessSnapshot::missingMask() const {
  return static_cast<uint8_t>((~readyMask()) & kPreflightReadinessMask);
}

bool PreflightReadinessSnapshot::normalReady() const {
  return resources_preallocated && missingMask() == 0;
}

bool ControlAvailability::ready() const {
  return fin_control_available && fin_zero_hold_valid && attitude_valid &&
         airspeed_above_60 && lps_available && ssc_available &&
         gyro_bias_valid && ssc_zero_valid && attitude_fresh &&
         std::isfinite(roll_estimate_liftoff_relative_unwrapped_rad) &&
         roll_estimator_timestamp_us != 0;
}

TransitionResult MissionStateMachine::startSequence(
    uint64_t, const PreflightReadinessSnapshot &readiness, StartMode mode) {
  if (snapshot_.state != protocol::MissionState::command_receive)
    return TransitionResult::invalid_state;
  // CommandReceiveでvalidな角度が20 deg以内へ戻っていれば、Start判定前に
  // overtravelだけを復帰させる。別device faultには触れない。
  (void)control::clearFinOvertravelIfRecoverable();
  if (control::finOvertravelFaultLatched())
    return TransitionResult::runtime_unavailable;
  if (!readiness.resources_preallocated)
    return TransitionResult::runtime_unavailable;
  if (mode == StartMode::normal && readiness.missingMask() != 0)
    return TransitionResult::not_configured;

  snapshot_.flight_epoch = nextEpoch(snapshot_.flight_epoch);
  snapshot_.state = protocol::MissionState::liftoff_detection;
  snapshot_.fin_control_disabled = false;
  snapshot_.control_reentry_inhibited = false;
  snapshot_.reset_invalidated = false;
  snapshot_.deployment_started = false;
  snapshot_.deployment_power_cutoff_latched = false;
  snapshot_.forced_start = mode == StartMode::forced;
  snapshot_.preflight_generation = readiness.generation;
  snapshot_.preflight_captured_at_us = readiness.captured_at_us;
  snapshot_.preflight_ready_mask = readiness.readyMask();
  snapshot_.preflight_missing_mask = readiness.missingMask();
  snapshot_.parachute = ParaDirective::hold;
  control_gate_evaluated_ = false;
  fin_control_available_ =
      readiness.fin_zero_configured && readiness.motor_profile_valid;
  elapsed_offset_us_ = 0;
  invalidateControlRollReference();
  invalidateLiftoff();
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::cancelSequence() {
  if (snapshot_.state != protocol::MissionState::liftoff_detection)
    return TransitionResult::invalid_state;
  snapshot_.state = protocol::MissionState::command_receive;
  snapshot_.fin_control_disabled = false;
  snapshot_.control_reentry_inhibited = false;
  snapshot_.deployment_started = false;
  control_gate_evaluated_ = false;
  fin_control_available_ = false;
  elapsed_offset_us_ = 0;
  invalidateControlRollReference();
  invalidateLiftoff();
  clearFlightAttemptMetadata();
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::disableFinControl() {
  if (snapshot_.state != protocol::MissionState::liftoff_detection &&
      snapshot_.state != protocol::MissionState::engine_burn &&
      snapshot_.state != protocol::MissionState::control)
    return TransitionResult::invalid_state;
  snapshot_.fin_control_disabled = true;
  snapshot_.control_reentry_inhibited = true;
  invalidateControlRollReference();
  if (snapshot_.state == protocol::MissionState::control)
    snapshot_.state = protocol::MissionState::engine_burn;
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::liftoffDetectionEmergencyStop() {
  if (snapshot_.state != protocol::MissionState::engine_burn)
    return TransitionResult::invalid_state;
  snapshot_.flight_epoch = nextEpoch(snapshot_.flight_epoch);
  snapshot_.state = protocol::MissionState::liftoff_detection;
  snapshot_.control_reentry_inhibited = snapshot_.fin_control_disabled;
  snapshot_.deployment_started = false;
  control_gate_evaluated_ = false;
  elapsed_offset_us_ = 0;
  invalidateControlRollReference();
  invalidateLiftoff();
  // 同一flight attempt由来なのでforced_start/preflight snapshotは維持する。
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::restoreAfterReset(
    uint64_t now_us, const ResetCheckpoint &checkpoint) {
  if (!checkpoint.valid || checkpoint.flight_epoch == 0)
    return TransitionResult::not_configured;
  if (checkpoint.state == protocol::MissionState::liftoff_detection &&
      checkpoint.elapsed_valid)
    return TransitionResult::not_configured;
  if (checkpoint.state != protocol::MissionState::liftoff_detection &&
      (!checkpoint.elapsed_valid ||
       (checkpoint.state != protocol::MissionState::engine_burn &&
        checkpoint.state != protocol::MissionState::control &&
        checkpoint.state != protocol::MissionState::descent)))
    return TransitionResult::not_configured;

  snapshot_ = {};
  snapshot_.flight_epoch = checkpoint.flight_epoch;
  snapshot_.forced_start = checkpoint.forced_start;
  snapshot_.preflight_generation = checkpoint.preflight_generation;
  snapshot_.preflight_captured_at_us = checkpoint.preflight_captured_at_us;
  snapshot_.preflight_ready_mask = checkpoint.preflight_ready_mask;
  snapshot_.preflight_missing_mask = checkpoint.preflight_missing_mask;
  snapshot_.control_reentry_inhibited = true;
  snapshot_.reset_invalidated = true;
  invalidateControlRollReference();

  if (checkpoint.state == protocol::MissionState::liftoff_detection) {
    snapshot_.state = protocol::MissionState::liftoff_detection;
    snapshot_.parachute = ParaDirective::hold;
    control_gate_evaluated_ = false;
    fin_control_available_ = false;
    elapsed_offset_us_ = 0;
    invalidateLiftoff();
    updateDirectives(now_us);
    return TransitionResult::completed;
  }

  snapshot_.state = checkpoint.deployment_started ||
                            checkpoint.state == protocol::MissionState::descent
                        ? protocol::MissionState::descent
                        : protocol::MissionState::engine_burn;
  snapshot_.liftoff_time_valid = true;
  snapshot_.liftoff_time_us = now_us;
  snapshot_.elapsed_us = checkpoint.elapsed_us;
  snapshot_.deployment_started = checkpoint.deployment_started;
  snapshot_.deployment_power_cutoff_latched = checkpoint.power_cutoff_latched;
  snapshot_.fin = FinDirective::brake;
  snapshot_.parachute = checkpoint.power_cutoff_latched
                            ? ParaDirective::powered_off
                            : (checkpoint.deployment_started
                                   ? ParaDirective::open
                                   : ParaDirective::hold);
  control_gate_evaluated_ = true;
  fin_control_available_ = false;
  elapsed_offset_us_ = checkpoint.elapsed_us;
  updateDirectives(now_us);
  return TransitionResult::completed;
}

void MissionStateMachine::tick(const MissionTickInput &input,
                               const SafetyRequest &safety) {
  const bool current_safety = safety.flight_epoch != 0 &&
                              safety.flight_epoch == snapshot_.flight_epoch;
  if (current_safety && snapshot_.liftoff_time_valid &&
      safety.absolute_power_cutoff)
    snapshot_.deployment_power_cutoff_latched = true;

  if (snapshot_.state == protocol::MissionState::liftoff_detection &&
      input.liftoff_detected) {
    snapshot_.liftoff_time_valid = true;
    snapshot_.liftoff_time_us = input.monotonic_us >= kOneSecondUs
                                    ? input.monotonic_us - kOneSecondUs
                                    : 0;
    snapshot_.state = protocol::MissionState::engine_burn;
    control_gate_evaluated_ = false;
    elapsed_offset_us_ = 0;
  }

  uint64_t elapsed_us{};
  if (snapshot_.liftoff_time_valid &&
      input.monotonic_us >= snapshot_.liftoff_time_us)
    elapsed_us = elapsed_offset_us_ + input.monotonic_us -
                 snapshot_.liftoff_time_us;
  snapshot_.elapsed_us = elapsed_us;
  fin_control_available_ = input.control.fin_control_available;

  if (snapshot_.state == protocol::MissionState::command_receive) {
    // CommandReceiveだけはvalid/freshな観測が20 deg以内へ戻れば復帰可能。
    (void)control::clearFinOvertravelIfRecoverable();
  } else if (control::finOvertravelFaultLatched()) {
    // 飛行sequenceでは角度が戻っても同一flight epochのControlへ再entryしない。
    snapshot_.control_reentry_inhibited = true;
    fin_control_available_ = false;
    invalidateControlRollReference();
    if (snapshot_.state == protocol::MissionState::control)
      snapshot_.state = protocol::MissionState::engine_burn;
  }

  const bool flight_state =
      snapshot_.state == protocol::MissionState::engine_burn ||
      snapshot_.state == protocol::MissionState::control;
  if (flight_state &&
      ((current_safety && safety.deploy) ||
       (input.deployment_pressure_condition && snapshot_.liftoff_time_valid &&
        elapsed_us >= kPressureDeploymentGateUs) ||
       (snapshot_.liftoff_time_valid && elapsed_us >= kDeploymentDeadlineUs)))
    enterDescent();

  if (snapshot_.state == protocol::MissionState::engine_burn &&
      snapshot_.liftoff_time_valid && elapsed_us >= kControlGateUs &&
      !control_gate_evaluated_) {
    control_gate_evaluated_ = true;
    if (!snapshot_.fin_control_disabled &&
        !snapshot_.control_reentry_inhibited &&
        finFlightReadinessValid(snapshot_) && input.control.ready() &&
        captureControlRollReference(input))
      snapshot_.state = protocol::MissionState::control;
    else
      snapshot_.control_reentry_inhibited = true;
  }

  if (snapshot_.state == protocol::MissionState::control &&
      !input.control.ready()) {
    snapshot_.state = protocol::MissionState::engine_burn;
    snapshot_.control_reentry_inhibited = true;
  }

  if (snapshot_.liftoff_time_valid && elapsed_us >= kPowerCutoffUs)
    snapshot_.deployment_power_cutoff_latched = true;
  updateDirectives(input.monotonic_us);
}

void MissionStateMachine::enterDescent() {
  snapshot_.state = protocol::MissionState::descent;
  snapshot_.deployment_started = true;
  snapshot_.parachute = ParaDirective::open;
}

void MissionStateMachine::updateDirectives(uint64_t now_us) {
  if (snapshot_.deployment_power_cutoff_latched)
    snapshot_.parachute = ParaDirective::powered_off;
  else if (snapshot_.deployment_started)
    snapshot_.parachute = ParaDirective::open;
  else
    snapshot_.parachute = ParaDirective::hold;

  if (snapshot_.reset_invalidated || snapshot_.fin_control_disabled) {
    snapshot_.fin = FinDirective::brake;
    return;
  }
  if (snapshot_.state == protocol::MissionState::control) {
    snapshot_.fin = finFlightReadinessValid(snapshot_)
                        ? FinDirective::roll_control
                        : FinDirective::brake;
    return;
  }
  if (snapshot_.state == protocol::MissionState::liftoff_detection ||
      snapshot_.state == protocol::MissionState::engine_burn ||
      snapshot_.state == protocol::MissionState::descent) {
    if (snapshot_.liftoff_time_valid &&
        now_us >= snapshot_.liftoff_time_us + kPowerCutoffUs)
      snapshot_.fin = FinDirective::brake;
    else if (fin_control_available_ && finFlightReadinessValid(snapshot_))
      snapshot_.fin = FinDirective::zero_hold;
    else
      snapshot_.fin = FinDirective::brake;
    return;
  }
  snapshot_.fin = FinDirective::brake;
}

void MissionStateMachine::invalidateLiftoff() {
  snapshot_.liftoff_time_valid = false;
  snapshot_.liftoff_time_us = 0;
  snapshot_.elapsed_us = 0;
  elapsed_offset_us_ = 0;
}

void MissionStateMachine::invalidateControlRollReference() {
  snapshot_.control_roll_reference_unwrapped_rad = 0.0;
  snapshot_.control_roll_reference_capture_tick = 0;
  snapshot_.control_roll_reference_estimator_timestamp_us = 0;
  snapshot_.control_roll_reference_capture_event_sequence =
      control_roll_reference_capture_event_sequence_;
  snapshot_.control_roll_reference_valid = false;
}

void MissionStateMachine::clearFlightAttemptMetadata() {
  snapshot_.forced_start = false;
  snapshot_.preflight_generation = 0;
  snapshot_.preflight_captured_at_us = 0;
  snapshot_.preflight_ready_mask = 0;
  snapshot_.preflight_missing_mask = 0;
}

bool MissionStateMachine::captureControlRollReference(
    const MissionTickInput &input) {
  if (snapshot_.control_roll_reference_valid)
    return true;
  if (input.control.roll_estimator_timestamp_us > input.monotonic_us ||
      input.monotonic_us - input.control.roll_estimator_timestamp_us > 3'000)
    return false;
  ++control_roll_reference_capture_event_sequence_;
  snapshot_.control_roll_reference_unwrapped_rad =
      input.control.roll_estimate_liftoff_relative_unwrapped_rad;
  snapshot_.control_roll_reference_capture_tick = input.control_tick;
  snapshot_.control_roll_reference_estimator_timestamp_us =
      input.control.roll_estimator_timestamp_us;
  snapshot_.control_roll_reference_capture_event_sequence =
      control_roll_reference_capture_event_sequence_;
  snapshot_.control_roll_reference_valid = true;
  return true;
}

} // 名前空間 mission