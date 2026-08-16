#include "actuators/safety_core.hpp"

namespace actuators {

bool PowerArbiter::requestAuxiliary5v(bool enabled) {
  if (enabled && state_.cutoff_latched)
    return false;
  state_.auxiliary_5v = enabled;
  return true;
}

bool PowerArbiter::requestParachutePower(bool enabled) {
  if (enabled && state_.cutoff_latched)
    return false;
  state_.parachute_power = enabled;
  return true;
}

void PowerArbiter::latchDeploymentCutoff() {
  state_.cutoff_latched = true;
  state_.auxiliary_5v = false;
  state_.parachute_power = false;
}

ParachuteAction
ParachuteController::startOpen(uint64_t now_us,
                              uint16_t initial_position_count) {
  if (!AbsoluteParachuteAngle::fromCanonicalCount(initial_position_count)
           .has_value() ||
      status_.state != ParachuteOpenState::idle)
    return ParachuteAction::none;
  status_ = {};
  status_.state = ParachuteOpenState::opening;
  started_at_us_ = now_us;
  window_started_at_us_ = now_us;
  window_position_count_ = initial_position_count;
  return ParachuteAction::command_open;
}

ParachuteAction ParachuteController::tick(const ParachuteTick &input) {
  if (status_.state != ParachuteOpenState::opening &&
      status_.state != ParachuteOpenState::retrying)
    return ParachuteAction::none;
  if (input.now_us < started_at_us_ ||
      input.now_us - started_at_us_ >= kGlobalDeadlineUs) {
    status_.state = ParachuteOpenState::retry_exhausted;
    status_.open_attempt_finished = true;
    return ParachuteAction::stop_retrying;
  }
  if (input.target_reached && input.position_valid) {
    status_.state = ParachuteOpenState::open_confirmed;
    status_.servo_open_confirmed = true;
    status_.open_attempt_finished = true;
    return ParachuteAction::hold_open;
  }
  if (input.now_us < window_started_at_us_ ||
      input.now_us - window_started_at_us_ < kProgressWindowUs)
    return ParachuteAction::none;

  bool progressed = false;
  const auto previous =
      AbsoluteParachuteAngle::fromCanonicalCount(window_position_count_);
  const auto current =
      AbsoluteParachuteAngle::fromCanonicalCount(input.position_count);
  if (input.position_valid && previous.has_value() && current.has_value()) {
    const auto displacement = shortestParachuteDisplacement(*previous, *current);
    progressed = displacement.valid() &&
                 (displacement.counts >= kMinimumProgressCount ||
                  displacement.counts <= -kMinimumProgressCount);
  }
  window_started_at_us_ = input.now_us;
  if (input.position_valid && current.has_value())
    window_position_count_ = input.position_count;
  if (progressed)
    return ParachuteAction::none;
  status_.state = ParachuteOpenState::retrying;
  ++status_.retry_count;
  return ParachuteAction::retry_open;
}

void ParachuteController::notifyPowerCutoff() {
  status_.state = ParachuteOpenState::powered_off;
}

} // 名前空間 actuators
