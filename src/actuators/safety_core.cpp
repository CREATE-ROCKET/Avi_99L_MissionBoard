#include "actuators/safety_core.hpp"

#include <cmath>

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
ParachuteController::startOpen(uint64_t now_us, double initial_position_deg) {
  if (!std::isfinite(initial_position_deg) ||
      status_.state != ParachuteOpenState::idle)
    return ParachuteAction::none;
  status_ = {};
  status_.state = ParachuteOpenState::opening;
  started_at_us_ = now_us;
  window_started_at_us_ = now_us;
  window_position_deg_ = initial_position_deg;
  return ParachuteAction::command_open;
}

ParachuteAction ParachuteController::tick(const ParachuteTick &input) {
  if (status_.state != ParachuteOpenState::opening &&
      status_.state != ParachuteOpenState::retrying)
    return ParachuteAction::none;
  if (input.now_us < started_at_us_ ||
      input.now_us - started_at_us_ >= kGlobalDeadlineUs) {
    status_.state = ParachuteOpenState::retry_exhausted;
    status_.power_cutoff_requested = true;
    return ParachuteAction::cut_power;
  }
  if (input.target_reached && input.position_valid) {
    status_.state = ParachuteOpenState::open_confirmed;
    status_.servo_open_confirmed = true;
    status_.power_cutoff_requested = true;
    return ParachuteAction::cut_power;
  }
  if (input.now_us < window_started_at_us_ ||
      input.now_us - window_started_at_us_ < kProgressWindowUs)
    return ParachuteAction::none;

  const bool progressed = input.position_valid &&
                          std::isfinite(input.position_deg) &&
                          std::abs(input.position_deg - window_position_deg_) >=
                              kMinimumProgressDeg;
  window_started_at_us_ = input.now_us;
  if (input.position_valid && std::isfinite(input.position_deg))
    window_position_deg_ = input.position_deg;
  if (progressed)
    return ParachuteAction::none;
  status_.state = ParachuteOpenState::retrying;
  ++status_.retry_count;
  return ParachuteAction::retry_open;
}

void ParachuteController::notifyPowerCutoff() {
  status_.power_cutoff_requested = true;
  status_.state = ParachuteOpenState::powered_off;
}

} // 名前空間 actuators
