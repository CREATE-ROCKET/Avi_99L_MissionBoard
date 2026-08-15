#include "characterization/position_guard.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>

namespace avi::characterization {
namespace {

PositionGuardDecision abortDecision(PositionGuardReason reason) noexcept {
  return {PositionGuardAction::abort_run, reason,
          MotorRequest{0, MotorMode::coast}};
}

PositionGuardDecision coastDecision(PositionGuardReason reason) noexcept {
  return {PositionGuardAction::force_coast, reason,
          MotorRequest{0, MotorMode::coast}};
}

} // 無名名前空間

std::int32_t PositionGuard::sign(std::int32_t value) noexcept {
  if (value > 0)
    return 1;
  if (value < 0)
    return -1;
  return 0;
}

std::int32_t PositionGuard::saturatingAdd(std::int32_t lhs,
                                          std::int32_t rhs) noexcept {
  const std::int64_t sum = static_cast<std::int64_t>(lhs) + rhs;
  return static_cast<std::int32_t>(std::clamp<std::int64_t>(
      sum, std::numeric_limits<std::int32_t>::min(),
      std::numeric_limits<std::int32_t>::max()));
}

PositionGuardDecision
PositionGuard::evaluate(const PositionGuardInput &input) const noexcept {
  if (config_.normal_envelope_millideg <= 0 ||
      config_.hard_abort_millideg <=
          config_.normal_envelope_millideg ||
      config_.hard_abort_millideg >= kFinMechanicalLimitMilliDeg) {
    return abortDecision(PositionGuardReason::configuration_invalid);
  }
  if (config_.command_to_fin_sign != 1 &&
      config_.command_to_fin_sign != -1) {
    return abortDecision(PositionGuardReason::polarity_unconfigured);
  }
  if (!input.encoder_valid)
    return abortDecision(PositionGuardReason::encoder_invalid);
  if (!input.consumer_deadline_met)
    return abortDecision(PositionGuardReason::consumer_deadline_miss);
  if (!motorModeMatchesCommand(input.requested.command_permille,
                               input.requested.requested_mode)) {
    return abortDecision(PositionGuardReason::command_mode_mismatch);
  }

  const std::int64_t absolute_angle =
      std::llabs(static_cast<long long>(input.fin_angle_millideg));
  if (absolute_angle >= config_.hard_abort_millideg)
    return abortDecision(PositionGuardReason::hard_limit);

  const std::int32_t predicted_angle =
      saturatingAdd(input.fin_angle_millideg,
                    input.predicted_stopping_delta_millideg);
  const std::int64_t absolute_predicted =
      std::llabs(static_cast<long long>(predicted_angle));
  if (absolute_predicted >= config_.hard_abort_millideg) {
    return abortDecision(
        PositionGuardReason::predicted_stop_exceeds_limit);
  }

  const std::int32_t command_direction =
      sign(input.requested.command_permille) *
      static_cast<std::int32_t>(config_.command_to_fin_sign);
  const bool at_positive_envelope =
      input.fin_angle_millideg >= config_.normal_envelope_millideg;
  const bool at_negative_envelope =
      input.fin_angle_millideg <= -config_.normal_envelope_millideg;
  const bool command_is_outward =
      (at_positive_envelope && command_direction > 0) ||
      (at_negative_envelope && command_direction < 0);
  if (command_is_outward) {
    return coastDecision(PositionGuardReason::soft_limit_outward);
  }

  return {PositionGuardAction::allow, PositionGuardReason::none,
          input.requested};
}

} // 名前空間 avi::characterization
