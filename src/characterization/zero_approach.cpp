#include "characterization/zero_approach.hpp"

#include <cstdint>

namespace avi::characterization {

std::array<std::int32_t, ZeroApproachPlan::kTargetCount>
ZeroApproachPlan::makeTargets(ApproachBranch branch) noexcept {
  std::array<std::int32_t, kTargetCount> values{};
  std::int32_t sign = 0;
  if (branch == ApproachBranch::from_positive)
    sign = 1;
  else if (branch == ApproachBranch::from_negative)
    sign = -1;
  for (std::size_t index = 0; index < values.size(); ++index) {
    const auto remaining = static_cast<std::int32_t>(values.size() - 1U - index);
    values[index] = sign * remaining * kApproachStepMilliDeg;
  }
  return values;
}

ZeroApproachPlan::ZeroApproachPlan(
    ApproachBranch branch, std::int32_t tolerance_millideg,
    std::int32_t rate_tolerance_millideg_s,
    std::uint16_t dwell_samples,
    std::int32_t monotonic_tolerance_millideg) noexcept
    : branch_(branch), targets_(makeTargets(branch)),
      tolerance_millideg_(tolerance_millideg),
      rate_tolerance_millideg_s_(rate_tolerance_millideg_s),
      required_dwell_samples_(dwell_samples),
      monotonic_tolerance_millideg_(monotonic_tolerance_millideg),
      valid_((branch == ApproachBranch::from_positive ||
              branch == ApproachBranch::from_negative) &&
             tolerance_millideg >= 0 &&
             rate_tolerance_millideg_s >= 0 && dwell_samples != 0U &&
             monotonic_tolerance_millideg >= 0) {}

std::int32_t ZeroApproachPlan::targetMilliDeg() const noexcept {
  return targets_[index_];
}

ApproachUpdate ZeroApproachPlan::update(
    std::int32_t measured_millideg,
    std::int32_t rate_millideg_s) noexcept {
  if (!valid_)
    return ApproachUpdate::configuration_abort;
  if (complete_)
    return ApproachUpdate::complete;

  constexpr std::int32_t kOvershootMarginMilliDeg = 50;
  if ((branch_ == ApproachBranch::from_positive &&
       measured_millideg < -kOvershootMarginMilliDeg) ||
      (branch_ == ApproachBranch::from_negative &&
       measured_millideg > kOvershootMarginMilliDeg)) {
    dwell_samples_ = 0;
    return ApproachUpdate::overshoot_abort;
  }

  if (approach_to_zero_started_ && have_last_measured_) {
    const std::int64_t measured = measured_millideg;
    const std::int64_t previous = last_measured_millideg_;
    const std::int64_t monotonic_tolerance =
        monotonic_tolerance_millideg_;
    const bool reversed =
        (branch_ == ApproachBranch::from_positive &&
         measured > previous + monotonic_tolerance) ||
        (branch_ == ApproachBranch::from_negative &&
         measured < previous - monotonic_tolerance);
    if (reversed) {
      dwell_samples_ = 0;
      return ApproachUpdate::monotonicity_abort;
    }
  }
  last_measured_millideg_ = measured_millideg;
  have_last_measured_ = true;

  const std::int64_t angle_delta =
      static_cast<std::int64_t>(measured_millideg) - targetMilliDeg();
  const std::int64_t rate = rate_millideg_s;
  const bool angle_settled =
      angle_delta >= -static_cast<std::int64_t>(tolerance_millideg_) &&
      angle_delta <= static_cast<std::int64_t>(tolerance_millideg_);
  const bool rate_settled =
      rate >= -static_cast<std::int64_t>(rate_tolerance_millideg_s_) &&
      rate <= static_cast<std::int64_t>(rate_tolerance_millideg_s_);
  if (!angle_settled || !rate_settled) {
    dwell_samples_ = 0;
    return ApproachUpdate::holding;
  }

  if (dwell_samples_ < required_dwell_samples_)
    ++dwell_samples_;
  if (dwell_samples_ < required_dwell_samples_)
    return ApproachUpdate::holding;

  dwell_samples_ = 0;
  if (index_ + 1U >= targets_.size()) {
    complete_ = true;
    return ApproachUpdate::complete;
  }
  ++index_;
  if (index_ == 1U)
    approach_to_zero_started_ = true;
  return ApproachUpdate::target_advanced;
}

ApproachUpdate ZeroApproachController::update(
    std::int32_t measured_millideg, std::int32_t rate_millideg_s,
    std::uint64_t now_us) noexcept {
  if (!started_)
    reset(now_us);
  if (now_us < started_at_us_ ||
      now_us - started_at_us_ > timeout_us_)
    return ApproachUpdate::timeout_abort;
  return plan_.update(measured_millideg, rate_millideg_s);
}

} // 名前空間 avi::characterization
