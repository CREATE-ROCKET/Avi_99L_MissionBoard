#pragma once

#include "characterization/characterization_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace avi::characterization {

enum class ApproachUpdate : std::uint8_t {
  holding = 0,
  target_advanced = 1,
  complete = 2,
  overshoot_abort = 3,
  monotonicity_abort = 4,
  configuration_abort = 5,
  timeout_abort = 6,
};

class ZeroApproachPlan {
public:
  static constexpr std::size_t kTargetCount = 11U;

  explicit ZeroApproachPlan(ApproachBranch branch,
                            std::int32_t tolerance_millideg = 20,
                            std::int32_t rate_tolerance_millideg_s = 100,
                            std::uint16_t dwell_samples = 50,
                            std::int32_t monotonic_tolerance_millideg = 50)
      noexcept;

  [[nodiscard]] std::int32_t targetMilliDeg() const noexcept;
  [[nodiscard]] ApproachUpdate
  update(std::int32_t measured_millideg,
         std::int32_t rate_millideg_s) noexcept;
  [[nodiscard]] bool complete() const noexcept { return complete_; }
  [[nodiscard]] bool valid() const noexcept { return valid_; }
  [[nodiscard]] ApproachBranch branch() const noexcept { return branch_; }
  [[nodiscard]] const std::array<std::int32_t, kTargetCount> &targets() const
      noexcept {
    return targets_;
  }

private:
  static std::array<std::int32_t, kTargetCount>
  makeTargets(ApproachBranch branch) noexcept;

  ApproachBranch branch_{ApproachBranch::from_positive};
  std::array<std::int32_t, kTargetCount> targets_{};
  std::size_t index_{0};
  std::int32_t tolerance_millideg_{20};
  std::int32_t rate_tolerance_millideg_s_{100};
  std::uint16_t required_dwell_samples_{50};
  std::uint16_t dwell_samples_{0};
  std::int32_t monotonic_tolerance_millideg_{50};
  std::int32_t last_measured_millideg_{0};
  bool have_last_measured_{false};
  bool approach_to_zero_started_{false};
  bool complete_{false};
  bool valid_{false};
};

class ZeroApproachController {
public:
  explicit ZeroApproachController(
      ApproachBranch branch, std::uint64_t timeout_us = 18'000'000U,
      std::int32_t tolerance_millideg = 20,
      std::int32_t rate_tolerance_millideg_s = 100,
      std::uint16_t dwell_samples = 50) noexcept
      : plan_(branch, tolerance_millideg, rate_tolerance_millideg_s,
              dwell_samples),
        timeout_us_(timeout_us) {}

  void reset(std::uint64_t started_at_us) noexcept {
    started_at_us_ = started_at_us;
    started_ = true;
  }
  [[nodiscard]] ApproachUpdate
  update(std::int32_t measured_millideg,
         std::int32_t rate_millideg_s, std::uint64_t now_us) noexcept;
  [[nodiscard]] std::int32_t targetMilliDeg() const noexcept {
    return plan_.targetMilliDeg();
  }
  [[nodiscard]] bool complete() const noexcept { return plan_.complete(); }
  [[nodiscard]] const std::array<std::int32_t,
                                 ZeroApproachPlan::kTargetCount> &
  targets() const noexcept {
    return plan_.targets();
  }

private:
  ZeroApproachPlan plan_;
  std::uint64_t timeout_us_{0U};
  std::uint64_t started_at_us_{0U};
  bool started_{false};
};

} // 名前空間 avi::characterization
