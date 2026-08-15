#pragma once

#include "characterization/characterization_types.hpp"

#include <cstdint>

namespace avi::characterization {

struct PositionGuardConfig {
  std::int32_t normal_envelope_millideg{kRoutineEnvelopeMilliDeg};
  std::int32_t hard_abort_millideg{kHardAbortMilliDeg};
  // +1: 正commandでfin角が増加、-1: 正commandでfin角が減少。
  // 実機polarity確認前は0としてarmを拒否する。
  std::int8_t command_to_fin_sign{0};
};

struct PositionGuardInput {
  std::int32_t fin_angle_millideg{0};
  std::int32_t fin_rate_millideg_s{0};
  // 現在状態からcommandを追加せずcoastした場合の、符号付き予測移動量。
  std::int32_t predicted_stopping_delta_millideg{0};
  MotorRequest requested{};
  bool encoder_valid{false};
  bool consumer_deadline_met{false};
};

struct PositionGuardDecision {
  PositionGuardAction action{PositionGuardAction::abort_run};
  PositionGuardReason reason{PositionGuardReason::polarity_unconfigured};
  MotorRequest permitted{};
};

class PositionGuard {
public:
  explicit PositionGuard(PositionGuardConfig config) noexcept
      : config_(config) {}

  [[nodiscard]] PositionGuardDecision
  evaluate(const PositionGuardInput &input) const noexcept;

  [[nodiscard]] const PositionGuardConfig &config() const noexcept {
    return config_;
  }

private:
  [[nodiscard]] static std::int32_t sign(std::int32_t value) noexcept;
  [[nodiscard]] static std::int32_t saturatingAdd(std::int32_t lhs,
                                                  std::int32_t rhs) noexcept;

  PositionGuardConfig config_{};
};

} // 名前空間 avi::characterization
