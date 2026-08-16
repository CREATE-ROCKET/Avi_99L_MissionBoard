#pragma once

#include <cstdint>

namespace board {

enum class MotorPolarity : uint8_t {
  unconfigured,
  positive_in1,
  positive_in2,
};

struct MotorProfile {
  uint8_t id;
  MotorPolarity polarity;
  bool parameters_valid;
  float resistance_ohm;
  float torque_constant_nm_per_a;
  float speed_constant_rpm_per_v;
  float drivetrain_efficiency;
  float max_motor_current_a;
  float max_output_torque_nm;
};

struct FinSoftwareLimits {
  bool configured;
  float minimum_rad;
  float maximum_rad;
};

struct AlphaBetaConfig {
  bool configured;
  float alpha;
  float beta;
  uint32_t reset_gap_us;
};

struct ControlAuthorityLimits {
  double hold_position_limit_Nm{0.30};
  double zero_hold_requested_torque_limit_Nm{0.80};
  double roll_control_gentle_limit_Nm{1.21208};
  double roll_control_high_authority_limit_Nm{3.0};

  [[nodiscard]] constexpr bool valid() const {
    return hold_position_limit_Nm > 0.0 &&
           zero_hold_requested_torque_limit_Nm > 0.0 &&
           roll_control_gentle_limit_Nm > 0.0 &&
           roll_control_high_authority_limit_Nm >=
               roll_control_gentle_limit_Nm;
  }
};

struct PitotCoefficientDiagnosticsConfig {
  double pitot_coefficient_assumed{0.92};
  double pitot_coefficient_true_min{0.60};
  double pitot_coefficient_true_max{1.20};

  [[nodiscard]] constexpr bool valid() const {
    return pitot_coefficient_true_min > 0.0 &&
           pitot_coefficient_assumed >= pitot_coefficient_true_min &&
           pitot_coefficient_assumed <= pitot_coefficient_true_max;
  }
};

inline constexpr ControlAuthorityLimits kControlAuthorityLimits{};
inline constexpr PitotCoefficientDiagnosticsConfig
    kPitotCoefficientDiagnostics{};
// TODO(HW_TEST): 実機たわみ・backlash・stopper位置・zero誤差を含めて
// 20 degのovertravel fault閾値を最終確定する。通常の指令可能範囲とは別値。
inline constexpr double kFinOvertravelFaultLimitRad =
    0.34906585039886590;

// productionはAS5047D acquisition/consumerとも1 kHzへ固定する。
// 2/5/10 kHzはcharacterization/offline comparator専用であり、productionの
// EncoderCaptureTask/ring/block assemblerを必要としない。
// 将来production captureを1 kHz超へ戻す場合だけproducer/consumer分離を再導入する。
inline constexpr uint32_t kProductionEncoderAcquisitionHz = 1'000;
inline constexpr uint32_t kProductionEncoderConsumerHz = 1'000;
static_assert(kProductionEncoderAcquisitionHz ==
                  kProductionEncoderConsumerHz,
              "1 kHz production encoder must use direct consumption");

} // namespace board
