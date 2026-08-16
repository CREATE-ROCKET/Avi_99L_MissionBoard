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

struct EncoderPipelineConfig {
  uint32_t acquisition_hz{1'000};
  uint32_t consumer_hz{1'000};

  [[nodiscard]] constexpr bool valid() const {
    return consumer_hz != 0 && acquisition_hz >= consumer_hz &&
           acquisition_hz % consumer_hz == 0;
  }
  [[nodiscard]] constexpr uint32_t samplesPerBlock() const {
    return valid() ? acquisition_hz / consumer_hz : 0;
  }
};

inline constexpr ControlAuthorityLimits kControlAuthorityLimits{};
inline constexpr PitotCoefficientDiagnosticsConfig
    kPitotCoefficientDiagnostics{};
// TODO(HW_TEST): 実機たわみ・backlash・stopper位置・zero誤差を含めて
// 20 degのovertravel fault閾値を最終確定する。通常の指令可能範囲とは別値。
inline constexpr double kFinOvertravelFaultLimitRad =
    0.34906585039886590;
// TODO(HW_TEST): 1/2 kHzの最終選択後も本configだけを差し替える。
// 現在のproduction producer/consumerは1 kHz暫定である。
inline constexpr EncoderPipelineConfig kEncoderPipeline{1'000, 1'000};

} // 名前空間 board
