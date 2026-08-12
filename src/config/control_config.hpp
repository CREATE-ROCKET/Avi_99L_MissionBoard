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

} // 名前空間 board
