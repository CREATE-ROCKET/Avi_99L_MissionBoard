#pragma once

#include <cstddef>
#include <cstdint>

#include "config/board_config.hpp"
#include "control/control_pipeline.hpp"
#include "mission/mission_state.hpp"

namespace flight_config {

struct ParachuteConfig {
  float close_position_deg{};
  float open_position_deg{};
  float target_tolerance_deg{};
  float speed_deg_s{};
  float acceleration_deg_s2{};
  float torque_limit_percent{};
  uint32_t power_stabilization_ms{};
  uint32_t initialization_deadline_ms{};
  uint32_t retry_interval_ms{};

  [[nodiscard]] constexpr bool ready() const {
    return open_position_deg != close_position_deg &&
           target_tolerance_deg > 0.0F && speed_deg_s > 0.0F &&
           acceleration_deg_s2 > 0.0F && torque_limit_percent > 0.0F &&
           torque_limit_percent <= 100.0F &&
           initialization_deadline_ms > power_stabilization_ms &&
           retry_interval_ms > 0;
  }
};

struct AirDataConfig {
  // Firmwareが変換に使うsource nominal pressure correction coefficient。
  // Saint-Venantの圧力比ではK^2*q_c/P_sとして適用し、速度へ後掛けしない。
  // Simulation plant truthとは共用しない。
  double pitot_coefficient_assumed{};
  // 飛行中に同定せず、robustness診断の範囲として保持する。
  double pitot_coefficient_true_min{};
  double pitot_coefficient_true_max{};
  double negative_pressure_tolerance_pa{};
  uint16_t zero_calibration_samples{};
  uint8_t moving_average_samples{};

  [[nodiscard]] constexpr bool ready() const {
    return pitot_coefficient_true_min > 0.0 &&
           pitot_coefficient_assumed >= pitot_coefficient_true_min &&
           pitot_coefficient_assumed <= pitot_coefficient_true_max &&
           negative_pressure_tolerance_pa >= 0.0 &&
           zero_calibration_samples > 0 && moving_average_samples > 0;
  }
};

// TODO(HW_TEST): 実機の収納位置、開放位置、速度、加速度、保持torqueを確定する。
inline constexpr ParachuteConfig kParachute{
    0.0F, 90.0F, 2.0F, 180.0F, 360.0F, 20.0F, 100, 1'500, 20};

// source nominalは0.92。true 0.60..1.20は飛行中同定用ではなく、
// TODO(SIMULATION/AERO_VALIDATION): coefficient robustness診断用の暫定範囲。
inline constexpr AirDataConfig kAirData{
    board::kPitotCoefficientDiagnostics.pitot_coefficient_assumed,
    board::kPitotCoefficientDiagnostics.pitot_coefficient_true_min,
    board::kPitotCoefficientDiagnostics.pitot_coefficient_true_max,
    5.0,
    400,
    8};

// TODO(HW_TEST): ADCによる実測値へ置換し、電圧低下時の制御停止条件を決定する。
inline constexpr double kMotorBusVoltageV = 9.0;
// TODO(HW_TEST): 初期HILではbring-upと同じ15%へ制限し、実機同定後に確定する。
inline constexpr double kProductionMotorMaximumDuty = 0.15;

// TODO(SIMULATION): Spicaの同定結果から60～180 m/sのgain tableへ置換する。
// 暫定値は全速度点で同一とし、補間そのものだけをproduction経路で検証する。
inline constexpr control::RollGainSchedule kRollGainSchedule{
    {{{60.0, {0.08, 2.32, 0.04, 0.296}},
      {80.0, {0.08, 2.32, 0.04, 0.296}},
      {100.0, {0.08, 2.32, 0.04, 0.296}},
      {120.0, {0.08, 2.32, 0.04, 0.296}},
      {140.0, {0.08, 2.32, 0.04, 0.296}},
      {160.0, {0.08, 2.32, 0.04, 0.296}},
      {180.0, {0.08, 2.32, 0.04, 0.296}}}},
    true};

inline constexpr mission::SequenceConfiguration kSequenceConfiguration{
    true, true, true, true};

[[nodiscard]] inline bool productionFlightConfigurationReady() {
  return board::kFlightMotorA.parameters_valid &&
         board::kFlightMotorA.polarity !=
             board::MotorPolarity::unconfigured &&
         board::kFinSoftwareLimits.configured &&
         board::kFinSoftwareLimits.minimum_rad <
             board::kFinSoftwareLimits.maximum_rad &&
         kParachute.ready() && kAirData.ready() &&
         board::kControlAuthorityLimits.valid() &&
         board::kEncoderPipeline.valid() &&
         kRollGainSchedule.configured && kSequenceConfiguration.ready() &&
         kMotorBusVoltageV > 0.0 && kProductionMotorMaximumDuty > 0.0 &&
         kProductionMotorMaximumDuty <= 1.0;
}

} // 名前空間 flight_config
