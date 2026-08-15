#pragma once

#include <cstddef>
#include <cstdint>

#include "config/board_config.hpp"
#include "control/control_pipeline.hpp"

namespace flight_config {

struct ParachuteMotionConfig {
  float target_tolerance_deg{};
  float speed_deg_s{};
  float acceleration_deg_s2{};
  float torque_limit_percent{};
  uint32_t power_stabilization_ms{};
  uint32_t initialization_deadline_ms{};
  uint32_t retry_interval_ms{};

  [[nodiscard]] constexpr bool ready() const {
    return target_tolerance_deg > 0.0F && speed_deg_s > 0.0F &&
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
inline constexpr ParachuteMotionConfig kParachute{
    2.0F, 180.0F, 360.0F, 20.0F, 100, 1'500, 20};
// Vault 01: CommandReceiveの未接続STSは1秒周期で再接続する。
// 飛行中の初期化retryは従来のkParachute.retry_interval_msを維持する。
inline constexpr uint32_t kParachuteCommandReceiveReconnectMs = 1'000;

// source nominalは0.92。true 0.60..1.20は飛行中同定用ではなく、
// TODO(SIMULATION/AERO_VALIDATION): coefficient robustness診断用の暫定範囲。
inline constexpr AirDataConfig kAirData{
    board::kPitotCoefficientDiagnostics.pitot_coefficient_assumed,
    board::kPitotCoefficientDiagnostics.pitot_coefficient_true_min,
    board::kPitotCoefficientDiagnostics.pitot_coefficient_true_max,
    5.0,
    400,
    8};

#if !defined(AVI_99L_MOTOR_PROFILE_ID)
#error "AVI_99L_MOTOR_PROFILE_ID must be defined for every MissionBoard build"
#elif AVI_99L_MOTOR_PROFILE_ID == 1
inline constexpr const board::MotorProfile &kActiveFlightMotorProfile =
    board::kFlightMotorA;
inline constexpr bool kActiveFlightMotorProfileQualified =
    board::kFlightMotorAFlightQualified;
#elif AVI_99L_MOTOR_PROFILE_ID == 2
inline constexpr const board::MotorProfile &kActiveFlightMotorProfile =
    board::kSpareMotorB;
inline constexpr bool kActiveFlightMotorProfileQualified =
    board::kSpareMotorBFlightQualified;
#else
#error "AVI_99L_MOTOR_PROFILE_ID must be 1 or 2"
#endif

// TODO(HW_TEST): ADCによる実測値へ置換し、電圧低下時の制御停止条件を決定する。
inline constexpr double kMotorBusVoltageV = 9.0;
#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION
// Characterization profileは最大30%までを実測し、wire contractは35%を上限とする。
// production flightの15%暫定limitを流用すると、logger上30%でも実PWMが15%へ
// silent clampされるため、専用buildだけdriver上限を35%へ広げる。
inline constexpr double kProductionMotorMaximumDuty = 0.35;
#else
// productionではTorqueMapperのcurrent/torque/angle制限を維持したまま、
// PWM dutyだけを追加clampせず100%まで許可する。
inline constexpr double kProductionMotorMaximumDuty = 1.0;
#endif

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

[[nodiscard]] inline bool motorProfileValid() {
  return kActiveFlightMotorProfile.parameters_valid &&
         kActiveFlightMotorProfile.polarity != board::MotorPolarity::unconfigured &&
         kActiveFlightMotorProfileQualified;
}

[[nodiscard]] inline bool nonBypassFlightConfigurationReady() {
  return board::kFinSoftwareLimits.configured &&
         board::kFinSoftwareLimits.minimum_rad <
             board::kFinSoftwareLimits.maximum_rad &&
         kParachute.ready() && kAirData.ready() &&
         board::kControlAuthorityLimits.valid() &&
         board::kEncoderPipeline.valid() &&
         kRollGainSchedule.configured && kMotorBusVoltageV > 0.0 &&
         kProductionMotorMaximumDuty > 0.0 &&
         kProductionMotorMaximumDuty <= 1.0;
}

[[nodiscard]] inline bool productionFlightConfigurationReady() {
  return motorProfileValid() && nonBypassFlightConfigurationReady();
}

} // 名前空間 flight_config
