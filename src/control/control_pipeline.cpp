#include "control/control_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include "control/fin_overtravel_guard.hpp"

namespace control {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansPerRpm = 2.0 * kPi / 60.0;
constexpr double kStopperAngleRad = 15.0 * kPi / 180.0;
constexpr double kStaticStructuralTorqueNm = 7.10328;
constexpr double kTotalGearRatio = 176.175;

double clampMagnitude(double value, double limit, bool &saturated) {
  if (std::abs(value) <= limit)
    return value;
  saturated = true;
  return std::copysign(limit, value);
}

bool finiteState(const RollState &state) {
  return std::isfinite(state.roll_deviation_unwrapped_rad) &&
         std::isfinite(state.fin_rad) &&
         std::isfinite(state.roll_rate_rad_s) &&
         std::isfinite(state.fin_rate_rad_s);
}

} // 無名名前空間

TorqueRequest RollController::compute(const RollState &state,
                                      double airspeed_mps,
                                      RollControlAuthority authority,
                                      const board::ControlAuthorityLimits &limits,
                                      RollVerificationMode verification_mode) const {
  const double output_limit_nm =
      authority == RollControlAuthority::gentle
          ? limits.roll_control_gentle_limit_Nm
          : limits.roll_control_high_authority_limit_Nm;
  if (!schedule_.configured || !finiteState(state) ||
      !std::isfinite(airspeed_mps) ||
      !std::isfinite(output_limit_nm) || output_limit_nm <= 0.0)
    return {};
  constexpr std::array<double, 7> kRequiredSpeeds{60.0, 80.0, 100.0, 120.0,
                                                  140.0, 160.0, 180.0};
  for (std::size_t index = 0; index < kRequiredSpeeds.size(); ++index) {
    if (schedule_.points[index].airspeed_mps != kRequiredSpeeds[index])
      return {};
  }
  const double limited_speed = std::clamp(airspeed_mps, 60.0, 180.0);
  std::size_t high = 1;
  while (high < schedule_.points.size() &&
         schedule_.points[high].airspeed_mps < limited_speed)
    ++high;
  high = std::min(high, schedule_.points.size() - 1);
  const std::size_t low = high == 0 ? 0 : high - 1;
  const auto &left = schedule_.points[low];
  const auto &right = schedule_.points[high];
  if (right.airspeed_mps < left.airspeed_mps ||
      left.airspeed_mps < 60.0 || right.airspeed_mps > 180.0)
    return {};
  const double denominator = right.airspeed_mps - left.airspeed_mps;
  const double fraction = denominator > 0.0
                              ? (limited_speed - left.airspeed_mps) /
                                    denominator
                              : 0.0;
  // Control遷移時に取得したunwrapped基準角との差を呼出側から受け取る。
  // ここで最短角へwrapすると、複数回転後に逆方向の目標へ化ける。
  const std::array<double, 4> values{
      state.roll_deviation_unwrapped_rad, state.fin_rad, state.roll_rate_rad_s,
      state.fin_rate_rad_s};
  double torque{};
  for (std::size_t index = 0; index < values.size(); ++index) {
    const double gain = left.gain[index] +
                        fraction * (right.gain[index] - left.gain[index]);
    if (!std::isfinite(gain))
      return {};
    torque -= gain * values[index];
  }
  if (!std::isfinite(torque))
    return {};
  // The matched Control-OFF path deliberately runs the same state, schedule,
  // lookup and finite-value validation above.  Its only changed condition is
  // the final requested torque.
  if (verification_mode == RollVerificationMode::matched_control_off)
    return {0.0, false, true};
  bool saturated = false;
  torque = clampMagnitude(torque, output_limit_nm, saturated);
  return {torque, saturated, true};
}

TorqueRequest ZeroHoldController::compute(double angle_rad,
                                          double rate_rad_s) const {
  if (finOvertravelFaultLatched())
    return {};
  if (!std::isfinite(angle_rad) || !std::isfinite(rate_rad_s) ||
      !std::isfinite(config_.proportional_gain) ||
      !std::isfinite(config_.derivative_gain) ||
      config_.zero_hold_requested_torque_limit_Nm <= 0.0)
    return {};
  bool saturated = false;
  double torque = -config_.proportional_gain * angle_rad -
                  config_.derivative_gain * rate_rad_s;
  torque = clampMagnitude(torque,
                          config_.zero_hold_requested_torque_limit_Nm,
                          saturated);
  return {torque, saturated, true};
}

bool ZeroHoldController::updateValidity(double angle_rad, double rate_rad_s,
                                        bool sample_valid) {
  observeFinOvertravel(angle_rad, sample_valid);
  const bool inside = sample_valid && std::isfinite(angle_rad) &&
                      std::isfinite(rate_rad_s) &&
                      std::abs(angle_rad) <= config_.valid_angle_rad &&
                      std::abs(rate_rad_s) <= config_.valid_rate_rad_s;
  if (!inside)
    valid_count_ = 0;
  else if (valid_count_ < config_.valid_samples)
    ++valid_count_;
  return config_.valid_samples != 0 && valid_count_ >= config_.valid_samples;
}

void QuadraticN3FinVelocityEstimator::reset() { *this = {}; }

bool QuadraticN3FinVelocityEstimator::update(uint64_t timestamp_us,
                                             double angle_rad,
                                             double &rate_rad_s) {
  if (!std::isfinite(angle_rad) ||
      (count_ != 0 && timestamp_us <= points_[count_ - 1].timestamp_us)) {
    reset();
    return false;
  }
  if (count_ < points_.size())
    points_[count_++] = {timestamp_us, angle_rad};
  else {
    points_[0] = points_[1];
    points_[1] = points_[2];
    points_[2] = {timestamp_us, angle_rad};
  }
  if (count_ < points_.size())
    return false;

  // TODO(SIMULATION): N=3 quadraticを実機AS5047D noiseでalpha-betaと再比較する。
  const double t0 = static_cast<double>(
                        static_cast<int64_t>(points_[0].timestamp_us) -
                        static_cast<int64_t>(points_[2].timestamp_us)) *
                    1.0e-6;
  const double t1 = static_cast<double>(
                        static_cast<int64_t>(points_[1].timestamp_us) -
                        static_cast<int64_t>(points_[2].timestamp_us)) *
                    1.0e-6;
  if (t0 == 0.0 || t1 == 0.0 || t0 == t1)
    return false;
  const double derivative_l0 = -t1 / (t0 * (t0 - t1));
  const double derivative_l1 = -t0 / (t1 * (t1 - t0));
  const double derivative_l2 = -(t0 + t1) / (t0 * t1);
  const double candidate = points_[0].angle_rad * derivative_l0 +
                           points_[1].angle_rad * derivative_l1 +
                           points_[2].angle_rad * derivative_l2;
  if (!std::isfinite(candidate))
    return false;
  rate_rad_s = candidate;
  return true;
}

MotorCommand TorqueMapper::map(double requested_output_torque_nm,
                               double fin_angle_rad,
                               double fin_rate_rad_s,
                               double motor_bus_voltage_v) const {
  if (!profile_.parameters_valid ||
      profile_.polarity == board::MotorPolarity::unconfigured ||
      !limits_.configured ||
      profile_.resistance_ohm <= 0.0 ||
      profile_.torque_constant_nm_per_a <= 0.0 ||
      profile_.speed_constant_rpm_per_v <= 0.0 ||
      profile_.drivetrain_efficiency <= 0.0 ||
      profile_.max_motor_current_a <= 0.0 ||
      profile_.max_output_torque_nm <= 0.0 ||
      limits_.minimum_rad >= limits_.maximum_rad ||
      !std::isfinite(requested_output_torque_nm) ||
      !std::isfinite(fin_angle_rad) || !std::isfinite(fin_rate_rad_s) ||
      !std::isfinite(motor_bus_voltage_v) || motor_bus_voltage_v <= 0.0)
    return {};

  MotorCommand result{};
  bool saturated = false;
  double torque = clampMagnitude(requested_output_torque_nm,
                                 profile_.max_output_torque_nm, saturated);
  const bool upper_limit_brake =
      fin_angle_rad >= limits_.maximum_rad && torque >= 0.0;
  const bool lower_limit_brake =
      fin_angle_rad <= limits_.minimum_rad && torque <= 0.0;
  if (upper_limit_brake || lower_limit_brake) {
    // limit位置でtorqueを0にするだけではback-EMF補償電圧が残り、
    // stopper方向へPWMを出し続け得るため、明示的なbrake commandとする。
    return {0.0, 0.0, 0.0, 0.0, true, true, true, true};
  }
  const double motor_torque =
      torque / (kTotalGearRatio * profile_.drivetrain_efficiency);
  double current = motor_torque / profile_.torque_constant_nm_per_a;
  current = clampMagnitude(current, profile_.max_motor_current_a, saturated);
  const double motor_speed_rad_s = fin_rate_rad_s * kTotalGearRatio;
  const double motor_speed_rpm = motor_speed_rad_s / kRadiansPerRpm;
  const double back_emf_v = motor_speed_rpm / profile_.speed_constant_rpm_per_v;
  const double voltage = current * profile_.resistance_ohm + back_emf_v;
  const double duty = std::clamp(std::abs(voltage) / motor_bus_voltage_v,
                                 0.0, 1.0);
  if (std::abs(voltage) > motor_bus_voltage_v)
    saturated = true;
  const bool positive_voltage = voltage >= 0.0;
  const bool positive_in1 =
      profile_.polarity == board::MotorPolarity::positive_in1
          ? positive_voltage
          : !positive_voltage;
  result = {torque, current, voltage, duty, positive_in1, saturated, false,
            true};
  return result;
}

double
DrivetrainStructuralModel::elasticDeflectionRad(double output_torque_nm) {
  return output_torque_nm /
         StructuralParameters::support_stiffness_nm_per_rad;
}

double DrivetrainStructuralModel::conservativeLostMotionRad() {
  return StructuralParameters::measured_backlash_full_width_rad;
}

StopperJudgement
judgeStopperContact(const StopperObservation &observation) {
  if (!std::isfinite(observation.fin_angle_rad) ||
      !std::isfinite(observation.fin_rate_rad_s) ||
      !std::isfinite(observation.static_contact_torque_nm) ||
      std::abs(observation.fin_angle_rad) > kStopperAngleRad ||
      std::abs(observation.static_contact_torque_nm) >
          kStaticStructuralTorqueNm)
    return StopperJudgement::fail;
  if (observation.dynamic_contact)
    // TODO(HW_TEST): dynamic impact許容速度/energyは未qualificationである。
    return StopperJudgement::needs_hardware_validation;
  return StopperJudgement::pass;
}

ControlMode MissionControlCoordinator::choose(bool roll_requested,
                                              bool zero_hold_requested,
                                              bool fin_available,
                                              bool control_inputs_valid,
                                              bool fin_control_disabled) {
  if (!fin_available || fin_control_disabled)
    return ControlMode::brake;
  if (roll_requested && control_inputs_valid)
    return ControlMode::roll_control;
  if (zero_hold_requested)
    return ControlMode::zero_hold;
  return ControlMode::brake;
}

} // 名前空間 control
