#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "config/control_config.hpp"

namespace control {

struct RollState {
  double roll_rad{};
  double fin_rad{};
  double roll_rate_rad_s{};
  double fin_rate_rad_s{};
};

struct GainPoint {
  double airspeed_mps{};
  std::array<double, 4> gain{};
};

struct RollGainSchedule {
  std::array<GainPoint, 7> points{};
  bool configured{};
};

struct TorqueRequest {
  double output_torque_nm{};
  bool saturated{};
  bool valid{};
};

class RollController {
public:
  explicit RollController(const RollGainSchedule &schedule)
      : schedule_(schedule) {}
  [[nodiscard]] TorqueRequest compute(const RollState &state,
                                      double airspeed_mps,
                                      double output_limit_nm) const;
  [[nodiscard]] static double wrapRollError(double roll_rad);

private:
  RollGainSchedule schedule_{};
};

struct ZeroHoldConfig {
  // TODO(SIMULATION): Phase 7/9で実測actuator modelを用いて確定する。
  double proportional_gain{2.32};
  double derivative_gain{0.296};
  double torque_limit_nm{0.80};
  double valid_angle_rad{0.017453292519943295};
  double valid_rate_rad_s{0.08726646259971647};
  uint16_t valid_samples{100};
};

class ZeroHoldController {
public:
  explicit ZeroHoldController(const ZeroHoldConfig &config = {})
      : config_(config) {}
  [[nodiscard]] TorqueRequest compute(double angle_rad,
                                      double rate_rad_s) const;
  [[nodiscard]] bool updateValidity(double angle_rad, double rate_rad_s,
                                    bool sample_valid);
  void resetValidity() { valid_count_ = 0; }

private:
  ZeroHoldConfig config_{};
  uint16_t valid_count_{};
};

class FinVelocityEstimator {
public:
  virtual ~FinVelocityEstimator() = default;
  virtual void reset() = 0;
  [[nodiscard]] virtual bool update(uint64_t timestamp_us,
                                    double unwrapped_angle_rad,
                                    double &rate_rad_s) = 0;
};

class QuadraticN3FinVelocityEstimator final : public FinVelocityEstimator {
public:
  void reset() override;
  [[nodiscard]] bool update(uint64_t timestamp_us,
                            double unwrapped_angle_rad,
                            double &rate_rad_s) override;

private:
  struct Point {
    uint64_t timestamp_us{};
    double angle_rad{};
  };
  std::array<Point, 3> points_{};
  std::size_t count_{};
};

struct MotorCommand {
  double requested_output_torque_nm{};
  double motor_current_a{};
  double motor_voltage_v{};
  double pwm_duty{};
  bool positive_in1{};
  bool saturated{};
  bool brake{true};
  bool valid{};
};

struct StructuralParameters {
  static constexpr double measured_backlash_full_width_rad = 0.00600;
  static constexpr double support_stiffness_nm_per_rad = 269.43;
  static constexpr double stopper_structural_torque_nm = 7.10328;
  static constexpr double stopper_static_proof_torque_nm = 10.65;
};

class DrivetrainStructuralModel {
public:
  [[nodiscard]] static double elasticDeflectionRad(double output_torque_nm);
  [[nodiscard]] static double conservativeLostMotionRad();
};

class TorqueMapper {
public:
  TorqueMapper(const board::MotorProfile &profile,
               const board::FinSoftwareLimits &limits)
      : profile_(profile), limits_(limits) {}
  [[nodiscard]] MotorCommand map(double requested_output_torque_nm,
                                 double fin_angle_rad,
                                 double fin_rate_rad_s,
                                 double motor_bus_voltage_v) const;

private:
  board::MotorProfile profile_{};
  board::FinSoftwareLimits limits_{};
};

enum class StopperJudgement : uint8_t {
  pass,
  fail,
  needs_hardware_validation,
};

struct StopperObservation {
  double fin_angle_rad{};
  double fin_rate_rad_s{};
  double static_contact_torque_nm{};
  bool dynamic_contact{};
};

[[nodiscard]] StopperJudgement
judgeStopperContact(const StopperObservation &observation);

enum class ControlMode : uint8_t { brake, zero_hold, roll_control };

class MissionControlCoordinator {
public:
  [[nodiscard]] static ControlMode choose(bool roll_requested,
                                          bool zero_hold_requested,
                                          bool fin_available,
                                          bool control_inputs_valid,
                                          bool fin_control_disabled);
};

} // 名前空間 control
