#pragma once

#include <cstdint>
#include <limits>

#include "protocol/can_protocol.hpp"

namespace mission {

enum class FinDirective : uint8_t { free, brake, zero_hold, roll_control };
enum class ParaDirective : uint8_t { hold, open, powered_off };
enum class StartMode : uint8_t { normal, forced };

enum class PreflightReadinessBit : uint8_t {
  fin_zero_configured = 0,
  motor_profile_valid = 1,
  gyro_bias_valid = 2,
  gravity_reference_valid = 3,
  ssc_zero_valid = 4,
};

constexpr uint8_t kPreflightReadinessMask = 0x1FU;

struct PreflightReadinessSnapshot {
  uint32_t generation{};
  uint64_t captured_at_us{};
  bool fin_zero_configured{};
  bool motor_profile_valid{};
  bool gyro_bias_valid{};
  bool gravity_reference_valid{};
  bool ssc_zero_valid{};
  bool resources_preallocated{};

  [[nodiscard]] uint8_t readyMask() const;
  [[nodiscard]] uint8_t missingMask() const;
  [[nodiscard]] bool normalReady() const;
};

struct ControlAvailability {
  bool fin_control_available{};
  bool fin_zero_hold_valid{};
  bool attitude_valid{};
  bool airspeed_above_60{};
  bool lps_available{};
  bool ssc_available{};
  bool gyro_bias_valid{};
  bool ssc_zero_valid{};
  double roll_estimate_liftoff_relative_unwrapped_rad{
      std::numeric_limits<double>::quiet_NaN()};
  uint64_t roll_estimator_timestamp_us{};
  bool attitude_fresh{};
  [[nodiscard]] bool ready() const;
};

struct MissionTickInput {
  uint64_t monotonic_us{};
  bool liftoff_detected{};
  bool deployment_pressure_condition{};
  ControlAvailability control{};
  uint64_t control_tick{};
};

struct SafetyRequest {
  uint32_t flight_epoch{};
  bool deploy{};
  bool absolute_power_cutoff{};
};

struct MissionSnapshot {
  protocol::MissionState state{protocol::MissionState::command_receive};
  uint32_t flight_epoch{};
  bool liftoff_time_valid{};
  uint64_t liftoff_time_us{};
  uint64_t elapsed_us{};
  bool fin_control_disabled{};
  bool control_reentry_inhibited{};
  bool reset_invalidated{};
  bool deployment_started{};
  bool deployment_power_cutoff_latched{};
  bool forced_start{};
  uint32_t preflight_generation{};
  uint64_t preflight_captured_at_us{};
  uint8_t preflight_ready_mask{};
  uint8_t preflight_missing_mask{};
  double control_roll_reference_unwrapped_rad{};
  uint64_t control_roll_reference_capture_tick{};
  uint64_t control_roll_reference_estimator_timestamp_us{};
  uint8_t control_roll_reference_capture_event_sequence{};
  bool control_roll_reference_valid{};
  FinDirective fin{FinDirective::brake};
  ParaDirective parachute{ParaDirective::hold};
};

enum class TransitionResult : uint8_t {
  completed,
  invalid_state,
  not_configured,
  runtime_unavailable,
};

struct ResetCheckpoint {
  bool valid{};
  protocol::MissionState state{protocol::MissionState::command_receive};
  uint32_t flight_epoch{};
  bool elapsed_valid{};
  uint64_t elapsed_us{};
  bool deployment_started{};
  bool power_cutoff_latched{};
  bool forced_start{};
  uint32_t preflight_generation{};
  uint64_t preflight_captured_at_us{};
  uint8_t preflight_ready_mask{};
  uint8_t preflight_missing_mask{};
};

class MissionStateMachine {
public:
  [[nodiscard]] const MissionSnapshot &snapshot() const { return snapshot_; }
  [[nodiscard]] TransitionResult
  startSequence(uint64_t now_us, const PreflightReadinessSnapshot &readiness,
                StartMode mode);
  [[nodiscard]] TransitionResult cancelSequence();
  [[nodiscard]] TransitionResult disableFinControl();
  [[nodiscard]] TransitionResult liftoffDetectionEmergencyStop();
  [[nodiscard]] TransitionResult
  restoreAfterReset(uint64_t now_us, const ResetCheckpoint &checkpoint);
  void tick(const MissionTickInput &input,
            const SafetyRequest &safety = SafetyRequest{});

private:
  void enterDescent();
  void updateDirectives(uint64_t now_us);
  void invalidateLiftoff();
  void invalidateControlRollReference();
  void clearFlightAttemptMetadata();
  [[nodiscard]] bool captureControlRollReference(const MissionTickInput &input);

  MissionSnapshot snapshot_{};
  bool control_gate_evaluated_{};
  bool fin_control_available_{};
  uint64_t elapsed_offset_us_{};
  uint8_t control_roll_reference_capture_event_sequence_{};
};

} // 名前空間 mission