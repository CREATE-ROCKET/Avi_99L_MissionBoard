#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace protocol {

enum class CanId : uint16_t {
  actuator_emergency_stop = 0x001,
  liftoff_detection_emergency_stop = 0x002,
  recovery_control = 0x008,
  generic_command_request = 0x010,
  command_result = 0x011,
  time_request = 0x012,
  time_response = 0x013,
  recovery_mode_command = 0x014,
  mission_event = 0x020,
  kinematics_telemetry = 0x100,
  control_telemetry = 0x101,
  mission_status_telemetry = 0x102,
  power_time_telemetry = 0x103,
  descent_core_telemetry = 0x104,
  recovery_status = 0x105,
  recovery_log_data = 0x106,
  attitude_tilt_telemetry = 0x107,
  lps_telemetry = 0x108,
  airspeed_telemetry = 0x109,
  control_roll_telemetry_v2 = 0x10A,
};

enum class MissionState : uint8_t {
  command_receive = 0,
  liftoff_detection = 1,
  engine_burn = 2,
  control = 3,
  descent = 4,
  unknown = 0xFF,
};

enum class CommandPhase : uint8_t {
  accepted = 0,
  completed = 1,
  rejected = 2,
  failed = 3,
};

enum class CommandReason : uint8_t {
  none = 0,
  busy = 1,
  invalid_state = 2,
  invalid_argument = 3,
  not_configured = 4,
  device_unavailable = 5,
  timeout = 6,
  stall = 7,
  protocol_error = 8,
  interrupted_by_emergency = 9,
  persistence_error = 10,
  internal_error = 11,
  not_supported = 12,
  safety_interlock = 13,
  already_satisfied = 14,
};

enum class TimeSource : uint8_t { invalid = 0, gnss = 1, ground = 2 };
enum class RecoveryOpcode : uint8_t {
  wake = 1,
  start_log_dump = 2,
  stop_log_dump = 3,
};
enum class RecoveryMode : uint8_t { enter_recovery_beacon = 1 };
enum class RecoveryModeReason : uint8_t {
  auto_elapsed_120 = 0,
  ground_requested = 1,
  recovery_wake_retry = 2,
  reset_recovery = 3,
};
enum class RecoverySource : uint8_t {
  internal_flash = 0,
  mission_sd_latest_flight = 1,
};
enum class RecoveryStatusCode : uint8_t {
  ready = 0,
  dumping = 1,
  complete = 2,
  busy = 3,
  invalid_state = 4,
  invalid_argument = 5,
  source_unavailable = 6,
  io_error = 7,
  aborted = 8,
  internal_error = 9,
};
enum class MissionEventFlag : uint16_t {
  icm_data_loss_or_error = 1U << 0U,
  encoder_error = 1U << 1U,
  air_data_error = 1U << 2U,
  fin_motor_saturation = 1U << 3U,
  reset_or_recovery = 1U << 4U,
  state_changed = 1U << 5U,
  liftoff_detected = 1U << 6U,
  parachute_open_started = 1U << 7U,
  deployment_power_cutoff = 1U << 8U,
  persistence_error = 1U << 9U,
  mission_sd_error = 1U << 10U,
  can_recovery = 1U << 11U,
  liftoff_emergency_rollback = 1U << 12U,
  actuator_emergency_stop = 1U << 13U,
  fin_control_disabled_by_ground = 1U << 14U,
  parachute_deployment_failure = 1U << 15U,
};

[[nodiscard]] constexpr uint16_t eventFlag(MissionEventFlag flag) {
  return static_cast<uint16_t>(flag);
}
enum class FinMode : uint8_t {
  free = 0,
  brake = 1,
  position_hold = 2,
  zero_hold = 3,
  relative_move = 4,
  roll_control = 5,
  unknown = 15,
};
enum class ParaMode : uint8_t {
  free = 0,
  hold = 1,
  relative_move = 2,
  opening_or_retrying = 3,
  closing = 4,
  powered_off = 5,
  unknown = 15,
};

struct CanFrame {
  uint16_t identifier{};
  uint8_t data_length{};
  std::array<uint8_t, 8> data{};
  bool extended{};
  bool remote{};
};

enum class CodecError : uint8_t {
  none,
  wrong_identifier,
  wrong_length,
  unsupported_frame,
  reserved_bits,
  invalid_enum,
};

struct EmergencyStop {
  uint8_t transaction_id{};
};

struct RecoveryControl {
  RecoveryOpcode opcode{};
  RecoverySource source{};
  uint8_t transfer_id{};
  uint32_t offset{};
  uint32_t length{};
};

struct RecoveryModeCommand {
  uint8_t sequence{};
  RecoveryMode mode{RecoveryMode::enter_recovery_beacon};
  RecoveryModeReason reason{RecoveryModeReason::auto_elapsed_120};
};

struct GenericCommandRequest {
  uint8_t transaction_id{};
  uint8_t command{};
  std::array<uint8_t, 6> arguments{};
};

struct CommandResult {
  uint8_t transaction_id{};
  uint8_t command{};
  CommandPhase phase{};
  CommandReason reason{};
  uint32_t detail{};
};

struct TimeRequest {
  uint8_t request_id{};
};

struct TimeResponse {
  uint8_t request_id{};
  TimeSource source{};
  uint32_t unix_seconds{};
  uint16_t milliseconds{};
};

struct MissionEvent {
  uint8_t sequence{};
  uint16_t flags{};
  MissionState state{};
  uint16_t elapsed_raw{};
  uint16_t detail{};
};

struct KinematicsTelemetry {
  uint8_t sequence{};
  uint16_t roll_raw{};
  uint16_t roll_rate_raw{};
  uint8_t fin_angle_raw{};
  uint16_t fin_rate_raw{};
};

struct ControlTelemetry {
  uint8_t sequence{};
  uint16_t requested_torque_raw{};
  uint8_t flight_elapsed_raw{};
};

struct MissionStatusTelemetry {
  uint8_t sequence{};
  MissionState state{};
  uint16_t flight_status{};
  uint8_t config_flags{};
  FinMode fin_mode{};
  ParaMode para_mode{};
  uint8_t parachute_angle_raw{};
};

struct PowerTimeTelemetry {
  uint8_t sequence{};
  uint8_t logic_voltage_raw{};
  uint8_t motor_voltage_raw{};
  uint16_t descent_elapsed_raw{};
  uint16_t recovery_elapsed_raw{};
  uint8_t persistence_flags{};
};

struct DescentCoreTelemetry {
  uint8_t sequence{};
  uint16_t descent_status{};
  uint8_t parachute_angle_raw{};
};

struct RecoveryStatusMessage {
  RecoveryOpcode opcode{};
  uint8_t transfer_id{};
  RecoveryStatusCode status{};
  RecoverySource source{};
  uint32_t total_size{};
};

struct RecoveryLogData {
  uint8_t transfer_id{};
  uint8_t sequence{};
  std::array<uint8_t, 6> data{};
};

struct AttitudeTiltTelemetry {
  uint8_t sequence{};
  uint8_t magnitude_raw{};
  uint16_t direction_raw{};
};

struct LpsTelemetry {
  uint8_t sequence{};
  uint16_t pressure_raw{};
  uint8_t temperature_raw{};
};

struct AirspeedTelemetry {
  uint8_t sequence{};
  uint8_t airspeed_raw{};
};

struct ControlRollTelemetryV2 {
  static constexpr uint8_t schema_version = 2;
  static constexpr uint8_t reference_valid = 1U << 0U;
  static constexpr uint8_t reference_captured_since_previous_frame = 1U << 1U;
  static constexpr uint8_t control_active = 1U << 2U;
  static constexpr uint8_t reference_out_of_range = 1U << 3U;
  static constexpr uint8_t deviation_out_of_range = 1U << 4U;

  uint8_t sequence{};
  uint16_t control_roll_reference_unwrapped_raw{};
  uint16_t roll_deviation_unwrapped_raw{};
  uint8_t flags{};
  uint8_t reference_capture_event_sequence{};
};

[[nodiscard]] uint32_t controlRollStatusSignature(uint16_t reference_raw,
                                                  uint16_t deviation_raw,
                                                  uint8_t flags);

[[nodiscard]] CanFrame encode(CanId id, const EmergencyStop &message);
[[nodiscard]] CanFrame encode(const RecoveryControl &message);
[[nodiscard]] CanFrame encode(const RecoveryModeCommand &message);
[[nodiscard]] CanFrame encode(const GenericCommandRequest &message);
[[nodiscard]] CanFrame encode(const CommandResult &message);
[[nodiscard]] CanFrame encode(const TimeRequest &message);
[[nodiscard]] CanFrame encode(const TimeResponse &message);
[[nodiscard]] CanFrame encode(const MissionEvent &message);
[[nodiscard]] CanFrame encode(const KinematicsTelemetry &message);
[[nodiscard]] CanFrame encode(const ControlTelemetry &message);
[[nodiscard]] CanFrame encode(const MissionStatusTelemetry &message);
[[nodiscard]] CanFrame encode(const PowerTimeTelemetry &message);
[[nodiscard]] CanFrame encode(const DescentCoreTelemetry &message);
[[nodiscard]] CanFrame encode(const RecoveryStatusMessage &message);
[[nodiscard]] CanFrame encode(const RecoveryLogData &message);
[[nodiscard]] CanFrame encode(const AttitudeTiltTelemetry &message);
[[nodiscard]] CanFrame encode(const LpsTelemetry &message);
[[nodiscard]] CanFrame encode(const AirspeedTelemetry &message);
[[nodiscard]] CanFrame encode(const ControlRollTelemetryV2 &message);

[[nodiscard]] CodecError decode(const CanFrame &frame, CanId id,
                                EmergencyStop &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                RecoveryControl &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                RecoveryModeCommand &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                GenericCommandRequest &message);
[[nodiscard]] CodecError decode(const CanFrame &frame, CommandResult &message);
[[nodiscard]] CodecError decode(const CanFrame &frame, TimeRequest &message);
[[nodiscard]] CodecError decode(const CanFrame &frame, TimeResponse &message);
[[nodiscard]] CodecError decode(const CanFrame &frame, MissionEvent &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                KinematicsTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                ControlTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                MissionStatusTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                PowerTimeTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                DescentCoreTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                RecoveryStatusMessage &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                RecoveryLogData &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                AttitudeTiltTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame, LpsTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                AirspeedTelemetry &message);
[[nodiscard]] CodecError decode(const CanFrame &frame,
                                ControlRollTelemetryV2 &message);

[[nodiscard]] uint16_t canPeriodMilliseconds(CanId id);

class TelemetrySequences {
public:
  [[nodiscard]] uint8_t next(CanId id);

private:
  std::array<uint8_t, 11> counters_{};
};

} // 名前空間 protocol
