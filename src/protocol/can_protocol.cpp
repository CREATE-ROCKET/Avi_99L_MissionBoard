#include "protocol/can_protocol.hpp"
#include "protocol/quantization.hpp"

#include <algorithm>

namespace protocol {
namespace {

CanFrame frame(CanId id, uint8_t length) {
  CanFrame result{};
  result.identifier = static_cast<uint16_t>(id);
  result.data_length = length;
  return result;
}

void putU16(std::array<uint8_t, 8> &data, std::size_t offset, uint16_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1] = static_cast<uint8_t>(value >> 8U);
}

void putU24(std::array<uint8_t, 8> &data, std::size_t offset, uint32_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1] = static_cast<uint8_t>(value >> 8U);
  data[offset + 2] = static_cast<uint8_t>(value >> 16U);
}

void putU32(std::array<uint8_t, 8> &data, std::size_t offset, uint32_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1] = static_cast<uint8_t>(value >> 8U);
  data[offset + 2] = static_cast<uint8_t>(value >> 16U);
  data[offset + 3] = static_cast<uint8_t>(value >> 24U);
}

uint16_t getU16(const std::array<uint8_t, 8> &data, std::size_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(data[offset + 1]) << 8U;
}

uint32_t getU24(const std::array<uint8_t, 8> &data, std::size_t offset) {
  return static_cast<uint32_t>(data[offset]) |
         static_cast<uint32_t>(data[offset + 1]) << 8U |
         static_cast<uint32_t>(data[offset + 2]) << 16U;
}

uint32_t getU32(const std::array<uint8_t, 8> &data, std::size_t offset) {
  return static_cast<uint32_t>(data[offset]) |
         static_cast<uint32_t>(data[offset + 1]) << 8U |
         static_cast<uint32_t>(data[offset + 2]) << 16U |
         static_cast<uint32_t>(data[offset + 3]) << 24U;
}

CodecError validate(const CanFrame &input, CanId id, uint8_t length) {
  if (input.extended || input.remote)
    return CodecError::unsupported_frame;
  if (input.identifier != static_cast<uint16_t>(id))
    return CodecError::wrong_identifier;
  return input.data_length == length ? CodecError::none
                                     : CodecError::wrong_length;
}

bool validMissionState(MissionState value) {
  return static_cast<uint8_t>(value) <=
             static_cast<uint8_t>(MissionState::descent) ||
         value == MissionState::unknown;
}

bool validFinMode(FinMode value) {
  return static_cast<uint8_t>(value) <=
             static_cast<uint8_t>(FinMode::roll_control) ||
         value == FinMode::unknown;
}

bool validParaMode(ParaMode value) {
  return static_cast<uint8_t>(value) <=
             static_cast<uint8_t>(ParaMode::powered_off) ||
         value == ParaMode::unknown;
}

uint8_t controlRollQualityCode(uint16_t raw) {
  return raw >= 0x8000U && raw <= 0x800FU
             ? static_cast<uint8_t>((raw & 0x000FU) + 1U)
             : 0U;
}

std::size_t sequenceIndex(CanId id) {
  switch (id) {
  case CanId::kinematics_telemetry:
    return 0;
  case CanId::control_telemetry:
    return 1;
  case CanId::mission_status_telemetry:
    return 2;
  case CanId::power_time_telemetry:
    return 3;
  case CanId::descent_core_telemetry:
    return 4;
  case CanId::recovery_status:
    return 5;
  case CanId::recovery_log_data:
    return 6;
  case CanId::attitude_tilt_telemetry:
    return 7;
  case CanId::lps_telemetry:
    return 8;
  case CanId::airspeed_telemetry:
    return 9;
  case CanId::control_roll_telemetry_v2:
    return 10;
  default:
    return 0;
  }
}

} // 無名名前空間

uint32_t controlRollStatusSignature(uint16_t reference_raw,
                                    uint16_t deviation_raw, uint8_t flags) {
  constexpr uint8_t kStatusFlagMask =
      ControlRollTelemetryV2::reference_valid |
      ControlRollTelemetryV2::control_active |
      ControlRollTelemetryV2::reference_out_of_range |
      ControlRollTelemetryV2::deviation_out_of_range;
  return static_cast<uint32_t>(flags & kStatusFlagMask) |
         static_cast<uint32_t>(controlRollQualityCode(reference_raw)) << 8U |
         static_cast<uint32_t>(controlRollQualityCode(deviation_raw)) << 13U;
}

CanFrame encode(CanId id, const EmergencyStop &message) {
  auto result = frame(id, 1);
  result.data[0] = message.transaction_id;
  return result;
}

CanFrame encode(const RecoveryControl &message) {
  auto result = frame(CanId::recovery_control, 8);
  result.data[0] = static_cast<uint8_t>(message.opcode) |
                   static_cast<uint8_t>(static_cast<uint8_t>(message.source)
                                        << 4U);
  result.data[1] = message.transfer_id;
  putU24(result.data, 2, message.offset);
  putU24(result.data, 5, message.length);
  return result;
}

CanFrame encode(const RecoveryModeCommand &message) {
  auto result = frame(CanId::recovery_mode_command, 3);
  result.data[0] = message.sequence;
  result.data[1] = static_cast<uint8_t>(message.mode);
  result.data[2] = static_cast<uint8_t>(message.reason);
  return result;
}

CanFrame encode(const GenericCommandRequest &message) {
  auto result = frame(CanId::generic_command_request, 8);
  result.data[0] = message.transaction_id;
  result.data[1] = message.command;
  std::copy(message.arguments.begin(), message.arguments.end(),
            result.data.begin() + 2);
  return result;
}

CanFrame encode(const CommandResult &message) {
  auto result = frame(CanId::command_result, 8);
  result.data[0] = message.transaction_id;
  result.data[1] = message.command;
  result.data[2] = static_cast<uint8_t>(message.phase);
  result.data[3] = static_cast<uint8_t>(message.reason);
  putU32(result.data, 4, message.detail);
  return result;
}

CanFrame encode(const TimeRequest &message) {
  auto result = frame(CanId::time_request, 1);
  result.data[0] = message.request_id;
  return result;
}

CanFrame encode(const TimeResponse &message) {
  auto result = frame(CanId::time_response, 8);
  result.data[0] = message.request_id;
  result.data[1] = static_cast<uint8_t>(message.source);
  putU32(result.data, 2, message.unix_seconds);
  putU16(result.data, 6, message.milliseconds);
  return result;
}

CanFrame encode(const MissionEvent &message) {
  auto result = frame(CanId::mission_event, 8);
  result.data[0] = message.sequence;
  putU16(result.data, 1, message.flags);
  result.data[3] = static_cast<uint8_t>(message.state);
  putU16(result.data, 4, message.elapsed_raw);
  putU16(result.data, 6, message.detail);
  return result;
}

CanFrame encode(const KinematicsTelemetry &message) {
  auto result = frame(CanId::kinematics_telemetry, 8);
  result.data[0] = message.sequence;
  putU16(result.data, 1, message.roll_raw);
  putU16(result.data, 3, message.roll_rate_raw);
  result.data[5] = message.fin_angle_raw;
  putU16(result.data, 6, message.fin_rate_raw);
  return result;
}

CanFrame encode(const ControlTelemetry &message) {
  auto result = frame(CanId::control_telemetry, 4);
  result.data[0] = message.sequence;
  putU16(result.data, 1, static_cast<uint16_t>(message.requested_torque_raw &
                                               0x0FFFU));
  result.data[3] = message.flight_elapsed_raw;
  return result;
}

CanFrame encode(const MissionStatusTelemetry &message) {
  auto result = frame(CanId::mission_status_telemetry, 8);
  result.data[0] = message.sequence;
  result.data[1] = static_cast<uint8_t>(message.state);
  putU16(result.data, 2, message.flight_status);
  result.data[4] = message.config_flags;
  result.data[5] = static_cast<uint8_t>(message.fin_mode);
  result.data[6] = static_cast<uint8_t>(message.para_mode);
  result.data[7] = message.parachute_angle_raw;
  return result;
}

CanFrame encode(const PowerTimeTelemetry &message) {
  auto result = frame(CanId::power_time_telemetry, 8);
  result.data[0] = message.sequence;
  result.data[1] = message.logic_voltage_raw;
  result.data[2] = message.motor_voltage_raw;
  putU16(result.data, 3, message.descent_elapsed_raw);
  putU16(result.data, 5, message.recovery_elapsed_raw);
  result.data[7] = static_cast<uint8_t>(message.persistence_flags & 0x87U);
  return result;
}

CanFrame encode(const DescentCoreTelemetry &message) {
  auto result = frame(CanId::descent_core_telemetry, 4);
  result.data[0] = message.sequence;
  putU16(result.data, 1,
         static_cast<uint16_t>(message.descent_status & 0x001FU));
  result.data[3] = message.parachute_angle_raw;
  return result;
}

CanFrame encode(const RecoveryStatusMessage &message) {
  auto result = frame(CanId::recovery_status, 8);
  result.data[0] = static_cast<uint8_t>(message.opcode);
  result.data[1] = message.transfer_id;
  result.data[2] = static_cast<uint8_t>(message.status);
  result.data[3] = static_cast<uint8_t>(message.source);
  putU32(result.data, 4, message.total_size);
  return result;
}

CanFrame encode(const RecoveryLogData &message) {
  auto result = frame(CanId::recovery_log_data, 8);
  result.data[0] = message.transfer_id;
  result.data[1] = message.sequence;
  std::copy(message.data.begin(), message.data.end(), result.data.begin() + 2);
  return result;
}

CanFrame encode(const AttitudeTiltTelemetry &message) {
  auto result = frame(CanId::attitude_tilt_telemetry, 3);
  result.data[0] = message.sequence;
  const uint16_t packed =
      static_cast<uint16_t>(message.magnitude_raw & 0x7FU) |
      static_cast<uint16_t>((message.direction_raw & 0x01FFU) << 7U);
  putU16(result.data, 1, packed);
  return result;
}

CanFrame encode(const LpsTelemetry &message) {
  auto result = frame(CanId::lps_telemetry, 4);
  result.data[0] = message.sequence;
  putU16(result.data, 1,
         static_cast<uint16_t>(message.pressure_raw & 0x07FFU));
  result.data[3] = message.temperature_raw;
  return result;
}

CanFrame encode(const AirspeedTelemetry &message) {
  auto result = frame(CanId::airspeed_telemetry, 2);
  result.data[0] = message.sequence;
  result.data[1] = message.airspeed_raw;
  return result;
}

CanFrame encode(const ControlRollTelemetryV2 &message) {
  auto result = frame(CanId::control_roll_telemetry_v2, 8);
  result.data[0] = message.sequence;
  result.data[1] = ControlRollTelemetryV2::schema_version;
  putU16(result.data, 2, message.control_roll_reference_unwrapped_raw);
  putU16(result.data, 4, message.roll_deviation_unwrapped_raw);
  result.data[6] = static_cast<uint8_t>(message.flags & 0x1FU);
  result.data[7] = message.reference_capture_event_sequence;
  return result;
}

CodecError decode(const CanFrame &input, CanId id, EmergencyStop &message) {
  if (id != CanId::actuator_emergency_stop &&
      id != CanId::liftoff_detection_emergency_stop)
    return CodecError::wrong_identifier;
  const auto error = validate(input, id, 1);
  if (error != CodecError::none)
    return error;
  message.transaction_id = input.data[0];
  return CodecError::none;
}

CodecError decode(const CanFrame &input, RecoveryControl &message) {
  const auto error = validate(input, CanId::recovery_control, 8);
  if (error != CodecError::none)
    return error;
  if ((input.data[0] & 0xE0U) != 0)
    return CodecError::reserved_bits;
  const uint8_t opcode_raw = input.data[0] & 0x0FU;
  const auto opcode = static_cast<RecoveryOpcode>(opcode_raw);
  const auto source =
      static_cast<RecoverySource>((input.data[0] >> 4U) & 0x01U);
  if (opcode_raw < static_cast<uint8_t>(RecoveryOpcode::wake) ||
      opcode_raw > static_cast<uint8_t>(RecoveryOpcode::stop_log_dump))
    return CodecError::invalid_enum;
  message = {opcode, source, input.data[1], getU24(input.data, 2),
             getU24(input.data, 5)};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, RecoveryModeCommand &message) {
  const auto error = validate(input, CanId::recovery_mode_command, 3);
  if (error != CodecError::none)
    return error;
  const auto mode = static_cast<RecoveryMode>(input.data[1]);
  const auto reason = static_cast<RecoveryModeReason>(input.data[2]);
  if (mode != RecoveryMode::enter_recovery_beacon ||
      static_cast<uint8_t>(reason) >
          static_cast<uint8_t>(RecoveryModeReason::reset_recovery))
    return CodecError::invalid_enum;
  message = {input.data[0], mode, reason};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, GenericCommandRequest &message) {
  const auto error = validate(input, CanId::generic_command_request, 8);
  if (error != CodecError::none)
    return error;
  message.transaction_id = input.data[0];
  message.command = input.data[1];
  std::copy_n(input.data.begin() + 2, message.arguments.size(),
              message.arguments.begin());
  return CodecError::none;
}

CodecError decode(const CanFrame &input, CommandResult &message) {
  const auto error = validate(input, CanId::command_result, 8);
  if (error != CodecError::none)
    return error;
  const auto phase = static_cast<CommandPhase>(input.data[2]);
  const auto reason = static_cast<CommandReason>(input.data[3]);
  if (static_cast<uint8_t>(phase) > 3 || static_cast<uint8_t>(reason) > 14)
    return CodecError::invalid_enum;
  message = {input.data[0], input.data[1], phase, reason,
             getU32(input.data, 4)};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, TimeRequest &message) {
  const auto error = validate(input, CanId::time_request, 1);
  if (error == CodecError::none)
    message.request_id = input.data[0];
  return error;
}

CodecError decode(const CanFrame &input, TimeResponse &message) {
  const auto error = validate(input, CanId::time_response, 8);
  if (error != CodecError::none)
    return error;
  const auto source = static_cast<TimeSource>(input.data[1]);
  if (static_cast<uint8_t>(source) > 2 || getU16(input.data, 6) > 999)
    return CodecError::invalid_enum;
  message = {input.data[0], source, getU32(input.data, 2),
             getU16(input.data, 6)};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, MissionEvent &message) {
  const auto error = validate(input, CanId::mission_event, 8);
  if (error != CodecError::none)
    return error;
  const auto state = static_cast<MissionState>(input.data[3]);
  if (!validMissionState(state))
    return CodecError::invalid_enum;
  message = {input.data[0], getU16(input.data, 1), state,
             getU16(input.data, 4), getU16(input.data, 6)};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, KinematicsTelemetry &message) {
  const auto error = validate(input, CanId::kinematics_telemetry, 8);
  if (error == CodecError::none)
    message = {input.data[0], getU16(input.data, 1), getU16(input.data, 3),
               input.data[5], getU16(input.data, 6)};
  return error;
}

CodecError decode(const CanFrame &input, ControlTelemetry &message) {
  const auto error = validate(input, CanId::control_telemetry, 4);
  if (error != CodecError::none)
    return error;
  const uint16_t raw = getU16(input.data, 1);
  if ((raw & 0xF000U) != 0)
    return CodecError::reserved_bits;
  message = {input.data[0], raw, input.data[3]};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, MissionStatusTelemetry &message) {
  const auto error = validate(input, CanId::mission_status_telemetry, 8);
  if (error != CodecError::none)
    return error;
  const auto state = static_cast<MissionState>(input.data[1]);
  const auto fin = static_cast<FinMode>(input.data[5]);
  const auto para = static_cast<ParaMode>(input.data[6]);
  if (!validMissionState(state) || !validFinMode(fin) || !validParaMode(para))
    return CodecError::invalid_enum;
  message = {input.data[0], state, getU16(input.data, 2), input.data[4],
             fin, para, input.data[7]};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, PowerTimeTelemetry &message) {
  const auto error = validate(input, CanId::power_time_telemetry, 8);
  if (error != CodecError::none)
    return error;
  if ((input.data[7] & 0x78U) != 0)
    return CodecError::reserved_bits;
  message = {input.data[0], input.data[1], input.data[2],
             getU16(input.data, 3), getU16(input.data, 5), input.data[7]};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, DescentCoreTelemetry &message) {
  const auto error = validate(input, CanId::descent_core_telemetry, 4);
  if (error != CodecError::none)
    return error;
  const uint16_t status = getU16(input.data, 1);
  if ((status & 0xFFE0U) != 0)
    return CodecError::reserved_bits;
  message = {input.data[0], status, input.data[3]};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, RecoveryStatusMessage &message) {
  const auto error = validate(input, CanId::recovery_status, 8);
  if (error != CodecError::none)
    return error;
  const uint8_t opcode_raw = input.data[0];
  const auto opcode = static_cast<RecoveryOpcode>(opcode_raw);
  const auto status = static_cast<RecoveryStatusCode>(input.data[2]);
  const auto source = static_cast<RecoverySource>(input.data[3]);
  if (opcode_raw < static_cast<uint8_t>(RecoveryOpcode::wake) ||
      opcode_raw > static_cast<uint8_t>(RecoveryOpcode::stop_log_dump) ||
      static_cast<uint8_t>(status) > 9 ||
      static_cast<uint8_t>(source) > 1)
    return CodecError::invalid_enum;
  message = {opcode, input.data[1], status, source, getU32(input.data, 4)};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, RecoveryLogData &message) {
  const auto error = validate(input, CanId::recovery_log_data, 8);
  if (error != CodecError::none)
    return error;
  message.transfer_id = input.data[0];
  message.sequence = input.data[1];
  std::copy_n(input.data.begin() + 2, message.data.size(),
              message.data.begin());
  return CodecError::none;
}

CodecError decode(const CanFrame &input, AttitudeTiltTelemetry &message) {
  const auto error = validate(input, CanId::attitude_tilt_telemetry, 3);
  if (error != CodecError::none)
    return error;
  const uint16_t packed = getU16(input.data, 1);
  message = {input.data[0], static_cast<uint8_t>(packed & 0x7FU),
             static_cast<uint16_t>(packed >> 7U)};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, LpsTelemetry &message) {
  const auto error = validate(input, CanId::lps_telemetry, 4);
  if (error != CodecError::none)
    return error;
  const uint16_t pressure = getU16(input.data, 1);
  if ((pressure & 0xF800U) != 0)
    return CodecError::reserved_bits;
  message = {input.data[0], pressure, input.data[3]};
  return CodecError::none;
}

CodecError decode(const CanFrame &input, AirspeedTelemetry &message) {
  const auto error = validate(input, CanId::airspeed_telemetry, 2);
  if (error == CodecError::none)
    message = {input.data[0], input.data[1]};
  return error;
}

CodecError decode(const CanFrame &input, ControlRollTelemetryV2 &message) {
  const auto error = validate(input, CanId::control_roll_telemetry_v2, 8);
  if (error != CodecError::none)
    return error;
  if (input.data[1] != ControlRollTelemetryV2::schema_version)
    return CodecError::invalid_enum;
  if ((input.data[6] & 0xE0U) != 0)
    return CodecError::reserved_bits;
  const uint16_t reference_raw = getU16(input.data, 2);
  const uint16_t deviation_raw = getU16(input.data, 4);
  const uint16_t out_of_range =
      static_cast<uint16_t>(quantization::RollError::out_of_range);
  const bool reference_out_of_range =
      (input.data[6] & ControlRollTelemetryV2::reference_out_of_range) != 0;
  const bool deviation_out_of_range =
      (input.data[6] & ControlRollTelemetryV2::deviation_out_of_range) != 0;
  if ((reference_raw == out_of_range) != reference_out_of_range ||
      (deviation_raw == out_of_range) != deviation_out_of_range)
    return CodecError::invalid_enum;
  message = {input.data[0], reference_raw, deviation_raw, input.data[6],
             input.data[7]};
  return CodecError::none;
}

uint16_t canPeriodMilliseconds(CanId id) {
  switch (id) {
  case CanId::kinematics_telemetry:
  case CanId::control_telemetry:
  case CanId::descent_core_telemetry:
  case CanId::airspeed_telemetry:
    return 10;
  case CanId::control_roll_telemetry_v2:
    return 100;
  case CanId::lps_telemetry:
    return 40;
  case CanId::mission_status_telemetry:
  case CanId::power_time_telemetry:
  case CanId::attitude_tilt_telemetry:
    return 100;
  default:
    return 0;
  }
}

uint8_t TelemetrySequences::next(CanId id) {
  auto &counter = counters_[sequenceIndex(id)];
  const uint8_t result = counter;
  ++counter;
  return result;
}

} // 名前空間 protocol
