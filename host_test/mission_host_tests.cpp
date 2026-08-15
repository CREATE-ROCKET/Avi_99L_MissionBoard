#include "actuators/safety_core.hpp"
#include "control/control_pipeline.hpp"
#include "mission/command_executor.hpp"
#include "mission/mission_state.hpp"
#include "mission/preflight_readiness.hpp"
#include "mission/recovery.hpp"
#include "protocol/can_protocol.hpp"
#include "protocol/quantization.hpp"
#include "runtime/task_architecture.hpp"
#include "runtime/emergency_latch.hpp"
#include "sensors/air_data_flight_logic.hpp"
#include "sensors/airspeed_estimator.hpp"
#include "sensors/attitude_estimator.hpp"
#include "sensors/flight_detectors.hpp"
#include "sensors/sensor_health.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using protocol::CanFrame;
using protocol::CanId;

std::vector<uint8_t> parseHex(const std::string &text) {
  if (text.size() % 2 != 0)
    throw std::runtime_error("golden vectorのhex桁数が奇数です");
  std::vector<uint8_t> bytes;
  bytes.reserve(text.size() / 2);
  for (std::size_t i = 0; i < text.size(); i += 2)
    bytes.push_back(static_cast<uint8_t>(std::stoul(text.substr(i, 2), nullptr,
                                                    16)));
  return bytes;
}

std::map<std::string, std::vector<uint8_t>> readGolden() {
  std::ifstream input{GOLDEN_VECTOR_PATH};
  if (!input)
    throw std::runtime_error("golden vectorを開けません");
  std::map<std::string, std::vector<uint8_t>> result;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    const auto separator = line.find('=');
    if (separator == std::string::npos)
      throw std::runtime_error("golden vector形式が不正です");
    result.emplace(line.substr(0, separator),
                   parseHex(line.substr(separator + 1)));
  }
  return result;
}

void expectFrame(const std::map<std::string, std::vector<uint8_t>> &golden,
                 const std::string &name, const CanFrame &actual) {
  const auto &expected = golden.at(name);
  assert(actual.data_length == expected.size());
  assert(!actual.extended && !actual.remote);
  for (std::size_t i = 0; i < expected.size(); ++i)
    assert(actual.data[i] == expected[i]);
}

void expectScalar(const std::map<std::string, std::vector<uint8_t>> &golden,
                  const std::string &name, uint16_t raw) {
  const auto &expected = golden.at(name);
  assert(expected.size() == 1 || expected.size() == 2);
  assert(expected[0] == static_cast<uint8_t>(raw));
  if (expected.size() == 2)
    assert(expected[1] == static_cast<uint8_t>(raw >> 8U));
}

void testCanGolden(
    const std::map<std::string, std::vector<uint8_t>> &golden) {
  using namespace protocol;
  const uint16_t event_flags =
      eventFlag(MissionEventFlag::icm_data_loss_or_error) |
      eventFlag(MissionEventFlag::state_changed) |
      eventFlag(MissionEventFlag::liftoff_detected) |
      eventFlag(MissionEventFlag::fin_control_disabled_by_ground);
  assert(event_flags == 0x4061);
  expectFrame(golden, "CAN_001",
              encode(CanId::actuator_emergency_stop, EmergencyStop{0x2A}));
  expectFrame(golden, "CAN_002", encode(
      CanId::liftoff_detection_emergency_stop, EmergencyStop{0x2B}));
  expectFrame(golden, "CAN_008",
              encode(RecoveryControl{RecoveryOpcode::start_log_dump,
                                     RecoverySource::mission_sd_latest_flight,
                                     0x34, 0x012345, 0x000456}));
  expectFrame(golden, "CAN_010",
              encode(GenericCommandRequest{0x2A, 0x13,
                                           {0x85, 0xFF, 0, 0, 0, 0}}));
  expectFrame(golden, "CAN_011",
              encode(CommandResult{0x2A, 0x13, CommandPhase::failed,
                                   CommandReason::interrupted_by_emergency,
                                   0x12345678}));
  expectFrame(golden, "CAN_012", encode(TimeRequest{7}));
  expectFrame(golden, "CAN_013",
              encode(TimeResponse{7, TimeSource::ground, 0x12345678, 999}));
  expectFrame(golden, "CAN_020",
              encode(MissionEvent{0xFF, event_flags, MissionState::control, 1234,
                                  0xBEEF}));
  expectFrame(golden, "CAN_100",
              encode(KinematicsTelemetry{0xFF, 0x800D, 0xFFF6, 0xFE,
                                         0x8009}));
  expectFrame(golden, "CAN_101",
              encode(ControlTelemetry{0xFE, 0x0F85, 0x7B}));
  expectFrame(golden, "CAN_102",
              encode(MissionStatusTelemetry{0xFD, MissionState::control,
                                            0xA55A, 0x6D,
                                            FinMode::roll_control,
                                            ParaMode::hold, 0x78}));
  expectFrame(golden, "CAN_103",
              encode(PowerTimeTelemetry{0xFC, 0xA0, 0xDC, 0xFFFA, 0x000C,
                                        0x65}));
  expectFrame(golden, "CAN_104",
              encode(DescentCoreTelemetry{0xFB, 0x1A55, 0xF7}));
  expectFrame(golden, "CAN_105",
              encode(RecoveryStatusMessage{RecoveryOpcode::start_log_dump,
                                           0x34,
                                           RecoveryStatusCode::dumping,
                                           RecoverySource::mission_sd_latest_flight,
                                           100000}));
  expectFrame(golden, "CAN_106",
              encode(RecoveryLogData{0x34, 0xFF,
                                     {0xDE, 0xAD, 0xBE, 0xEF, 0, 0x11}}));
  expectFrame(golden, "CAN_107",
              encode(AttitudeTiltTelemetry{0xFA, 20, 280}));
  expectFrame(golden, "CAN_108", encode(LpsTelemetry{0xF9, 1066, 70}));
  expectFrame(golden, "CAN_109", encode(AirspeedTelemetry{0xF8, 61}));
  expectFrame(golden, "CAN_10A_POS",
              encode(ControlRollTelemetryV2{0xF7, 760, 1440, 0x07, 0x2A}));
  expectFrame(golden, "CAN_10A_NEG",
              encode(ControlRollTelemetryV2{
                  0xF8, 0, static_cast<uint16_t>(-1440), 0x05, 0x2A}));
  expectFrame(golden, "CAN_10A_OUT_OF_RANGE",
              encode(ControlRollTelemetryV2{0xF9, 0x800A, 0x800A, 0x19,
                                            0x2B}));

  ControlTelemetry control{};
  auto invalid = encode(ControlTelemetry{1, 2, 3});
  invalid.data[2] |= 0x10;
  assert(decode(invalid, control) == CodecError::reserved_bits);
  invalid = encode(ControlTelemetry{1, 2, 3});
  invalid.data_length = 8;
  assert(decode(invalid, control) == CodecError::wrong_length);
  invalid = encode(ControlTelemetry{1, 2, 3});
  invalid.extended = true;
  assert(decode(invalid, control) == CodecError::unsupported_frame);

  TelemetrySequences sequences;
  for (unsigned i = 0; i < 256; ++i)
    assert(sequences.next(CanId::kinematics_telemetry) ==
           static_cast<uint8_t>(i));
  assert(sequences.next(CanId::kinematics_telemetry) == 0);
  assert(sequences.next(CanId::control_telemetry) == 0);
  assert(canPeriodMilliseconds(CanId::kinematics_telemetry) == 10);
  assert(canPeriodMilliseconds(CanId::descent_core_telemetry) == 10);
  assert(canPeriodMilliseconds(CanId::lps_telemetry) == 40);
  assert(canPeriodMilliseconds(CanId::mission_status_telemetry) == 100);
  assert(canPeriodMilliseconds(CanId::control_roll_telemetry_v2) == 100);

  ControlRollTelemetryV2 control_roll{};
  const auto control_roll_frame =
      encode(ControlRollTelemetryV2{1, 760, 1440, 0x07, 2});
  assert(decode(control_roll_frame, control_roll) == CodecError::none);
  assert(control_roll.control_roll_reference_unwrapped_raw == 760);
  assert(control_roll.roll_deviation_unwrapped_raw == 1440);
  const uint32_t nominal_control_roll_signature = controlRollStatusSignature(
      control_roll.control_roll_reference_unwrapped_raw,
      control_roll.roll_deviation_unwrapped_raw, control_roll.flags);
  const uint8_t flags_without_capture = static_cast<uint8_t>(
      control_roll.flags &
      ~ControlRollTelemetryV2::reference_captured_since_previous_frame);
  assert(nominal_control_roll_signature ==
         controlRollStatusSignature(
             control_roll.control_roll_reference_unwrapped_raw,
             control_roll.roll_deviation_unwrapped_raw,
             flags_without_capture));
  const auto out_of_range_control_roll =
      CanFrame{static_cast<uint16_t>(CanId::control_roll_telemetry_v2), 8,
               {0xF9, 0x02, 0x0A, 0x80, 0x0A, 0x80, 0x19, 0x2B}};
  assert(decode(out_of_range_control_roll, control_roll) == CodecError::none);
  assert(control_roll.control_roll_reference_unwrapped_raw == 0x800A);
  assert(control_roll.roll_deviation_unwrapped_raw == 0x800A);
  assert(nominal_control_roll_signature !=
         controlRollStatusSignature(
             control_roll.control_roll_reference_unwrapped_raw,
             control_roll.roll_deviation_unwrapped_raw, control_roll.flags));
  auto invalid_control_roll = control_roll_frame;
  invalid_control_roll.data[1] = 1;
  assert(decode(invalid_control_roll, control_roll) ==
         CodecError::invalid_enum);
  invalid_control_roll = control_roll_frame;
  invalid_control_roll.data[6] = 0x80;
  assert(decode(invalid_control_roll, control_roll) ==
         CodecError::reserved_bits);
  invalid_control_roll = control_roll_frame;
  invalid_control_roll.data[6] |=
      ControlRollTelemetryV2::reference_out_of_range;
  assert(decode(invalid_control_roll, control_roll) ==
         CodecError::invalid_enum);
  invalid_control_roll = out_of_range_control_roll;
  invalid_control_roll.data[6] &=
      static_cast<uint8_t>(~ControlRollTelemetryV2::deviation_out_of_range);
  assert(decode(invalid_control_roll, control_roll) ==
         CodecError::invalid_enum);
}

void testQuantization(
    const std::map<std::string, std::vector<uint8_t>> &golden) {
  using namespace protocol::quantization;
  expectScalar(golden, "SCALAR_ROLL_NEG1",
               encodeRoll(-1.0, RollError::unknown));
  expectScalar(golden, "SCALAR_ROLL_MIN",
               encodeRoll(-16376.0, RollError::unknown));
  expectScalar(golden, "SCALAR_ROLL_RESET_INVALIDATED",
               encodeRoll(NAN, RollError::reset_invalidated));
  expectScalar(golden, "SCALAR_ROLL_POS_380",
               encodeRoll(380.0, RollError::unknown));
  expectScalar(golden, "SCALAR_ROLL_POS_720",
               encodeRoll(720.0, RollError::unknown));
  expectScalar(golden, "SCALAR_ROLL_NEG_720",
               encodeRoll(-720.0, RollError::unknown));
  expectScalar(golden, "SCALAR_ROLL_OUT_OF_RANGE",
               encodeRoll(20'000.0, RollError::unknown));
  assert(decodeRoll(0x8000).semantic == Semantic::error);
  assert(decodeRoll(0x8010).semantic == Semantic::numeric);
  assert(encodeRoll(-16376.5, RollError::unknown) ==
         static_cast<uint16_t>(RollError::out_of_range));

  const uint16_t tilt = static_cast<uint16_t>(encodeTiltMagnitude(90, 127)) |
                        static_cast<uint16_t>(encodeTiltDirection(359) << 7U);
  expectScalar(golden, "SCALAR_TILT_MAX", tilt);
  assert(decodeTiltMagnitude(121).semantic == Semantic::error);
  assert(decodeTiltDirection(360).semantic == Semantic::reserved);
  assert(encodeTiltDirection(-1) == 359);

  expectScalar(golden, "SCALAR_FIN_ANGLE_MIN",
               encodeFinAngle(-15, FinAngleError::internal_or_unknown));
  expectScalar(golden, "SCALAR_FIN_ANGLE_ZERO",
               encodeFinAngle(0, FinAngleError::internal_or_unknown));
  expectScalar(golden, "SCALAR_FIN_ANGLE_MAX",
               encodeFinAngle(15, FinAngleError::internal_or_unknown));
  assert(encodeFinAngle(15.01, FinAngleError::out_of_mechanical_range) == 254);
  assert(decodeFinAngle(241).semantic == Semantic::error);
  expectScalar(golden, "SCALAR_FIN_RATE_NEG1",
               encodeFinRate(-1, FinRateError::unavailable));
  assert(decodeFinRate(0x800A).semantic == Semantic::reserved);

  expectScalar(golden, "SCALAR_TORQUE_NEG_0P246",
               encodeRequestedTorque(-0.246, TorqueError::unknown));
  assert(decodeRequestedTorque(0x807).semantic == Semantic::reserved);
  assert(decodeRequestedTorque(0x1800).semantic == Semantic::reserved);

  expectScalar(golden, "SCALAR_LPS_PRESSURE_1013P2",
               encodeLpsPressure(1013.2, LpsPressureError::unknown));
  expectScalar(golden, "SCALAR_LPS_PRESSURE_MAX",
               encodeLpsPressure(1206.2, LpsPressureError::unknown));
  expectScalar(golden, "SCALAR_LPS_PRESSURE_STALE",
               encodeLpsPressure(NAN, LpsPressureError::stale));
  assert(decodeLpsPressure(2039).semantic == Semantic::error);
  expectScalar(golden, "SCALAR_LPS_TEMP_20",
               encodeLpsTemperature(20, LpsTemperatureError::unknown));
  assert(decodeLpsTemperature(201).semantic == Semantic::reserved);

  expectScalar(golden, "SCALAR_AIRSPEED_245",
               encodeAirspeed(245, AirspeedError::internal_invalid));
  expectScalar(golden, "SCALAR_AIRSPEED_NEGATIVE",
               encodeAirspeed(-1, AirspeedError::internal_invalid));
  expectScalar(golden, "SCALAR_AIRSPEED_STALE",
               encodeAirspeed(NAN, AirspeedError::ssc_stale));
  expectScalar(golden, "SCALAR_FLIGHT_ELAPSED_23P9",
               encodeFlightElapsed(23.999, TimeError::unknown));
  expectScalar(golden, "SCALAR_FLIGHT_ELAPSED_STALE",
               encodeFlightElapsed(NAN, TimeError::stale));

  expectScalar(golden, "SCALAR_GNSS_EAST_NEG1",
               encodeGnssOffset(-1, GnssError::unknown));
  expectScalar(golden, "SCALAR_GNSS_NO_FIX",
               encodeGnssOffset(NAN, GnssError::no_fix));
  expectScalar(golden, "SCALAR_GNSS_STALE",
               encodeGnssOffset(NAN, GnssError::stale));
  assert(decodeGnssOffset(0x8009).semantic == Semantic::reserved);
  expectScalar(golden, "SCALAR_GNSS_HEIGHT_100",
               encodeGnssHeight(100, GnssHeightError::unknown));
  expectScalar(golden, "SCALAR_GNSS_HEIGHT_NO_FIX",
               encodeGnssHeight(NAN, GnssHeightError::no_fix));
  expectScalar(golden, "SCALAR_GNSS_HEIGHT_STALE",
               encodeGnssHeight(NAN, GnssHeightError::stale));
  assert(decodeGnssHeight(504).semantic == Semantic::reserved);

  expectScalar(golden, "SCALAR_PARA_ANGLE_360",
               encodeParachuteAngle(360,
                                    ParachuteAngleError::position_out_of_range));
  expectScalar(golden, "SCALAR_PARA_STALE",
               encodeParachuteAngle(NAN, ParachuteAngleError::stale));
  expectScalar(golden, "SCALAR_DESCENT_ELAPSED_MAX",
               encodeDescentElapsed(6551.999, TimeError::unknown));
  expectScalar(golden, "SCALAR_DESCENT_ELAPSED_STALE",
               encodeDescentElapsed(NAN, TimeError::stale));
  expectScalar(golden, "SCALAR_BATTERY_12V",
               encodeBatteryVoltage(12, BatteryError::unavailable));
  expectScalar(golden, "SCALAR_BATTERY_FIRST_RESERVED", 241);
  expectScalar(golden, "SCALAR_BATTERY_STALE",
               encodeBatteryVoltage(NAN, BatteryError::stale));
  assert(decodeBatteryVoltage(241).semantic == Semantic::reserved);
}

mission::ControlAvailability readyControl() {
  mission::ControlAvailability result{};
  result.fin_control_available = true;
  result.fin_zero_hold_valid = true;
  result.attitude_valid = true;
  result.airspeed_above_60 = true;
  result.lps_available = true;
  result.ssc_available = true;
  result.gyro_bias_valid = true;
  result.ssc_zero_valid = true;
  result.roll_estimate_liftoff_relative_unwrapped_rad = 12.75;
  result.roll_estimator_timestamp_us = 8'999'900;
  result.attitude_fresh = true;
  return result;
}

void testMissionStateMachine() {
  using mission::FinDirective;
  using mission::MissionStateMachine;
  using mission::SequenceConfiguration;
  using mission::TransitionResult;
  using protocol::MissionState;

  mission::PreflightReadinessSnapshot ready_snapshot{};
  ready_snapshot.fin_zero_configured = true;
  ready_snapshot.parachute_open_configured = true;
  ready_snapshot.parachute_close_configured = true;
  ready_snapshot.motor_profile_valid = true;
  ready_snapshot.gyro_bias_valid = true;
  ready_snapshot.gravity_reference_valid = true;
  ready_snapshot.ssc_zero_valid = true;
  ready_snapshot.resources_preallocated = true;
  const SequenceConfiguration ready{ready_snapshot, false};
  MissionStateMachine cancelled;
  assert(cancelled.startSequence(0, ready) == TransitionResult::completed);
  const uint32_t first_epoch = cancelled.snapshot().flight_epoch;
  assert(cancelled.snapshot().state == MissionState::liftoff_detection);
  assert(cancelled.snapshot().fin == FinDirective::zero_hold);
  assert(cancelled.cancelSequence() == TransitionResult::completed);
  assert(cancelled.snapshot().state == MissionState::command_receive);
  assert(cancelled.snapshot().fin == FinDirective::brake);

  MissionStateMachine emergency;
  assert(emergency.startSequence(0, ready) == TransitionResult::completed);
  emergency.tick({2'000'000, true, false, readyControl()});
  assert(emergency.snapshot().state == MissionState::engine_burn);
  assert(emergency.snapshot().liftoff_time_us == 1'000'000);
  assert(emergency.liftoffDetectionEmergencyStop() ==
         TransitionResult::completed);
  assert(emergency.snapshot().state == MissionState::liftoff_detection);
  assert(emergency.snapshot().flight_epoch != first_epoch);
  assert(!emergency.snapshot().liftoff_time_valid);

  MissionStateMachine control;
  assert(control.startSequence(0, ready) == TransitionResult::completed);
  control.tick({2'000'000, true, false, readyControl()});
  control.tick({8'999'999, false, false, readyControl()});
  assert(control.snapshot().state == MissionState::engine_burn);
  auto entry_tick = mission::MissionTickInput{
      9'000'000, false, false, readyControl(), 9'000};
  control.tick(entry_tick);
  assert(control.snapshot().state == MissionState::control);
  assert(control.snapshot().control_roll_reference_valid);
  assert(control.snapshot().control_roll_reference_unwrapped_rad == 12.75);
  assert(control.snapshot().control_roll_reference_estimator_timestamp_us ==
         8'999'900);
  assert(control.snapshot().control_roll_reference_capture_tick == 9'000);
  assert(entry_tick.control.roll_estimate_liftoff_relative_unwrapped_rad -
             control.snapshot().control_roll_reference_unwrapped_rad ==
         0.0);
  const uint8_t capture_sequence =
      control.snapshot().control_roll_reference_capture_event_sequence;
  auto changed_roll = readyControl();
  changed_roll.roll_estimate_liftoff_relative_unwrapped_rad +=
      2.0 * 3.14159265358979323846;
  changed_roll.roll_estimator_timestamp_us = 9'000'900;
  control.tick({9'001'000, false, false, changed_roll, 9'001});
  assert(control.snapshot().state == MissionState::control);
  assert(control.snapshot().control_roll_reference_unwrapped_rad == 12.75);
  assert(control.snapshot().control_roll_reference_capture_event_sequence ==
         capture_sequence);
  auto unavailable = readyControl();
  unavailable.ssc_available = false;
  control.tick({9'002'000, false, false, unavailable});
  assert(control.snapshot().state == MissionState::engine_burn);
  assert(control.snapshot().control_reentry_inhibited);
  assert(control.snapshot().fin == FinDirective::zero_hold);
  auto fin_unavailable = unavailable;
  fin_unavailable.fin_control_available = false;
  control.tick({9'003'000, false, false, fin_unavailable});
  assert(control.snapshot().fin == FinDirective::brake);
  control.tick({10'000'000, false, false, readyControl()});
  assert(control.snapshot().state == MissionState::engine_burn);
  assert(control.snapshot().control_roll_reference_capture_event_sequence ==
         capture_sequence);
  assert(control.liftoffDetectionEmergencyStop() ==
         TransitionResult::completed);
  assert(!control.snapshot().control_roll_reference_valid);

  MissionStateMachine gate_failure;
  assert(gate_failure.startSequence(0, ready) == TransitionResult::completed);
  gate_failure.tick({2'000'000, true, false, readyControl()});
  gate_failure.tick({9'000'000, false, false, unavailable});
  assert(gate_failure.snapshot().control_reentry_inhibited);
  gate_failure.tick({9'100'000, false, false, readyControl()});
  assert(gate_failure.snapshot().state == MissionState::engine_burn);
  assert(!gate_failure.snapshot().control_roll_reference_valid);

  for (int loss_case = 0; loss_case < 2; ++loss_case) {
    MissionStateMachine loss;
    assert(loss.startSequence(0, ready) == TransitionResult::completed);
    loss.tick({2'000'000, true, false, readyControl()});
    loss.tick({9'000'000, false, false, readyControl(), 9'000});
    auto lost_input = readyControl();
    if (loss_case == 0)
      lost_input.lps_available = false;
    else
      lost_input.attitude_fresh = false;
    loss.tick({9'001'000, false, false, lost_input, 9'001});
    assert(loss.snapshot().state == MissionState::engine_burn);
    assert(loss.snapshot().control_reentry_inhibited);
    loss.tick({9'100'000, false, false, readyControl(), 9'100});
    assert(loss.snapshot().state == MissionState::engine_burn);
  }

  MissionStateMachine stale_reference;
  assert(stale_reference.startSequence(0, ready) ==
         TransitionResult::completed);
  stale_reference.tick({2'000'000, true, false, readyControl()});
  auto stale_control = readyControl();
  stale_control.roll_estimator_timestamp_us = 8'996'000;
  stale_reference.tick({9'000'000, false, false, stale_control, 9'000});
  assert(stale_reference.snapshot().state == MissionState::engine_burn);
  assert(stale_reference.snapshot().control_reentry_inhibited);
  assert(!stale_reference.snapshot().control_roll_reference_valid);

  MissionStateMachine future_reference;
  assert(future_reference.startSequence(0, ready) ==
         TransitionResult::completed);
  future_reference.tick({2'000'000, true, false, readyControl()});
  auto future_control = readyControl();
  future_control.roll_estimator_timestamp_us = 9'000'001;
  future_reference.tick({9'000'000, false, false, future_control, 9'000});
  assert(future_reference.snapshot().state == MissionState::engine_burn);
  assert(!future_reference.snapshot().control_roll_reference_valid);

  MissionStateMachine disabled_after_capture;
  assert(disabled_after_capture.startSequence(0, ready) ==
         TransitionResult::completed);
  disabled_after_capture.tick({2'000'000, true, false, readyControl()});
  disabled_after_capture.tick(
      {9'000'000, false, false, readyControl(), 9'000});
  assert(disabled_after_capture.snapshot().control_roll_reference_valid);
  assert(disabled_after_capture.disableFinControl() ==
         TransitionResult::completed);
  assert(!disabled_after_capture.snapshot().control_roll_reference_valid);

  MissionStateMachine disabled;
  assert(disabled.startSequence(0, ready) == TransitionResult::completed);
  assert(disabled.disableFinControl() == TransitionResult::completed);
  assert(disabled.snapshot().state == MissionState::liftoff_detection);
  assert(disabled.snapshot().fin == FinDirective::brake);
  disabled.tick({2'000'000, true, false, readyControl()});
  disabled.tick({9'000'000, false, false, readyControl()});
  assert(disabled.snapshot().state == MissionState::engine_burn);

  MissionStateMachine deadlines;
  assert(deadlines.startSequence(0, ready) == TransitionResult::completed);
  deadlines.tick({2'000'000, true, false, readyControl()});
  const uint32_t deadline_epoch = deadlines.snapshot().flight_epoch;
  deadlines.tick({17'999'999, false, false, readyControl()},
                 {deadline_epoch - 1, true, true});
  assert(deadlines.snapshot().state != MissionState::descent);
  assert(!deadlines.snapshot().deployment_power_cutoff_latched);
  deadlines.tick({18'000'000, false, false, readyControl()});
  assert(deadlines.snapshot().state == MissionState::descent);
  assert(deadlines.snapshot().deployment_started);
  deadlines.tick({26'000'000, false, false, readyControl()});
  assert(deadlines.snapshot().deployment_power_cutoff_latched);
  assert(deadlines.snapshot().fin == FinDirective::brake);

  MissionStateMachine pressure;
  assert(pressure.startSequence(0, ready) == TransitionResult::completed);
  pressure.tick({2'000'000, true, false, readyControl()});
  pressure.tick({10'999'999, false, true, readyControl()});
  assert(pressure.snapshot().state != MissionState::descent);
  pressure.tick({11'000'000, false, true, readyControl()});
  assert(pressure.snapshot().state == MissionState::descent);

  MissionStateMachine reset;
  assert(reset.restoreAfterReset(
             100'000'000,
             {true, MissionState::control, 7, true, 20'000'000, false,
              false}) == TransitionResult::completed);
  assert(reset.snapshot().state == MissionState::engine_burn);
  assert(reset.snapshot().reset_invalidated);
  assert(!reset.snapshot().control_roll_reference_valid);
  assert(reset.snapshot().fin == FinDirective::brake);
  reset.tick({100'000'000, false, false, readyControl()});
  assert(reset.snapshot().state == MissionState::descent);
  reset.tick({105'000'000, false, false, readyControl()});
  assert(reset.snapshot().deployment_power_cutoff_latched);
  assert(reset.snapshot().elapsed_us == 25'000'000);

  MissionStateMachine reset_near_cutoff;
  assert(reset_near_cutoff.restoreAfterReset(
             5'000'000,
             {true, MissionState::descent, 8, true, 24'999'000, true,
              false}) == TransitionResult::completed);
  reset_near_cutoff.tick({5'001'000, false, false, readyControl()});
  assert(reset_near_cutoff.snapshot().deployment_power_cutoff_latched);
}

protocol::GenericCommandRequest command(uint8_t transaction, uint8_t code) {
  return {transaction, code, {0, 0, 0, 0, 0, 0}};
}

void testCommandExecutor() {
  using mission::CommandCode;
  using mission::CommandContext;
  using mission::CommandExecutor;
  using protocol::CommandPhase;
  using protocol::CommandReason;
  using protocol::MissionState;

  CommandContext context{};
  context.sequence_configured = true;
  context.resources_preallocated = true;
  context.fin_safe_commands_supported = true;
  CommandExecutor executor;
  auto decision = executor.begin(
      command(0, static_cast<uint8_t>(CommandCode::start_sequence)), context);
  assert(!decision.execute);
  assert(decision.result.reason == CommandReason::invalid_argument);

  auto invalid_args =
      command(1, static_cast<uint8_t>(CommandCode::start_sequence));
  invalid_args.arguments[5] = 1;
  decision = executor.begin(invalid_args, context);
  assert(decision.result.reason == CommandReason::invalid_argument);
  decision = executor.begin(invalid_args, context);
  assert(decision.replay && !decision.execute);

  CommandContext wrong_state = context;
  wrong_state.state = MissionState::engine_burn;
  decision = executor.begin(
      command(2, static_cast<uint8_t>(CommandCode::fin_free)), wrong_state);
  assert(decision.result.reason == CommandReason::invalid_state);

  auto fin = command(3, static_cast<uint8_t>(CommandCode::fin_move_relative));
  fin.arguments[0] = 10;
  decision = executor.begin(fin, context);
  assert(decision.execute && decision.result.phase == CommandPhase::accepted);
  decision = executor.begin(fin, context);
  assert(decision.replay && !decision.execute &&
         decision.result.phase == CommandPhase::accepted);
  auto conflicting = fin;
  conflicting.arguments[0] = 11;
  decision = executor.begin(conflicting, context);
  assert(decision.result.reason == CommandReason::protocol_error);
  decision = executor.begin(
      command(4, static_cast<uint8_t>(CommandCode::fin_free)), context);
  assert(decision.result.reason == CommandReason::busy);
  const auto completed =
      executor.finish(3, CommandPhase::completed, CommandReason::none, 7);
  assert(completed.phase == CommandPhase::completed && completed.detail == 7);
  decision = executor.begin(fin, context);
  assert(decision.replay && decision.result.phase == CommandPhase::completed);

  auto para = command(5, static_cast<uint8_t>(CommandCode::para_open));
  assert(executor.begin(para, context).execute);
  auto fin_two = command(6, static_cast<uint8_t>(CommandCode::fin_free));
  assert(executor.begin(fin_two, context).execute);
  const auto emergency =
      executor.actuatorEmergency(8, MissionState::command_receive);
  assert(emergency.execute && emergency.interrupted_count == 2);
  for (std::size_t index = 0; index < emergency.interrupted_count; ++index)
    assert(emergency.interrupted[index].reason ==
           CommandReason::interrupted_by_emergency);
  assert(!executor.actuatorEmergency(0, MissionState::command_receive)
              .execute);
  assert(executor.actuatorEmergency(0, MissionState::command_receive)
             .result.reason == CommandReason::invalid_argument);
  assert(executor.liftoffEmergencyResult(0, true).reason ==
         CommandReason::invalid_argument);
  assert(executor.liftoffEmergencyResult(9, false).reason ==
         CommandReason::invalid_state);

  auto para_limit =
      command(10, static_cast<uint8_t>(CommandCode::para_move_relative));
  para_limit.arguments[0] = 0x08;
  para_limit.arguments[1] = 0x07;
  assert(executor.begin(para_limit, context).result.reason ==
         CommandReason::invalid_argument);

  CommandExecutor parachute_arguments;
  auto set_open =
      command(20, static_cast<uint8_t>(CommandCode::set_para_open));
  assert(parachute_arguments.begin(set_open, context).execute);
  (void)parachute_arguments.finish(20, CommandPhase::completed);
  set_open = command(21, static_cast<uint8_t>(CommandCode::set_para_open));
  set_open.arguments[0] = 1;
  assert(parachute_arguments.begin(set_open, context).result.reason ==
         CommandReason::invalid_argument);
  auto negative_relative =
      command(22, static_cast<uint8_t>(CommandCode::para_move_relative));
  negative_relative.arguments[0] = 0xFF;
  negative_relative.arguments[1] = 0xFF;
  assert(parachute_arguments.begin(negative_relative, context).execute);

  CommandExecutor start_busy;
  assert(start_busy
             .begin(command(23,
                            static_cast<uint8_t>(CommandCode::para_hold)),
                    context)
             .execute);
  CommandContext not_configured = context;
  not_configured.sequence_configured = false;
  assert(start_busy
             .begin(command(24,
                            static_cast<uint8_t>(CommandCode::start_sequence)),
                    not_configured)
             .result.reason == CommandReason::busy);

  CommandExecutor parachute_busy;
  assert(parachute_busy
             .begin(command(25,
                            static_cast<uint8_t>(CommandCode::start_sequence)),
                    context)
             .execute);
  assert(parachute_busy
             .begin(command(26,
                            static_cast<uint8_t>(CommandCode::para_free)),
                    context)
             .result.reason == CommandReason::busy);
  assert(parachute_busy
             .begin(command(27,
                            static_cast<uint8_t>(CommandCode::set_para_close)),
                    wrong_state)
             .result.reason == CommandReason::invalid_state);

  CommandContext calibration_context = context;
  calibration_context.calibration_supported = true;
  const auto calibration = executor.begin(
      command(11, static_cast<uint8_t>(
                      CommandCode::run_preflight_calibration)),
      calibration_context);
  assert(calibration.execute &&
         calibration.result.phase == CommandPhase::accepted);
  (void)executor.finish(11, CommandPhase::completed);

  CommandExecutor cache;
  for (uint8_t id = 1; id <= CommandExecutor::kResultCacheSize; ++id) {
    const auto result = cache.begin(command(id, 0x7F), context);
    assert(result.result.reason == CommandReason::not_supported);
  }
  assert(cache.cachedCount() == CommandExecutor::kResultCacheSize);
  assert(cache.begin(command(1, 0x7F), context).replay);
  assert(cache.begin(command(17, 0x7F), context).result.reason ==
         CommandReason::not_supported);
  assert(cache.cachedCount() == CommandExecutor::kResultCacheSize);
}

void testSensors() {
  using sensors::AvailabilityReason;
  sensors::FreshSample<double> sample;
  assert(sample.availability(0, 20'000) ==
         AvailabilityReason::not_initialized);
  sample.setInitialized(true);
  sample.setHealthy(true);
  assert(sample.availability(0, 20'000) ==
         AvailabilityReason::no_valid_sample);
  sample.publish(1.0, 10'000);
  assert(sample.availability(30'000, 20'000) ==
         AvailabilityReason::available);
  assert(sample.availability(30'001, 20'000) ==
         AvailabilityReason::stale);
  sample.publish(1.0, 40'000);
  assert(sample.availability(39'999, 20'000) ==
         AvailabilityReason::stale);
  sample.setPowered(false);
  assert(sample.availability(20'000, 20'000) ==
         AvailabilityReason::powered_off);

  sensors::AirspeedGate gate;
  for (int index = 0; index < 49; ++index)
    assert(!gate.update(true, 61.0));
  assert(gate.update(true, 61.0));
  for (int index = 0; index < 19; ++index)
    assert(gate.update(true, 60.0));
  assert(!gate.update(true, 60.0));
  assert(!gate.update(false, 100.0));
  for (int index = 0; index < 49; ++index)
    assert(!gate.update(true, 61.0));
  assert(!gate.update(true, NAN));
  assert(!gate.aboveThreshold());

  sensors::ImuLiftoffDetector imu_liftoff;
  for (int index = 0; index < 68; ++index)
    assert(!imu_liftoff.update(2.1, 0.0, 0.0, true));
  assert(imu_liftoff.update(2.1, 0.0, 0.0, true));
  assert(!imu_liftoff.update(2.1, 0.0, 0.0, false));

  sensors::LpsLiftoffDetector lps_liftoff;
  for (int index = 0; index < 9; ++index)
    assert(!lps_liftoff.update(1000.0 - index * 0.3, true));
  assert(lps_liftoff.update(997.3, true));
  assert(!lps_liftoff.update(NAN, false));

  sensors::PressureApexDetector apex;
  for (int index = 0; index < 29; ++index)
    assert(!apex.update(900.0 + index, true, 10'000'000));
  assert(apex.update(929.0, true, 10'000'000));
  assert(!apex.update(NAN, false, 11'000'000));

  sensors::DifferentialPressureConditioner conditioner{4, 2, 1.0};
  assert(!conditioner.updateZero(2.0, false));
  assert(!conditioner.updateZero(2.0, true));
  assert(!conditioner.updateZero(4.0, true));
  assert(!conditioner.updateZero(2.0, true));
  assert(conditioner.updateZero(4.0, true));
  assert(std::abs(conditioner.zeroOffsetPa() - 3.0) < 1.0e-12);
  auto conditioned = conditioner.update(5.0);
  assert(conditioned.valid && conditioned.zero_valid &&
         std::abs(conditioned.pressure_pa - 2.0) < 1.0e-12);
  conditioned = conditioner.update(7.0);
  assert(conditioned.valid &&
         std::abs(conditioned.pressure_pa - 3.0) < 1.0e-12);
  const auto pressure_domain_filtered_speed =
      sensors::computeSaintVenantAirspeed(101325.0, conditioned.pressure_pa,
                                          20.0, 0.92);
  const auto speed_at_2_pa =
      sensors::computeSaintVenantAirspeed(101325.0, 2.0, 20.0, 0.92);
  const auto speed_at_4_pa =
      sensors::computeSaintVenantAirspeed(101325.0, 4.0, 20.0, 0.92);
  assert(pressure_domain_filtered_speed.valid && speed_at_2_pa.valid &&
         speed_at_4_pa.valid);
  assert(std::abs(pressure_domain_filtered_speed.airspeed_mps -
                  (speed_at_2_pa.airspeed_mps + speed_at_4_pa.airspeed_mps) /
                      2.0) > 1.0e-6);
  conditioned = conditioner.update(1.5);
  assert(conditioned.negative_beyond_tolerance && !conditioned.valid);

  const auto zero_speed = sensors::computeSaintVenantAirspeed(
      101325.0, 0.0, 20.0, 0.92);
  assert(zero_speed.valid && zero_speed.airspeed_mps == 0.0);
  const auto positive_speed = sensors::computeSaintVenantAirspeed(
      101325.0, 100.0, 20.0, 0.92);
  assert(positive_speed.valid && positive_speed.airspeed_mps > 10.0 &&
         positive_speed.airspeed_mps < 20.0);
  constexpr double gamma = 1.4;
  constexpr double gas_constant = 287.05;
  const double temperature_kelvin = 20.0 + 273.15;
  const double expected_pressure_corrected_speed =
      std::sqrt(2.0 * gamma / (gamma - 1.0) * gas_constant *
                temperature_kelvin *
                std::expm1((gamma - 1.0) / gamma *
                           std::log1p(0.92 * 0.92 * 100.0 / 101325.0)));
  assert(std::abs(positive_speed.airspeed_mps -
                  expected_pressure_corrected_speed) < 1.0e-12);
  const auto unit_coefficient_speed = sensors::computeSaintVenantAirspeed(
      101325.0, 100.0, 20.0, 1.0);
  assert(unit_coefficient_speed.valid &&
         unit_coefficient_speed.airspeed_mps > positive_speed.airspeed_mps);
  assert(!sensors::computeSaintVenantAirspeed(
              0.0, 100.0, 20.0, 0.92)
              .valid);
  assert(!sensors::computeSaintVenantAirspeed(
              101325.0, -1.0, 20.0, 0.92)
              .valid);

  sensors::AirDataFlightLogic air_data_logic;
  sensors::AirDataFlightEvent event{};
  for (int index = 0; index < 10; ++index)
    event = air_data_logic.update(
        1, protocol::MissionState::liftoff_detection, 0,
        1000.0 - index * 0.3, true);
  assert(event.flight_epoch == 1 && event.lps_liftoff_detected);
  for (int index = 0; index < 30; ++index)
    event = air_data_logic.update(1, protocol::MissionState::engine_burn,
                                  10'000'000, 900.0 + index, true);
  assert(event.pressure_apex_detected);
  event = air_data_logic.update(2, protocol::MissionState::liftoff_detection,
                                0, 800.0, true);
  assert(event.flight_epoch == 2 && !event.lps_liftoff_detected &&
         !event.pressure_apex_detected);
}

void testAttitudeContinuity() {
  using sensors::AttitudeEstimator;
  using sensors::AttitudeInvalidReason;
  using sensors::GyroHistoryRing;
  using sensors::GyroSample;
  GyroHistoryRing history;
  history.push({1'000, 1.0, 4, 0, true, false, false, false});
  AttitudeEstimator estimator;
  assert(estimator.beginFlight(history, 1'000, 0.0));
  assert(estimator.update({2'000, 1.0, 4, 0, true, false, true, false}));
  assert(estimator.state().fifo_full_event_count == 1);
  assert(estimator.state().valid);
  assert(estimator.update({4'000, 3.0, 4, 1, true, false, false, false}));
  assert(estimator.state().interpolated_sample_count == 1);
  assert(std::abs(
             estimator.state().roll_estimate_liftoff_relative_unwrapped_rad -
             0.005) < 1.0e-12);
  assert(!estimator.update(
      {5'000, 3.0, 5, 0, true, false, false, false}));
  assert(estimator.state().invalid_reason ==
         AttitudeInvalidReason::timestamp_epoch_changed);

  AttitudeEstimator format;
  assert(format.beginFlight(history, 1'000, 0.0));
  assert(!format.update({2'000, 1.0, 4, 0, true, false, false, true}));
  assert(format.state().invalid_reason ==
         AttitudeInvalidReason::fifo_format_fault);

  AttitudeEstimator loss;
  assert(loss.beginFlight(history, 1'000, 0.0));
  assert(!loss.update({4'000, 1.0, 4, 2, true, false, false, false}));
  assert(loss.state().invalid_reason ==
         AttitudeInvalidReason::excess_data_loss);

  constexpr double kPi = 3.14159265358979323846;
  for (const double direction : {1.0, -1.0}) {
    GyroHistoryRing multi_turn_history;
    const double rate = direction * 2.0 * kPi;
    multi_turn_history.push(
        {1'000, rate, 6, 0, true, false, false, false});
    AttitudeEstimator multi_turn;
    assert(multi_turn.beginFlight(multi_turn_history, 1'000, 0.0));
    for (uint64_t index = 1; index <= 2'000; ++index) {
      assert(multi_turn.update({1'000 + index * 1'000, rate, 6, 0, true,
                                false, false, false}));
      if (index == 1'000)
        assert(std::abs(multi_turn.state()
                            .roll_estimate_liftoff_relative_unwrapped_rad -
                        direction * 2.0 * kPi) < 1.0e-9);
    }
    assert(std::abs(multi_turn.state()
                        .roll_estimate_liftoff_relative_unwrapped_rad -
                    direction * 4.0 * kPi) < 1.0e-9);
  }
}

void testControlPipeline() {
  using namespace control;
  constexpr double kPi = 3.14159265358979323846;

  RollGainSchedule schedule{};
  schedule.configured = true;
  for (std::size_t index = 0; index < schedule.points.size(); ++index) {
    schedule.points[index].airspeed_mps = 60.0 + index * 20.0;
    schedule.points[index].gain[0] = 1.0 + index * 2.0;
    schedule.points[index].gain[2] = 3.0;
  }
  RollController roll{schedule};
  const auto request =
      roll.compute({0.1, 0, 0, 0}, 70.0, RollControlAuthority::gentle);
  assert(request.valid && !request.saturated);
  assert(std::abs(request.output_torque_nm + 0.2) < 1.0e-12);
  assert(roll.compute({0, 0, 0, 0}, 60.0,
                      RollControlAuthority::gentle).valid);
  const RollState lookup_probe{0.1, 0.0, 0.0, 0.0};
  const auto at_60 = roll.compute(lookup_probe, 60.0,
                                  RollControlAuthority::high_authority);
  const auto at_180 = roll.compute(lookup_probe, 180.0,
                                   RollControlAuthority::high_authority);
  for (const double speed : {59.9, 60.0})
    assert(roll.compute(lookup_probe, speed,
                        RollControlAuthority::high_authority)
               .output_torque_nm == at_60.output_torque_nm);
  for (const double speed : {180.0, 180.1, 200.0, 220.0})
    assert(roll.compute(lookup_probe, speed,
                        RollControlAuthority::high_authority)
               .output_torque_nm == at_180.output_torque_nm);
  assert(roll.compute(lookup_probe, 60.1,
                      RollControlAuthority::high_authority)
             .output_torque_nm < at_60.output_torque_nm);
  assert(roll.compute(lookup_probe, 179.9,
                      RollControlAuthority::high_authority)
             .output_torque_nm > at_180.output_torque_nm);
  const auto control_off = roll.compute(
      lookup_probe, 220.0, RollControlAuthority::high_authority,
      board::kControlAuthorityLimits, RollVerificationMode::matched_control_off);
  assert(control_off.valid && !control_off.saturated &&
         control_off.output_torque_nm == 0.0);
  const double radians380 = 380.0 * kPi / 180.0;
  const double reference720 = 720.0 * kPi / 180.0;
  const double current1100 = 1100.0 * kPi / 180.0;
  assert(std::abs((current1100 - reference720) - radians380) < 1.0e-12);
  const double positive_pi_crossing =
      181.0 * kPi / 180.0 - 179.0 * kPi / 180.0;
  const double negative_pi_crossing =
      -181.0 * kPi / 180.0 - (-179.0 * kPi / 180.0);
  assert(std::abs(positive_pi_crossing - 2.0 * kPi / 180.0) < 1.0e-12);
  assert(std::abs(negative_pi_crossing + 2.0 * kPi / 180.0) < 1.0e-12);
  assert(std::abs(((current1100 + 4.0 * kPi) -
                   (reference720 + 4.0 * kPi)) -
                  radians380) < 1.0e-12);
  assert(std::abs(((current1100 + 2.0 * kPi) - reference720) -
                  (radians380 + 2.0 * kPi)) <
         1.0e-12);
  const auto positive_turn = roll.compute(
      {radians380, 0, 0, 0}, 60.0, RollControlAuthority::gentle);
  const auto negative_turn = roll.compute(
      {-radians380, 0, 0, 0}, 60.0, RollControlAuthority::gentle);
  assert(positive_turn.output_torque_nm == -1.21208);
  assert(negative_turn.output_torque_nm == 1.21208);
  assert(roll.compute({radians380 + 2.0 * kPi, 0, 0, 0},
                      60.0, RollControlAuthority::high_authority)
             .output_torque_nm == -3.0);
  const auto entry_rate_feedback = roll.compute(
      {0.0, 0.0, 0.2, 0.0}, 60.0, RollControlAuthority::gentle);
  assert(entry_rate_feedback.valid);
  assert(std::abs(entry_rate_feedback.output_torque_nm + 0.6) < 1.0e-12);

  const board::ControlAuthorityLimits authority{};
  assert(authority.hold_position_limit_Nm == 0.30);
  assert(authority.zero_hold_requested_torque_limit_Nm == 0.80);
  assert(authority.roll_control_gentle_limit_Nm == 1.21208);
  assert(authority.roll_control_high_authority_limit_Nm == 3.0);
  assert(authority.valid());
  assert(board::kControlAuthorityLimits.roll_control_gentle_limit_Nm ==
         1.21208);
  assert(board::kControlAuthorityLimits
             .zero_hold_requested_torque_limit_Nm == 0.80);
  const board::EncoderPipelineConfig encoder_2k{2'000, 1'000};
  const board::EncoderPipelineConfig encoder_5k{5'000, 1'000};
  const board::EncoderPipelineConfig encoder_invalid{2'500, 1'000};
  assert(encoder_2k.samplesPerBlock() == 2);
  assert(encoder_5k.samplesPerBlock() == 5);
  assert(!encoder_invalid.valid());
  const board::PitotCoefficientDiagnosticsConfig pitot{};
  assert(pitot.pitot_coefficient_assumed == 0.92);
  assert(pitot.pitot_coefficient_true_min == 0.60);
  assert(pitot.pitot_coefficient_true_max == 1.20);
  assert(pitot.valid());
  assert(board::kPitotCoefficientDiagnostics.pitot_coefficient_assumed ==
         0.92);
  assert(board::kPitotCoefficientDiagnostics.pitot_coefficient_true_min ==
         0.60);
  assert(board::kPitotCoefficientDiagnostics.pitot_coefficient_true_max ==
         1.20);
  assert(board::kEncoderPipeline.valid());

  ZeroHoldController zero;
  const auto zero_request = zero.compute(1.0, 0.0);
  assert(zero_request.valid && zero_request.saturated &&
         zero_request.output_torque_nm == -0.8);
  for (int index = 0; index < 99; ++index)
    assert(!zero.updateValidity(0.0, 0.0, true));
  assert(zero.updateValidity(0.0, 0.0, true));
  assert(!zero.updateValidity(0.0, 0.0, false));

  QuadraticN3FinVelocityEstimator velocity;
  double rate{};
  assert(!velocity.update(1'000, 1.0e-6, rate));
  assert(!velocity.update(2'000, 4.0e-6, rate));
  assert(velocity.update(3'000, 9.0e-6, rate));
  assert(std::abs(rate - 0.006) < 1.0e-12);
  assert(!velocity.update(3'000, 9.0e-6, rate));

  const board::MotorProfile profile{
      1, board::MotorPolarity::positive_in1, true, 3.48F, 0.00855F,
      1120.0F, 0.60F, 2.0F, 1.21208F};
  const board::FinSoftwareLimits limits{true, -0.2F, 0.2F};
  TorqueMapper mapper{profile, limits};
  auto motor = mapper.map(0.5, 0.0, 0.0, 9.0);
  assert(motor.valid && !motor.brake && motor.positive_in1);
  motor = mapper.map(0.5, 0.21, 1.0, 9.0);
  assert(motor.valid && motor.brake && motor.saturated &&
         motor.requested_output_torque_nm == 0 && motor.pwm_duty == 0);
  motor = mapper.map(-0.5, 0.21, 1.0, 9.0);
  assert(motor.valid && !motor.brake);
  motor = mapper.map(10.0, 0.0, 0.0, 9.0);
  assert(motor.valid && motor.saturated &&
         motor.requested_output_torque_nm <= 1.212081);
  const board::FinSoftwareLimits unconfigured{};
  assert(!TorqueMapper(profile, unconfigured).map(0.1, 0, 0, 9).valid);
  assert(std::abs(DrivetrainStructuralModel::elasticDeflectionRad(269.43) -
                  1.0) < 1.0e-12);
  assert(DrivetrainStructuralModel::conservativeLostMotionRad() == 0.006);
  assert(judgeStopperContact({0.1, 0.0, 1.0, false}) ==
         StopperJudgement::pass);
  assert(judgeStopperContact({0.1, 1.0, 1.0, true}) ==
         StopperJudgement::needs_hardware_validation);
  assert(judgeStopperContact({0.3, 0.0, 1.0, false}) ==
         StopperJudgement::fail);
  assert(MissionControlCoordinator::choose(true, true, true, true, false) ==
         ControlMode::roll_control);
  assert(MissionControlCoordinator::choose(true, true, false, true, false) ==
         ControlMode::brake);
}

void testParachuteAndRecovery() {
  using namespace actuators;
  PowerArbiter power;
  assert(power.requestAuxiliary5v(true));
  assert(power.requestParachutePower(true));
  power.latchDeploymentCutoff();
  assert(!power.state().auxiliary_5v && !power.state().parachute_power);
  assert(!power.requestAuxiliary5v(true));
  assert(!power.requestParachutePower(true));

  ParachuteController parachute;
  assert(parachute.startOpen(1'000'000, 4095) ==
         ParachuteAction::command_open);
  assert(parachute.tick({1'499'999, true, 4095, false}) ==
         ParachuteAction::none);
  assert(parachute.tick({1'500'000, true, 4095, false}) ==
         ParachuteAction::retry_open);
  assert(parachute.status().retry_count == 1);
  assert(parachute.tick({2'000'000, true, 30, false}) ==
         ParachuteAction::none);
  assert(parachute.tick({6'000'000, false, 0, false}) ==
         ParachuteAction::hold_position);
  assert(parachute.status().state == ParachuteOpenState::retry_exhausted);

  ParachuteController positive_wrap;
  assert(positive_wrap.startOpen(0, 4095) ==
         ParachuteAction::command_open);
  assert(positive_wrap.tick({500'000, true, 30, false}) ==
         ParachuteAction::none);
  ParachuteController negative_wrap;
  assert(negative_wrap.startOpen(0, 0) == ParachuteAction::command_open);
  assert(negative_wrap.tick({500'000, true, 4060, false}) ==
         ParachuteAction::none);
  ParachuteController half_turn_progress;
  assert(half_turn_progress.startOpen(0, 0) ==
         ParachuteAction::command_open);
  assert(half_turn_progress.tick({500'000, true, 2048, false}) ==
         ParachuteAction::retry_open);

  ParachuteController confirmed;
  assert(confirmed.startOpen(0, 0) == ParachuteAction::command_open);
  assert(confirmed.tick({100'000, true, 30, true}) ==
         ParachuteAction::hold_position);
  assert(confirmed.status().servo_open_confirmed);
  confirmed.notifyPowerCutoff();
  assert(confirmed.status().state == ParachuteOpenState::powered_off);

  using namespace mission;
  const auto marker = makeRecoveryMarker(7);
  assert(validRecoveryMarker(marker));
  assert(selectBootPath({ResetKind::deep_sleep, false, false, false, false,
                         true, marker}) == BootPath::recovery_only);
  assert(selectBootPath({ResetKind::deep_sleep, false, false, false, false,
                         false, marker}) == BootPath::fail_safe);
  assert(selectBootPath({ResetKind::software_or_watchdog, true, true, false,
                         false, false, {}}) ==
         BootPath::resume_flight_safely);
  assert(selectBootPath({ResetKind::power_on, true, false, false, false,
                         false, {}}) == BootPath::await_absolute_time);
  RecoveryRuntime recovery;
  assert(recovery.requestEntry());
  assert(recovery.markResourcesSafeAndFlushed());
  assert(recovery.mayEnterDeepSleep());
  assert(recovery.wake(true, true));
  assert(recovery.beginTransfer());
  assert(!recovery.mayEnterDeepSleep());
  recovery.finishTransfer();
  assert(recovery.mayEnterDeepSleep());
}

void testRuntimeQueue() {
  runtime::SpscBoundedQueue<uint32_t, 2> queue;
  assert(queue.push(1));
  assert(queue.push(2));
  assert(!queue.push(3));
  assert(queue.overflowCount() == 1);
  uint32_t value{};
  assert(queue.pop(value) && value == 1);
  assert(queue.pop(value) && value == 2);
  assert(!queue.pop(value));
  assert(runtime::kTaskArchitecture[0].owner ==
         runtime::HardwareOwner::deployment_power);
  assert(runtime::kTaskArchitecture[2].owner ==
         runtime::HardwareOwner::mission_spi_and_motor);

  runtime::EmergencyLatch emergency;
  assert(!emergency.signal(42));
  assert(emergency.signal(43));
  uint8_t transaction{};
  assert(emergency.pending());
  assert(emergency.take(transaction) && transaction == 43);
  assert(!emergency.take(transaction));
}

} // 無名名前空間


void testForceStartReadinessAndState() {
  using mission::CommandCode;
  using mission::CommandContext;
  using mission::CommandExecutor;
  using mission::MissionStateMachine;
  using mission::PreflightReadinessBit;
  using mission::PreflightReadinessSnapshot;
  using mission::SequenceConfiguration;
  using protocol::CommandPhase;
  using protocol::CommandReason;
  using protocol::MissionState;

  PreflightReadinessSnapshot readiness{};
  readiness.generation = 7;
  readiness.captured_at_us = 1234;
  readiness.resources_preallocated = true;
  readiness.motor_profile_valid = true;
  readiness.gyro_bias_valid = true;
  readiness.gravity_reference_valid = true;
  readiness.ssc_zero_valid = true;
  readiness.parachute_open_configured = true;
  readiness.parachute_close_configured = true;
  assert(readiness.missingMask() ==
         mission::preflightReadinessBit(
             PreflightReadinessBit::fin_zero_configured));
  assert(!readiness.ready());

  CommandContext context{};
  context.resources_preallocated = true;
  context.persistence_load_complete = true;
  context.persistence_ready = true;
  CommandExecutor executor;
  const auto normal = executor.begin(
      command(40, static_cast<uint8_t>(CommandCode::start_sequence)), context);
  assert(normal.execute && normal.result.phase == CommandPhase::accepted);
  (void)executor.finish(40, CommandPhase::failed,
                        CommandReason::not_configured,
                        readiness.missingMask());

  const auto force = executor.begin(
      command(41, static_cast<uint8_t>(CommandCode::force_start_sequence)),
      context);
  assert(force.execute && force.result.phase == CommandPhase::accepted);
  (void)executor.finish(41, CommandPhase::completed, CommandReason::none,
                        readiness.missingMask());

  auto malformed =
      command(42, static_cast<uint8_t>(CommandCode::force_start_sequence));
  malformed.arguments[0] = 1;
  assert(executor.begin(malformed, context).result.reason ==
         CommandReason::invalid_argument);

  CommandContext loading = context;
  loading.persistence_load_complete = false;
  CommandExecutor loading_executor;
  assert(loading_executor
             .begin(command(43, static_cast<uint8_t>(
                                    CommandCode::force_start_sequence)),
                    loading)
             .result.reason == CommandReason::busy);
  CommandContext broken_persistence = context;
  broken_persistence.persistence_ready = false;
  CommandExecutor persistence_executor;
  assert(persistence_executor
             .begin(command(44, static_cast<uint8_t>(
                                    CommandCode::force_start_sequence)),
                    broken_persistence)
             .result.reason == CommandReason::persistence_error);

  MissionStateMachine normal_state;
  const SequenceConfiguration normal_configuration{readiness, false};
  assert(normal_state.startSequence(0, normal_configuration) ==
         mission::TransitionResult::not_configured);

  MissionStateMachine forced_state;
  const SequenceConfiguration forced_configuration{readiness, true};
  assert(forced_state.startSequence(0, forced_configuration) ==
         mission::TransitionResult::completed);
  assert(forced_state.snapshot().state == MissionState::liftoff_detection);
  assert(forced_state.snapshot().forced_start);
  assert(forced_state.snapshot().preflight_missing_mask ==
         readiness.missingMask());
  assert(forced_state.cancelSequence() == mission::TransitionResult::completed);
  assert(!forced_state.snapshot().forced_start &&
         forced_state.snapshot().preflight_missing_mask == 0);

  MissionStateMachine emergency_state;
  PreflightReadinessSnapshot ready = readiness;
  ready.fin_zero_configured = true;
  assert(emergency_state.startSequence(0, {ready, true}) ==
         mission::TransitionResult::completed);
  mission::MissionTickInput liftoff{};
  liftoff.monotonic_us = 2'000'000;
  liftoff.liftoff_detected = true;
  emergency_state.tick(liftoff);
  assert(emergency_state.snapshot().state == MissionState::engine_burn);
  assert(emergency_state.liftoffDetectionEmergencyStop() ==
         mission::TransitionResult::completed);
  assert(emergency_state.snapshot().forced_start);

  mission::ResetCheckpoint checkpoint{};
  checkpoint.valid = true;
  checkpoint.state = MissionState::liftoff_detection;
  checkpoint.flight_epoch = emergency_state.snapshot().flight_epoch;
  checkpoint.elapsed_valid = false;
  checkpoint.forced_start = true;
  checkpoint.preflight_missing_mask = readiness.missingMask();
  MissionStateMachine restored;
  assert(restored.restoreAfterReset(5'000'000, checkpoint) ==
         mission::TransitionResult::completed);
  assert(restored.snapshot().state == MissionState::liftoff_detection);
  assert(restored.snapshot().forced_start);
  assert(restored.snapshot().preflight_missing_mask ==
         readiness.missingMask());
}

void testParachuteDeadlineKeepsPowerPolicySeparate() {
  actuators::ParachuteController controller;
  assert(controller.startOpen(1'000, 100) ==
         actuators::ParachuteAction::command_open);
  const auto deadline = controller.tick({5'001'000, true, 100, false});
  assert(deadline == actuators::ParachuteAction::hold_position);
  assert(controller.status().state ==
         actuators::ParachuteOpenState::retry_exhausted);
  assert(controller.status().open_attempt_finished);
  assert(!controller.status().servo_open_confirmed);

  actuators::ParachuteController success;
  assert(success.startOpen(10'000, 100) ==
         actuators::ParachuteAction::command_open);
  assert(success.tick({20'000, true, 200, true}) ==
         actuators::ParachuteAction::hold_position);
  assert(success.status().servo_open_confirmed);
  success.notifyPowerCutoff();
  assert(success.status().state == actuators::ParachuteOpenState::powered_off);
}

int main() {
  const auto golden = readGolden();
  testCanGolden(golden);
  testQuantization(golden);
  testMissionStateMachine();
  testCommandExecutor();
  testForceStartReadinessAndState();
  testParachuteDeadlineKeepsPowerPolicySeparate();
  testSensors();
  testAttitudeContinuity();
  testControlPipeline();
  testParachuteAndRecovery();
  testRuntimeQueue();
  std::cout << "mission host tests: PASS\n";
}
