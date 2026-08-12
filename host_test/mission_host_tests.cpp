#include "actuators/safety_core.hpp"
#include "control/control_pipeline.hpp"
#include "mission/command_executor.hpp"
#include "mission/mission_state.hpp"
#include "mission/recovery.hpp"
#include "protocol/can_protocol.hpp"
#include "protocol/quantization.hpp"
#include "runtime/task_architecture.hpp"
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
              encode(MissionEvent{0xFF, 0x4061, MissionState::control, 1234,
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
  return {true, true, true, true, true, true, true, true};
}

void testMissionStateMachine() {
  using mission::FinDirective;
  using mission::MissionStateMachine;
  using mission::SequenceConfiguration;
  using mission::TransitionResult;
  using protocol::MissionState;

  const SequenceConfiguration ready{true, true, true, true};
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
  control.tick({9'000'000, false, false, readyControl()});
  assert(control.snapshot().state == MissionState::control);
  auto unavailable = readyControl();
  unavailable.ssc_available = false;
  control.tick({9'001'000, false, false, unavailable});
  assert(control.snapshot().state == MissionState::engine_burn);
  assert(control.snapshot().control_reentry_inhibited);
  control.tick({10'000'000, false, false, readyControl()});
  assert(control.snapshot().state == MissionState::engine_burn);

  MissionStateMachine gate_failure;
  assert(gate_failure.startSequence(0, ready) == TransitionResult::completed);
  gate_failure.tick({2'000'000, true, false, readyControl()});
  gate_failure.tick({9'000'000, false, false, unavailable});
  assert(gate_failure.snapshot().control_reentry_inhibited);
  gate_failure.tick({9'100'000, false, false, readyControl()});
  assert(gate_failure.snapshot().state == MissionState::engine_burn);

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
  assert(reset.snapshot().fin == FinDirective::brake);
  reset.tick({100'000'000, false, false, readyControl()});
  assert(reset.snapshot().state == MissionState::descent);
  reset.tick({105'000'000, false, false, readyControl()});
  assert(reset.snapshot().deployment_power_cutoff_latched);
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
  assert(std::abs(estimator.state().roll_rad - 0.005) < 1.0e-12);
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
}

void testControlPipeline() {
  using namespace control;
  RollGainSchedule schedule{};
  schedule.configured = true;
  for (std::size_t index = 0; index < schedule.points.size(); ++index) {
    schedule.points[index].airspeed_mps = 60.0 + index * 20.0;
    schedule.points[index].gain[0] = 1.0 + index * 2.0;
  }
  RollController roll{schedule};
  const double degrees370 = 370.0 * 3.14159265358979323846 / 180.0;
  assert(std::abs(RollController::wrapRollError(degrees370) +
                  10.0 * 3.14159265358979323846 / 180.0) < 1.0e-12);
  const auto request = roll.compute({0.1, 0, 0, 0}, 70.0, 1.0);
  assert(request.valid && !request.saturated);
  assert(std::abs(request.output_torque_nm - 0.2) < 1.0e-12);
  assert(!roll.compute({0, 0, 0, 0}, 60.0, 1.0).valid);

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
  motor = mapper.map(0.5, 0.21, 0.0, 9.0);
  assert(motor.valid && motor.saturated && motor.requested_output_torque_nm == 0);
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
  assert(parachute.startOpen(1'000'000, 10.0) ==
         ParachuteAction::command_open);
  assert(parachute.tick({1'499'999, true, 10.0, false}) ==
         ParachuteAction::none);
  assert(parachute.tick({1'500'000, true, 10.0, false}) ==
         ParachuteAction::retry_open);
  assert(parachute.status().retry_count == 1);
  assert(parachute.tick({2'000'000, true, 13.0, false}) ==
         ParachuteAction::none);
  assert(parachute.tick({6'000'000, false, 0.0, false}) ==
         ParachuteAction::cut_power);
  assert(parachute.status().state == ParachuteOpenState::retry_exhausted);

  ParachuteController confirmed;
  assert(confirmed.startOpen(0, 0) == ParachuteAction::command_open);
  assert(confirmed.tick({100'000, true, 30.0, true}) ==
         ParachuteAction::cut_power);
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
}

} // 無名名前空間

int main() {
  const auto golden = readGolden();
  testCanGolden(golden);
  testQuantization(golden);
  testMissionStateMachine();
  testCommandExecutor();
  testSensors();
  testAttitudeContinuity();
  testControlPipeline();
  testParachuteAndRecovery();
  testRuntimeQueue();
  std::cout << "mission host tests: PASS\n";
}
