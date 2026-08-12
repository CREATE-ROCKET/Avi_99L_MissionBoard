#include "protocol/can_protocol.hpp"
#include "protocol/quantization.hpp"

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

} // 無名名前空間

int main() {
  const auto golden = readGolden();
  testCanGolden(golden);
  testQuantization(golden);
  std::cout << "mission host tests: PASS\n";
}
