#include "protocol/quantization.hpp"

#include <cmath>
#include <limits>

namespace protocol::quantization {
namespace {

template <typename Raw>
Decoded<Raw> numeric(Raw raw, double value) {
  return {Semantic::numeric, value, raw};
}

template <typename Raw> Decoded<Raw> error(Raw raw) {
  return {Semantic::error, 0.0, raw};
}

template <typename Raw> Decoded<Raw> reserved(Raw raw) {
  return {Semantic::reserved, 0.0, raw};
}

bool finite(double value) { return std::isfinite(value); }

uint16_t signedRaw(long value, unsigned width) {
  const uint32_t mask = (1U << width) - 1U;
  return static_cast<uint16_t>(static_cast<uint32_t>(value) & mask);
}

int32_t signedValue(uint16_t raw, unsigned width) {
  const uint32_t sign = 1U << (width - 1U);
  const uint32_t mask = (1U << width) - 1U;
  const uint32_t value = raw & mask;
  return static_cast<int32_t>((value ^ sign) - sign);
}

} // 無名名前空間

uint16_t encodeRoll(double degrees, RollError invalid) {
  if (!finite(degrees))
    return static_cast<uint16_t>(invalid);
  const long count = std::lround(degrees / 0.5);
  if (count < -32752L || count > 32767L)
    return static_cast<uint16_t>(RollError::out_of_range);
  return signedRaw(count, 16);
}

Decoded<uint16_t> decodeRoll(uint16_t raw) {
  if (raw >= 0x8000 && raw <= 0x800F)
    return error(raw);
  return numeric(raw, static_cast<double>(static_cast<int16_t>(raw)) * 0.5);
}

uint16_t encodeRollRate(double value, RollError invalid) {
  if (!finite(value))
    return static_cast<uint16_t>(invalid);
  const long count = std::lround(value / 0.1);
  if (count < -32752L || count > 32767L)
    return static_cast<uint16_t>(RollError::out_of_range);
  return signedRaw(count, 16);
}

Decoded<uint16_t> decodeRollRate(uint16_t raw) {
  if (raw >= 0x8000 && raw <= 0x800F)
    return error(raw);
  return numeric(raw, static_cast<double>(static_cast<int16_t>(raw)) * 0.1);
}

uint8_t encodeTiltMagnitude(double degrees, uint8_t error_raw) {
  if (!finite(degrees) || degrees < 0.0 || degrees > 90.0)
    return error_raw;
  return static_cast<uint8_t>(std::lround(degrees / 0.75));
}

Decoded<uint8_t> decodeTiltMagnitude(uint8_t raw) {
  if (raw <= 120)
    return numeric(raw, static_cast<double>(raw) * 0.75);
  return raw <= 127 ? error(raw) : reserved(raw);
}

uint16_t encodeTiltDirection(double degrees) {
  if (!finite(degrees))
    return 511;
  double normalized = std::fmod(degrees, 360.0);
  if (normalized < 0.0)
    normalized += 360.0;
  return static_cast<uint16_t>(std::lround(normalized)) % 360U;
}

Decoded<uint16_t> decodeTiltDirection(uint16_t raw) {
  return raw <= 359 ? numeric(raw, raw) : reserved(raw);
}

uint8_t encodeFinAngle(double degrees, FinAngleError invalid) {
  if (!finite(degrees) || degrees < -15.0 || degrees > 15.0)
    return static_cast<uint8_t>(invalid);
  return static_cast<uint8_t>(std::lround((degrees + 15.0) / 0.125));
}

Decoded<uint8_t> decodeFinAngle(uint8_t raw) {
  if (raw <= 240)
    return numeric(raw, -15.0 + static_cast<double>(raw) * 0.125);
  return error(raw);
}

uint16_t encodeFinRate(double value, FinRateError invalid) {
  if (!finite(value))
    return static_cast<uint16_t>(invalid);
  const long count = std::lround(value / 0.02);
  if (count < -32752L || count > 32767L)
    return static_cast<uint16_t>(FinRateError::out_of_range);
  return signedRaw(count, 16);
}

Decoded<uint16_t> decodeFinRate(uint16_t raw) {
  if (raw >= 0x8000 && raw <= 0x8009)
    return error(raw);
  if (raw >= 0x800A && raw <= 0x800F)
    return reserved(raw);
  return numeric(raw, static_cast<double>(static_cast<int16_t>(raw)) * 0.02);
}

uint16_t encodeRequestedTorque(double value, TorqueError invalid) {
  // TODO(SIMULATION): 0.002 N.m/LSBはSpica Phase 9で確定する。
  if (!finite(value))
    return static_cast<uint16_t>(invalid);
  const long count = std::lround(value / 0.002);
  if (count < -2032L || count > 2047L)
    return static_cast<uint16_t>(TorqueError::limit_config_invalid);
  return signedRaw(count, 12);
}

Decoded<uint16_t> decodeRequestedTorque(uint16_t raw) {
  if (raw > 0x0FFFU)
    return reserved(raw);
  raw &= 0x0FFFU;
  if (raw >= 0x800 && raw <= 0x806)
    return error(raw);
  if (raw >= 0x807 && raw <= 0x80F)
    return reserved(raw);
  return numeric(raw, static_cast<double>(signedValue(raw, 12)) * 0.002);
}

uint16_t encodeLpsPressure(double value, LpsPressureError invalid) {
  if (!finite(value))
    return static_cast<uint16_t>(invalid);
  if (value < 800.0)
    return static_cast<uint16_t>(LpsPressureError::below_range);
  if (value > 1206.2)
    return static_cast<uint16_t>(LpsPressureError::above_range);
  return static_cast<uint16_t>(std::lround((value - 800.0) / 0.2));
}

Decoded<uint16_t> decodeLpsPressure(uint16_t raw) {
  if (raw <= 2031)
    return numeric(raw, 800.0 + static_cast<double>(raw) * 0.2);
  return raw <= 2047 ? error(raw) : reserved(raw);
}

uint8_t encodeLpsTemperature(double value, LpsTemperatureError invalid) {
  if (!finite(value))
    return static_cast<uint8_t>(invalid);
  if (value < -50.0)
    return static_cast<uint8_t>(LpsTemperatureError::below_range);
  if (value > 150.0)
    return static_cast<uint8_t>(LpsTemperatureError::above_range);
  return static_cast<uint8_t>(std::lround(value + 50.0));
}

Decoded<uint8_t> decodeLpsTemperature(uint8_t raw) {
  if (raw <= 200)
    return numeric(raw, -50.0 + raw);
  if (raw <= 239)
    return reserved(raw);
  return error(raw);
}

uint8_t encodeAirspeed(double value, AirspeedError invalid) {
  if (!finite(value))
    return static_cast<uint8_t>(invalid);
  if (value < 0.0)
    return static_cast<uint8_t>(AirspeedError::negative_differential_pressure);
  if (value > 245.0)
    return static_cast<uint8_t>(AirspeedError::above_range);
  return static_cast<uint8_t>(std::lround(value));
}

Decoded<uint8_t> decodeAirspeed(uint8_t raw) {
  return raw <= 245 ? numeric(raw, raw) : error(raw);
}

uint8_t encodeFlightElapsed(double seconds, TimeError invalid) {
  if (!finite(seconds) || seconds < 0.0)
    return static_cast<uint8_t>(0xF0U | static_cast<uint8_t>(invalid));
  if (seconds >= 24.0)
    return 0xFBU;
  return static_cast<uint8_t>(std::floor(seconds * 10.0));
}

Decoded<uint8_t> decodeFlightElapsed(uint8_t raw) {
  return raw <= 0xEF ? numeric(raw, raw * 0.1) : error(raw);
}

uint16_t encodeDescentElapsed(double seconds, TimeError invalid) {
  if (!finite(seconds) || seconds < 0.0)
    return static_cast<uint16_t>(0xFFF0U | static_cast<uint8_t>(invalid));
  if (seconds >= 6552.0)
    return 0xFFFBU;
  return static_cast<uint16_t>(std::floor(seconds * 10.0));
}

Decoded<uint16_t> decodeDescentElapsed(uint16_t raw) {
  return raw <= 0xFFEF ? numeric(raw, raw * 0.1) : error(raw);
}

uint16_t encodeRecoveryElapsed(double seconds, TimeError invalid) {
  if (!finite(seconds) || seconds < 0.0)
    return static_cast<uint16_t>(0xFFF0U | static_cast<uint8_t>(invalid));
  const double count = std::floor(seconds / 10.0);
  if (count > 0xFFEF)
    return 0xFFFBU;
  return static_cast<uint16_t>(count);
}

Decoded<uint16_t> decodeRecoveryElapsed(uint16_t raw) {
  return raw <= 0xFFEF ? numeric(raw, raw * 10.0) : error(raw);
}

uint16_t encodeGnssOffset(double metres, GnssError invalid) {
  if (!finite(metres))
    return static_cast<uint16_t>(invalid);
  const long count = std::lround(metres);
  if (count < -32752L || count > 32767L)
    return static_cast<uint16_t>(GnssError::out_of_range);
  return signedRaw(count, 16);
}

Decoded<uint16_t> decodeGnssOffset(uint16_t raw) {
  if (raw >= 0x8000 && raw <= 0x8008)
    return error(raw);
  if (raw >= 0x8009 && raw <= 0x800F)
    return reserved(raw);
  return numeric(raw, static_cast<double>(static_cast<int16_t>(raw)));
}

uint16_t encodeGnssHeight(double metres, GnssHeightError invalid) {
  if (!finite(metres))
    return static_cast<uint16_t>(invalid);
  if (metres < -100.0 || metres > 2375.0)
    return static_cast<uint16_t>(GnssHeightError::out_of_range);
  return static_cast<uint16_t>(std::lround((metres + 100.0) / 5.0));
}

Decoded<uint16_t> decodeGnssHeight(uint16_t raw) {
  if (raw > 0x01FFU)
    return reserved(raw);
  raw &= 0x01FFU;
  if (raw <= 495)
    return numeric(raw, -100.0 + raw * 5.0);
  if (raw <= 503)
    return error(raw);
  return reserved(raw);
}

uint8_t encodeParachuteAngle(double degrees, ParachuteAngleError invalid) {
  if (!finite(degrees) || degrees < 0.0 || degrees > 360.0)
    return static_cast<uint8_t>(invalid);
  return static_cast<uint8_t>(std::lround(degrees / 1.5));
}

Decoded<uint8_t> decodeParachuteAngle(uint8_t raw) {
  return raw <= 240 ? numeric(raw, raw * 1.5) : error(raw);
}

uint8_t encodeBatteryVoltage(double volts, BatteryError invalid) {
  if (!finite(volts))
    return static_cast<uint8_t>(invalid);
  if (volts < 0.0 || volts > 12.0)
    return static_cast<uint8_t>(BatteryError::adc_error);
  return static_cast<uint8_t>(std::lround(volts / 0.05));
}

Decoded<uint8_t> decodeBatteryVoltage(uint8_t raw) {
  if (raw <= 240)
    return numeric(raw, raw * 0.05);
  if (raw <= 252)
    return reserved(raw);
  return error(raw);
}

} // 名前空間 protocol::quantization
