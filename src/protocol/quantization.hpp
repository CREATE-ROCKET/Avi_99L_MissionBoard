#pragma once

#include <cstdint>

namespace protocol::quantization {

enum class Semantic : uint8_t { numeric, error, reserved };

template <typename Raw> struct Decoded {
  Semantic semantic{Semantic::reserved};
  double value{};
  Raw raw{};
};

enum class RollError : uint16_t {
  unavailable = 0x8000,
  not_initialized = 0x8001,
  spi_timeout = 0x8002,
  spi_error = 0x8003,
  stale = 0x8004,
  fifo_full = 0x8005,
  fifo_lost_packet = 0x8006,
  fifo_format_fault = 0x8007,
  sample_invalid = 0x8008,
  odr_changed = 0x8009,
  out_of_range = 0x800A,
  timestamp_invalid = 0x800B,
  estimator_invalid = 0x800C,
  reset_invalidated = 0x800D,
  internal_error = 0x800E,
  unknown = 0x800F,
};

enum class FinRateError : uint16_t {
  unavailable = 0x8000,
  source_angle_error = 0x8001,
  stale = 0x8002,
  not_enough_samples = 0x8003,
  unwrap_ambiguous = 0x8004,
  timestamp_invalid = 0x8005,
  estimator_not_ready = 0x8006,
  estimator_numeric_error = 0x8007,
  out_of_range = 0x8008,
  reset_invalidated = 0x8009,
};

enum class FinAngleError : uint8_t {
  not_initialized = 241,
  spi_timeout = 242,
  spi_error = 243,
  response_parity_error = 244,
  sensor_parity_error = 245,
  invalid_command = 246,
  framing_error = 247,
  pipeline_state_error = 248,
  stale = 249,
  unwrap_ambiguous = 250,
  zero_not_configured = 251,
  reset_invalidated = 252,
  output_angle_invalid = 253,
  out_of_mechanical_range = 254,
  internal_or_unknown = 255,
};

enum class TorqueError : uint16_t {
  unavailable = 0x800,
  controller_input_invalid = 0x801,
  controller_numeric_error = 0x802,
  reset_invalidated = 0x803,
  limit_config_invalid = 0x804,
  internal_error = 0x805,
  unknown = 0x806,
};

enum class LpsPressureError : uint16_t {
  not_initialized = 2032,
  i2c_timeout = 2033,
  i2c_bus_error = 2034,
  data_not_ready = 2035,
  who_am_i_mismatch = 2036,
  reset_timeout = 2037,
  pressure_overrun = 2038,
  stale = 2039,
  powered_off = 2040,
  below_range = 2041,
  above_range = 2042,
  configuration_error = 2043,
  invalid_sample = 2044,
  internal_error = 2045,
  unknown = 2046,
  unavailable = 2047,
};

enum class LpsTemperatureError : uint8_t {
  not_initialized = 240,
  i2c_timeout = 241,
  i2c_bus_error = 242,
  data_not_ready = 243,
  who_am_i_mismatch = 244,
  reset_timeout = 245,
  temperature_overrun = 246,
  stale = 247,
  powered_off = 248,
  below_range = 249,
  above_range = 250,
  configuration_error = 251,
  invalid_sample = 252,
  internal_error = 253,
  unknown = 254,
  unavailable = 255,
};

enum class AirspeedError : uint8_t {
  negative_differential_pressure = 246,
  above_range = 247,
  static_pressure_invalid = 248,
  ssc_not_initialized = 249,
  ssc_i2c_timeout = 250,
  ssc_i2c_error = 251,
  ssc_stale = 252,
  ssc_command_mode = 253,
  ssc_diagnostic_fault = 254,
  internal_invalid = 255,
};

enum class TimeError : uint8_t {
  pre_liftoff = 0,
  unavailable = 1,
  recovery_in_progress = 2,
  rtc_recovery_failed = 3,
  checkpoint_missing = 4,
  checkpoint_invalid = 5,
  gnss_time_unavailable = 6,
  ground_time_unavailable = 7,
  absolute_time_unavailable = 8,
  time_inconsistent = 9,
  stale = 10,
  overflow = 11,
  persistence_error = 12,
  power_on_reset_unrecoverable = 13,
  internal_error = 14,
  unknown = 15,
};

enum class GnssError : uint16_t {
  unavailable = 0x8000,
  no_fix = 0x8001,
  stale = 0x8002,
  out_of_range = 0x8003,
  invalid_sample = 0x8004,
  receiver_error = 0x8005,
  reference_invalid = 0x8006,
  internal_error = 0x8007,
  unknown = 0x8008,
};

enum class GnssHeightError : uint16_t {
  unavailable = 496,
  no_fix = 497,
  stale = 498,
  out_of_range = 499,
  invalid_sample = 500,
  receiver_error = 501,
  internal_error = 502,
  unknown = 503,
};

enum class ParachuteAngleError : uint8_t {
  not_initialized = 241,
  uart_timeout = 242,
  uart_protocol_error = 243,
  device_error_response = 244,
  configuration_invalid = 245,
  wrong_operating_mode = 246,
  stale = 247,
  position_out_of_range = 248,
  powered_off = 249,
  open_command_failed = 250,
  retry_exhausted = 251,
  position_invalid = 252,
  internal_error = 253,
  unknown = 254,
  unavailable = 255,
};

enum class BatteryError : uint8_t {
  stale = 253,
  adc_error = 254,
  unavailable = 255,
};

[[nodiscard]] uint16_t encodeRoll(double degrees, RollError invalid);
[[nodiscard]] Decoded<uint16_t> decodeRoll(uint16_t raw);
[[nodiscard]] uint16_t encodeRollRate(double degrees_per_second,
                                      RollError invalid);
[[nodiscard]] Decoded<uint16_t> decodeRollRate(uint16_t raw);
[[nodiscard]] uint8_t encodeTiltMagnitude(double degrees, uint8_t error_raw);
[[nodiscard]] Decoded<uint8_t> decodeTiltMagnitude(uint8_t raw);
[[nodiscard]] uint16_t encodeTiltDirection(double degrees);
[[nodiscard]] Decoded<uint16_t> decodeTiltDirection(uint16_t raw);
[[nodiscard]] uint8_t encodeFinAngle(double degrees, FinAngleError invalid);
[[nodiscard]] Decoded<uint8_t> decodeFinAngle(uint8_t raw);
[[nodiscard]] uint16_t encodeFinRate(double degrees_per_second,
                                    FinRateError invalid);
[[nodiscard]] Decoded<uint16_t> decodeFinRate(uint16_t raw);
[[nodiscard]] uint16_t encodeRequestedTorque(double newton_metres,
                                             TorqueError invalid);
[[nodiscard]] Decoded<uint16_t> decodeRequestedTorque(uint16_t raw);
[[nodiscard]] uint16_t encodeLpsPressure(double hectopascals,
                                         LpsPressureError invalid);
[[nodiscard]] Decoded<uint16_t> decodeLpsPressure(uint16_t raw);
[[nodiscard]] uint8_t encodeLpsTemperature(double degrees_celsius,
                                           LpsTemperatureError invalid);
[[nodiscard]] Decoded<uint8_t> decodeLpsTemperature(uint8_t raw);
[[nodiscard]] uint8_t encodeAirspeed(double metres_per_second,
                                     AirspeedError invalid);
[[nodiscard]] Decoded<uint8_t> decodeAirspeed(uint8_t raw);
[[nodiscard]] uint8_t encodeFlightElapsed(double seconds, TimeError invalid);
[[nodiscard]] Decoded<uint8_t> decodeFlightElapsed(uint8_t raw);
[[nodiscard]] uint16_t encodeDescentElapsed(double seconds, TimeError invalid);
[[nodiscard]] Decoded<uint16_t> decodeDescentElapsed(uint16_t raw);
[[nodiscard]] uint16_t encodeRecoveryElapsed(double seconds,
                                             TimeError invalid);
[[nodiscard]] Decoded<uint16_t> decodeRecoveryElapsed(uint16_t raw);
[[nodiscard]] uint16_t encodeGnssOffset(double metres, GnssError invalid);
[[nodiscard]] Decoded<uint16_t> decodeGnssOffset(uint16_t raw);
[[nodiscard]] uint16_t encodeGnssHeight(double metres,
                                        GnssHeightError invalid);
[[nodiscard]] Decoded<uint16_t> decodeGnssHeight(uint16_t raw);
[[nodiscard]] uint8_t encodeParachuteAngle(double degrees,
                                           ParachuteAngleError invalid);
[[nodiscard]] Decoded<uint8_t> decodeParachuteAngle(uint8_t raw);
[[nodiscard]] uint8_t encodeBatteryVoltage(double volts, BatteryError invalid);
[[nodiscard]] Decoded<uint8_t> decodeBatteryVoltage(uint8_t raw);

} // 名前空間 protocol::quantization
