#include "runtime/flight_log.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace runtime::flight_log {
namespace {

constexpr std::array<uint8_t, 4> kMagic{'9', '9', 'L', 'G'};
constexpr uint8_t kValidityLps = 1U << 0U;
constexpr uint8_t kValidityAirspeed = 1U << 1U;
constexpr uint8_t kValidityPowerCutoff = 1U << 2U;
constexpr uint8_t kValidityControlReference = 1U << 3U;
constexpr uint8_t kValidityFinZero = 1U << 4U;
// schema headerのrecord lengthは1 byteなので、v2では0を256 byteのcodeとする。
constexpr uint8_t kRecordLengthCode = 0U;
constexpr std::size_t kCrcOffset = 252;
static_assert(kSerializedRecordBytes == 256U);

void putU16(SerializedRecord &out, std::size_t offset, uint16_t value) {
  out[offset] = static_cast<uint8_t>(value);
  out[offset + 1] = static_cast<uint8_t>(value >> 8U);
}

void putU32(SerializedRecord &out, std::size_t offset, uint32_t value) {
  for (std::size_t index = 0; index < 4; ++index)
    out[offset + index] = static_cast<uint8_t>(value >> (index * 8U));
}

void putU64(SerializedRecord &out, std::size_t offset, uint64_t value) {
  for (std::size_t index = 0; index < 8; ++index)
    out[offset + index] = static_cast<uint8_t>(value >> (index * 8U));
}

void putFloat(SerializedRecord &out, std::size_t offset, float value) {
  uint32_t raw = 0;
  static_assert(sizeof(raw) == sizeof(value));
  std::memcpy(&raw, &value, sizeof(raw));
  putU32(out, offset, raw);
}

uint32_t getU32(const SerializedRecord &record, std::size_t offset) {
  return static_cast<uint32_t>(record[offset]) |
         static_cast<uint32_t>(record[offset + 1]) << 8U |
         static_cast<uint32_t>(record[offset + 2]) << 16U |
         static_cast<uint32_t>(record[offset + 3]) << 24U;
}

} // namespace

uint32_t crc32(const uint8_t *data, std::size_t size) {
  uint32_t crc = 0xFFFF'FFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1U) ^ ((crc & 1U) != 0 ? 0xEDB8'8320U : 0U);
  }
  return ~crc;
}

SerializedRecord serialize(const Sample &sample) {
  SerializedRecord out{};
  std::copy(kMagic.begin(), kMagic.end(), out.begin());
  out[4] = kSchemaVersion;
  out[5] = kRecordLengthCode;
  out[6] = sample.state;
  out[7] = (sample.lps_valid ? kValidityLps : 0U) |
           (sample.airspeed_valid ? kValidityAirspeed : 0U) |
           (sample.deployment_power_cutoff ? kValidityPowerCutoff : 0U) |
           (sample.control_reference_valid ? kValidityControlReference : 0U) |
           (sample.fin_zero_configured ? kValidityFinZero : 0U);
  putU64(out, 8, sample.monotonic_us);
  putU64(out, 16, sample.flight_elapsed_us);
  putU32(out, 24, sample.flight_epoch);
  putU32(out, 28, sample.sd_drop_count);
  putU32(out, 32, sample.flash_drop_count);
  putU16(out, 36, sample.flight_status);
  out[38] = sample.config_flags;
  out[39] = sample.fin_mode;
  out[40] = sample.para_mode;
  out[41] = sample.lps_temperature_raw;
  out[42] = sample.airspeed_raw;
  out[43] = sample.fin_angle_raw;
  putU16(out, 44, sample.lps_pressure_raw);
  putU16(out, 46, sample.roll_raw);
  putU16(out, 48, sample.roll_rate_raw);
  putU16(out, 50, sample.fin_rate_raw);
  putU16(out, 52, sample.requested_torque_raw);
  putU16(out, 54, sample.control_roll_reference_raw);
  putU16(out, 56, sample.roll_deviation_raw);
  out[58] = sample.control_roll_flags;
  out[59] = sample.reference_capture_event_sequence;
  out[60] = sample.gain_clamp_flags;
  out[61] = sample.fin_zero_configured ? 1U : 0U;
  putU16(out, 62, sample.encoder_zero_count);
  putU64(out, 64, sample.reference_capture_tick);
  putU64(out, 72, sample.reference_estimator_timestamp_us);
  putFloat(out, 80, sample.roll_estimate_rad);
  putFloat(out, 84, sample.control_reference_rad);
  putFloat(out, 88, sample.roll_deviation_rad);
  putFloat(out, 92, sample.static_pressure_pa);
  putFloat(out, 96, sample.ssc_temperature_celsius);
  putFloat(out, 100, sample.airspeed_mps);
  putFloat(out, 104, sample.pitot_coefficient_assumed);
  putFloat(out, 108, sample.pitot_coefficient_diagnostic_min);
  putFloat(out, 112, sample.pitot_coefficient_diagnostic_max);
  putFloat(out, 116, sample.measured_bidirectional_span_rad);

  putU64(out, 120, sample.fin_zero_configured_timestamp_us);
  putU32(out, 128, sample.fin_zero_flight_epoch);
  out[132] = static_cast<uint8_t>(sample.fin_zero_approach_direction);
  out[133] = static_cast<uint8_t>(sample.fin_zero_calibration_method);
  out[134] =
      static_cast<uint8_t>(sample.fin_zero_ground_verification_status);
  out[135] = sample.encoder_diagnostic_flags;
  putU64(out, 136, sample.encoder_sample_timestamp_us);
  putU32(out, 144, sample.encoder_read_latency_us);
  putU32(out, 148, sample.encoder_sample_age_us);
  putU32(out, 152, sample.encoder_reconnect_count);
  putU32(out, 156, sample.encoder_error_count);
  // 160..251 reserved for schema v2 extension and kept zero.
  putU32(out, kCrcOffset, crc32(out.data(), kCrcOffset));
  return out;
}

bool validate(const SerializedRecord &record) {
  if (!std::equal(kMagic.begin(), kMagic.end(), record.begin()) ||
      record[4] != kSchemaVersion || record[5] != kRecordLengthCode)
    return false;
  return getU32(record, kCrcOffset) == crc32(record.data(), kCrcOffset);
}

bool erased(const SerializedRecord &record) {
  return std::all_of(record.begin(), record.end(),
                     [](uint8_t value) { return value == 0xFFU; });
}

} // namespace runtime::flight_log
