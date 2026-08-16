#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace runtime::flight_log {

inline constexpr uint8_t kSchemaVersion = 2;
inline constexpr std::size_t kSerializedRecordBytes = 192;
inline constexpr uint32_t kInvalidDropCounter = 0xFFFF'FFFFU;
inline constexpr uint16_t kUnknownEncoderZeroCount = 0xFFFFU;

struct Sample {
  uint64_t monotonic_us{};
  uint64_t flight_elapsed_us{};
  uint32_t flight_epoch{};
  uint32_t sd_drop_count{};
  uint32_t flash_drop_count{};
  uint16_t flight_status{};
  uint8_t state{};
  uint8_t config_flags{};
  uint8_t fin_mode{};
  uint8_t para_mode{};
  uint8_t lps_temperature_raw{};
  uint8_t airspeed_raw{};
  uint8_t fin_angle_raw{};
  uint16_t lps_pressure_raw{};
  uint16_t roll_raw{};
  uint16_t roll_rate_raw{};
  uint16_t fin_rate_raw{};
  uint16_t requested_torque_raw{};
  uint16_t control_roll_reference_raw{};
  uint16_t roll_deviation_raw{};
  uint8_t control_roll_flags{};
  uint8_t reference_capture_event_sequence{};
  uint8_t gain_clamp_flags{};
  bool lps_valid{};
  bool airspeed_valid{};
  bool deployment_power_cutoff{};
  bool control_reference_valid{};
  bool fin_zero_configured{};
  uint16_t encoder_zero_count{kUnknownEncoderZeroCount};
  uint64_t reference_capture_tick{};
  uint64_t reference_estimator_timestamp_us{};
  float roll_estimate_rad{};
  float control_reference_rad{};
  float roll_deviation_rad{};
  float static_pressure_pa{};
  float ssc_temperature_celsius{};
  float airspeed_mps{};
  float pitot_coefficient_assumed{};
  float pitot_coefficient_diagnostic_min{};
  float pitot_coefficient_diagnostic_max{};
  float measured_bidirectional_span_rad{};
};

using SerializedRecord = std::array<uint8_t, kSerializedRecordBytes>;

[[nodiscard]] uint32_t crc32(const uint8_t *data, std::size_t size);
[[nodiscard]] SerializedRecord serialize(const Sample &sample);
[[nodiscard]] bool validate(const SerializedRecord &record);
[[nodiscard]] bool erased(const SerializedRecord &record);

} // 名前空間 runtime::flight_log
