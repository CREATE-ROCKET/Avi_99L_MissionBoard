#pragma once

#include <cstdint>
#include <limits>

#include "bringup/stream_protocol.hpp"
#include "esp_err.h"

namespace bringup::power {

struct AdcReading {
  int32_t raw{std::numeric_limits<int32_t>::min()};
  float pin_voltage_v{std::numeric_limits<float>::quiet_NaN()};
  float source_voltage_v{std::numeric_limits<float>::quiet_NaN()};
  esp_err_t status{ESP_ERR_INVALID_STATE};
  bool raw_valid{false};
  bool calibrated_valid{false};
};

struct PowerSample {
  int64_t timestamp_us{0};
  AdcReading logic{};
  AdcReading motor{};
};

struct AdcChannelSummary {
  uint32_t raw_valid_samples{0};
  uint32_t calibrated_valid_samples{0};
  int32_t raw_min{0};
  int32_t raw_max{0};
  float pin_voltage_min_v{0.0F};
  float pin_voltage_max_v{0.0F};
  float pin_voltage_mean_v{0.0F};
  float source_voltage_min_v{0.0F};
  float source_voltage_max_v{0.0F};
  float source_voltage_mean_v{0.0F};
  esp_err_t calibration_status{ESP_ERR_INVALID_STATE};
};

struct AdcStreamResult {
  uint32_t requested_samples{0};
  uint32_t sample_count{0};
  uint32_t adc_error_count{0};
  uint32_t stream_error_count{0};
  uint32_t dropped_frames{0};
  uint32_t output_errors{0};
  uint32_t deadline_miss_count{0};
  uint64_t duration_us{0};
  uint64_t max_sample_latency_us{0};
  AdcChannelSummary logic{};
  AdcChannelSummary motor{};
};

[[nodiscard]] esp_err_t initialize();
[[nodiscard]] esp_err_t end();
[[nodiscard]] bool initialized();
[[nodiscard]] esp_err_t read(PowerSample &sample);

// ADCレコードはtimestamp、valid flag、logic、motorの順で
// リトルエンディアン直列化する。
[[nodiscard]] esp_err_t adcStream(uint32_t seconds, StreamProtocol &stream,
                                  AdcStreamResult &result);

} // 名前空間 bringup::power
