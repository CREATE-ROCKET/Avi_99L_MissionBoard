#pragma once

#include <atomic>
#include <cstdint>

#include "AS5047D.h"
#include "bringup/spi_bringup.hpp"
#include "bringup/stream_protocol.hpp"
#include "esp_err.h"

namespace bringup {

struct EncoderSample {
  uint64_t host_timestamp_us{0};
  uint16_t angle_raw{0};
  float angle_degrees{0.0F};
  float angle_radians{0.0F};
  uint32_t read_latency_us{0};
  bool valid{false};
};

struct EncoderTestResult {
  esp_err_t begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t status_result{ESP_ERR_NOT_FINISHED};
  esp_err_t read_result{ESP_ERR_NOT_FINISHED};
  esp_err_t pipeline_start_result{ESP_ERR_NOT_FINISHED};
  esp_err_t pipeline_read_result{ESP_ERR_NOT_FINISHED};
  esp_err_t pipeline_stop_result{ESP_ERR_NOT_FINISHED};
  esp_err_t error_flags_result{ESP_ERR_NOT_FINISHED};
  esp_err_t end_result{ESP_ERR_NOT_FINISHED};
  AS5047D::Status status{};
  AS5047D::ErrorFlags begin_error_flags{};
  AS5047D::ErrorFlags error_flags{};
  EncoderSample direct_sample{};
  EncoderSample pipelined_sample{};

  [[nodiscard]] bool passed() const;
};

struct EncoderStreamResult {
  uint32_t requested_samples{0};
  uint32_t sample_count{0};
  uint32_t driver_error_count{0};
  uint32_t parity_error_count{0};
  uint32_t sensor_error_count{0};
  uint32_t pipeline_restart_count{0};
  uint32_t stream_error_count{0};
  uint32_t dropped_frames{0};
  uint32_t output_errors{0};
  uint32_t deadline_miss_count{0};
  uint32_t boundary_crossing_count{0};
  uint32_t max_read_latency_us{0};
  uint16_t minimum_angle_raw{0};
  uint16_t maximum_angle_raw{0};
  AS5047D::Status final_status{};
  AS5047D::ErrorFlags final_error_flags{};
  esp_err_t begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t stream_begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t pipeline_start_result{ESP_ERR_NOT_FINISHED};
  esp_err_t pipeline_stop_result{ESP_ERR_NOT_FINISHED};
  esp_err_t final_status_result{ESP_ERR_NOT_FINISHED};
  esp_err_t final_error_flags_result{ESP_ERR_NOT_FINISHED};
  esp_err_t stream_finish_result{ESP_ERR_NOT_FINISHED};
  esp_err_t end_result{ESP_ERR_NOT_FINISHED};

  [[nodiscard]] bool passed() const;
};

class EncoderBringup {
public:
  EncoderBringup() = default;
  EncoderBringup(const EncoderBringup &) = delete;
  EncoderBringup &operator=(const EncoderBringup &) = delete;

  [[nodiscard]] esp_err_t begin(SpiBringup &spi);
  [[nodiscard]] esp_err_t read(EncoderSample &sample);
  [[nodiscard]] esp_err_t startPipelinedRead();
  [[nodiscard]] esp_err_t readPipelined(EncoderSample &sample);
  [[nodiscard]] esp_err_t stopPipelinedRead();
  [[nodiscard]] esp_err_t getStatus(AS5047D::Status &status);
  [[nodiscard]] esp_err_t
  readAndClearErrorFlags(AS5047D::ErrorFlags &flags);
  [[nodiscard]] AS5047D::ErrorFlags lastErrorFlags() const {
    return encoder_.lastErrorFlags();
  }
  [[nodiscard]] esp_err_t end();

  [[nodiscard]] esp_err_t test(SpiBringup &spi, EncoderTestResult &result);
  [[nodiscard]] esp_err_t stream(SpiBringup &spi, uint32_t seconds,
                                 StreamProtocol &protocol,
                                 EncoderStreamResult &result);

  [[nodiscard]] bool initialized() const { return encoder_.initialized(); }
  [[nodiscard]] bool busy() const { return busy_.load(); }

private:
  [[nodiscard]] esp_err_t beginImpl(SpiBringup &spi);
  [[nodiscard]] esp_err_t readImpl(EncoderSample &sample, bool pipelined);
  [[nodiscard]] esp_err_t endImpl();

  AS5047D encoder_{};
  std::atomic<bool> busy_{false};
};

} // 名前空間 bringup
