#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "ICM42688.h"
#include "bringup/spi_bringup.hpp"
#include "bringup/stream_protocol.hpp"
#include "esp_err.h"

namespace bringup {

struct ImuSample {
  uint64_t host_timestamp_us{0};
  uint64_t sensor_timestamp_us{0};
  uint16_t timestamp_ticks{0};
  std::array<int16_t, 3> acceleration_raw{};
  std::array<int16_t, 3> angular_velocity_raw{};
  int16_t temperature_raw{0};
  std::array<float, 3> acceleration_g{};
  std::array<float, 3> angular_velocity_dps{};
  float temperature_celsius{0.0F};
  uint32_t read_latency_us{0};
  bool acceleration_valid{false};
  bool angular_velocity_valid{false};
  bool temperature_valid{false};
  bool accel_odr_changed{false};
  bool gyro_odr_changed{false};
};

struct ImuSelfTestResult {
  esp_err_t begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t who_am_i_result{ESP_ERR_NOT_FINISHED};
  esp_err_t self_test_result{ESP_ERR_NOT_FINISHED};
  esp_err_t end_result{ESP_ERR_NOT_FINISHED};
  uint8_t who_am_i{0};
  ICM42688::SelfTestResult detail{};

  [[nodiscard]] bool passed() const;
};

struct ImuStreamResult {
  uint32_t requested_samples{0};
  uint32_t sample_count{0};
  uint32_t batch_count{0};
  uint32_t driver_error_count{0};
  uint32_t stream_error_count{0};
  uint32_t dropped_frames{0};
  uint32_t output_errors{0};
  uint32_t invalid_acceleration_count{0};
  uint32_t invalid_angular_velocity_count{0};
  uint32_t invalid_temperature_count{0};
  uint32_t accel_odr_change_count{0};
  uint32_t gyro_odr_change_count{0};
  uint32_t timestamp_nonmonotonic_count{0};
  uint32_t fifo_full_count{0};
  uint32_t fifo_fault_count{0};
  uint16_t maximum_lost_packets{0};
  uint32_t max_read_latency_us{0};
  ICM42688::FifoStatus final_fifo_status{};
  esp_err_t begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t stream_begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t final_status_result{ESP_ERR_NOT_FINISHED};
  esp_err_t stream_finish_result{ESP_ERR_NOT_FINISHED};
  esp_err_t end_result{ESP_ERR_NOT_FINISHED};

  [[nodiscard]] bool passed() const;
};

class ImuBringup {
public:
  static constexpr std::size_t kMaximumFifoBatch = 16;

  ImuBringup() = default;
  ImuBringup(const ImuBringup &) = delete;
  ImuBringup &operator=(const ImuBringup &) = delete;

  [[nodiscard]] esp_err_t begin(SpiBringup &spi, bool fifo_enabled);
  [[nodiscard]] esp_err_t read(ImuSample &sample);
  [[nodiscard]] esp_err_t waitFifo(uint32_t timeout_ms);
  [[nodiscard]] esp_err_t getFifoStatus(ICM42688::FifoStatus &status);
  [[nodiscard]] esp_err_t readFifo(ImuSample *samples, std::size_t capacity,
                                   std::size_t &count);
  [[nodiscard]] esp_err_t end();

  [[nodiscard]] esp_err_t selfTest(SpiBringup &spi,
                                   ImuSelfTestResult &result);
  [[nodiscard]] esp_err_t stream(SpiBringup &spi, uint32_t seconds,
                                 StreamProtocol &protocol,
                                 ImuStreamResult &result);
  [[nodiscard]] esp_err_t staticCapture(SpiBringup &spi, uint32_t seconds,
                                        StreamProtocol &protocol,
                                        ImuStreamResult &result);

  [[nodiscard]] bool initialized() const { return imu_.initialized(); }
  [[nodiscard]] bool fifoEnabled() const { return fifo_enabled_; }
  [[nodiscard]] bool busy() const { return busy_.load(); }

private:
  [[nodiscard]] esp_err_t beginImpl(SpiBringup &spi, bool fifo_enabled);
  [[nodiscard]] esp_err_t readImpl(ImuSample &sample);
  [[nodiscard]] esp_err_t readFifoImpl(ImuSample *samples,
                                       std::size_t capacity,
                                       std::size_t &count);
  [[nodiscard]] esp_err_t endImpl();
  [[nodiscard]] esp_err_t captureImpl(SpiBringup &spi, uint32_t seconds,
                                      StreamProtocol &protocol,
                                      ImuStreamResult &result);

  ICM42688 imu_{};
  bool fifo_enabled_{false};
  uint64_t fifo_timestamp_us_{0};
  uint8_t fifo_timestamp_remainder_{0};
  std::atomic<bool> busy_{false};
};

} // 名前空間 bringup
