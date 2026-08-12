#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>

#include "bringup/imu_bringup.hpp"
#include "bringup/spi_bringup.hpp"
#include "bringup/stream_protocol.hpp"
#include "esp_err.h"

namespace bringup {

struct CalibrationAttempt {
  uint32_t id{0};
  uint64_t completed_at_us{0};
  uint32_t imu_samples{0};
  uint32_t ssc_samples{0};
  uint32_t error_count{0};
  uint16_t maximum_lost_packets{0};
  std::array<float, 3> gyro_bias_dps{
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()};
  std::array<float, 3> gravity_sensor_g{
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()};
  float acceleration_norm_g{std::numeric_limits<float>::quiet_NaN()};
  float launcher_tilt_deg{std::numeric_limits<float>::quiet_NaN()};
  float ssc_zero_pa{std::numeric_limits<float>::quiet_NaN()};
  bool gyro_valid{false};
  bool gravity_valid{false};
  bool launcher_tilt_valid{false};
  bool ssc_valid{false};
  esp_err_t imu_status{ESP_ERR_INVALID_STATE};
  esp_err_t ssc_status{ESP_ERR_NOT_FOUND};
};

struct CalibrationRepeatResult {
  uint32_t requested_attempts{0};
  uint32_t completed_attempts{0};
  uint32_t successful_imu_attempts{0};
  uint32_t successful_ssc_attempts{0};
  uint32_t stream_error_count{0};
  uint32_t dropped_frames{0};
  uint32_t output_errors{0};
};

class CalibrationBringup {
public:
  CalibrationBringup(SpiBringup &spi, ImuBringup &imu,
                     StreamProtocol &stream)
      : spi_(spi), imu_(imu), stream_(stream) {}

  [[nodiscard]] esp_err_t calibrate(CalibrationAttempt &result);
  [[nodiscard]] esp_err_t repeat(uint32_t count,
                                 CalibrationRepeatResult &result);
  [[nodiscard]] bool hasLatest() const { return has_latest_; }
  [[nodiscard]] const CalibrationAttempt &latest() const { return latest_; }

private:
  SpiBringup &spi_;
  ImuBringup &imu_;
  StreamProtocol &stream_;
  CalibrationAttempt latest_{};
  uint64_t next_id_{1};
  std::atomic<bool> busy_{false};
  bool has_latest_{false};
};

} // 名前空間 bringup
