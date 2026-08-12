#pragma once

#include <atomic>
#include <cstddef>

#include "SPICREATE.h"
#include "esp_err.h"

namespace bringup {

struct SpiHostTestResult {
  esp_err_t begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t duplicate_begin_result{ESP_ERR_NOT_FINISHED};
  esp_err_t end_result{ESP_ERR_NOT_FINISHED};
  esp_err_t second_end_result{ESP_ERR_NOT_FINISHED};
  std::size_t device_count{0};
};

struct SpiTestResult {
  SpiHostTestResult encoder{};
  SpiHostTestResult imu{};
  esp_err_t invalid_config_result{ESP_ERR_NOT_FINISHED};
  bool timeout_api_valid{false};

  [[nodiscard]] bool passed() const;
};

// 2本のSPI busを所有する。deviceはbusより先にendすること。
class SpiBringup {
public:
  SpiBringup() = default;
  SpiBringup(const SpiBringup &) = delete;
  SpiBringup &operator=(const SpiBringup &) = delete;

  [[nodiscard]] esp_err_t begin();
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t test(SpiTestResult &result);

  [[nodiscard]] SPICREATE *encoderBus();
  [[nodiscard]] SPICREATE *imuBus();
  [[nodiscard]] bool encoderBusInitialized() const;
  [[nodiscard]] bool imuBusInitialized() const;
  [[nodiscard]] bool busy() const { return busy_.load(); }

private:
  [[nodiscard]] esp_err_t beginEncoderBus();
  [[nodiscard]] esp_err_t beginImuBus();
  [[nodiscard]] esp_err_t endEncoderBus();
  [[nodiscard]] esp_err_t endImuBus();

  SPICREATE encoder_bus_{};
  SPICREATE imu_bus_{};
  std::atomic<bool> busy_{false};
};

} // 名前空間 bringup
