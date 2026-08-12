#include "bringup/spi_bringup.hpp"

#include "avi_esp_libs/timeout.h"
#include "config/board_config.hpp"

namespace bringup {
namespace {

class ExclusiveGuard {
public:
  explicit ExclusiveGuard(std::atomic<bool> &busy) : busy_(busy) {
    bool expected = false;
    acquired_ = busy_.compare_exchange_strong(expected, true);
  }
  ~ExclusiveGuard() {
    if (acquired_)
      busy_.store(false);
  }
  [[nodiscard]] bool acquired() const { return acquired_; }

private:
  std::atomic<bool> &busy_;
  bool acquired_{false};
};

void rememberFirst(esp_err_t operation, esp_err_t &first_error) {
  if (first_error == ESP_OK && operation != ESP_OK)
    first_error = operation;
}

} // 無名名前空間

bool SpiTestResult::passed() const {
  const auto hostPassed = [](const SpiHostTestResult &host) {
    return host.begin_result == ESP_OK && host.device_count == 0 &&
           host.duplicate_begin_result == ESP_ERR_INVALID_STATE &&
           host.end_result == ESP_OK &&
           host.second_end_result == ESP_ERR_INVALID_STATE;
  };
  return hostPassed(encoder) && hostPassed(imu) &&
         invalid_config_result == ESP_ERR_INVALID_ARG && timeout_api_valid;
}

esp_err_t SpiBringup::beginEncoderBus() {
  SPICREATE::Config config{};
  config.host = board::kEncoderSpiHost;
  config.sck = board::kEncoderSclk;
  config.miso = board::kEncoderMiso;
  config.mosi = board::kEncoderMosi;
  config.max_transfer_size = SPI_MAX_DMA_LEN;
  // bring-up shellがbus所有権を直列化するため、競合時は待たず明示失敗する。
  config.transaction_timeout = avi::Timeout::noWait();
  return encoder_bus_.begin(config);
}

esp_err_t SpiBringup::beginImuBus() {
  SPICREATE::Config config{};
  config.host = board::kImuSpiHost;
  config.sck = board::kImuSclk;
  config.miso = board::kImuMiso;
  config.mosi = board::kImuMosi;
  config.max_transfer_size = SPI_MAX_DMA_LEN;
  // bring-up shellがbus所有権を直列化するため、競合時は待たず明示失敗する。
  config.transaction_timeout = avi::Timeout::noWait();
  return imu_bus_.begin(config);
}

esp_err_t SpiBringup::endEncoderBus() { return encoder_bus_.end(); }

esp_err_t SpiBringup::endImuBus() { return imu_bus_.end(); }

esp_err_t SpiBringup::begin() {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  if (encoder_bus_.initialized() || imu_bus_.initialized())
    return ESP_ERR_INVALID_STATE;

  esp_err_t result = beginEncoderBus();
  if (result != ESP_OK)
    return result;
  result = beginImuBus();
  if (result != ESP_OK) {
    const esp_err_t cleanup = endEncoderBus();
    return cleanup == ESP_OK ? result : cleanup;
  }
  return ESP_OK;
}

esp_err_t SpiBringup::end() {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  if (!encoder_bus_.initialized() && !imu_bus_.initialized())
    return ESP_ERR_INVALID_STATE;

  esp_err_t first_error = ESP_OK;
  if (imu_bus_.initialized())
    rememberFirst(endImuBus(), first_error);
  if (encoder_bus_.initialized())
    rememberFirst(endEncoderBus(), first_error);
  return first_error;
}

esp_err_t SpiBringup::test(SpiTestResult &result) {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  if (encoder_bus_.initialized() || imu_bus_.initialized())
    return ESP_ERR_INVALID_STATE;

  result = {};
  result.encoder.begin_result = beginEncoderBus();
  if (result.encoder.begin_result == ESP_OK) {
    result.encoder.device_count = encoder_bus_.deviceCount();
    result.encoder.duplicate_begin_result = beginEncoderBus();
  }

  result.imu.begin_result = beginImuBus();
  if (result.imu.begin_result == ESP_OK) {
    result.imu.device_count = imu_bus_.deviceCount();
    result.imu.duplicate_begin_result = beginImuBus();
  }

  if (result.imu.begin_result == ESP_OK) {
    result.imu.end_result = endImuBus();
    if (result.imu.end_result == ESP_OK)
      result.imu.second_end_result = endImuBus();
  }
  if (result.encoder.begin_result == ESP_OK) {
    result.encoder.end_result = endEncoderBus();
    if (result.encoder.end_result == ESP_OK)
      result.encoder.second_end_result = endEncoderBus();
  }

  SPICREATE invalid_bus;
  SPICREATE::Config invalid_config{};
  invalid_config.host = board::kEncoderSpiHost;
  invalid_config.sck = GPIO_NUM_NC;
  invalid_config.miso = board::kEncoderMiso;
  invalid_config.mosi = board::kEncoderMosi;
  invalid_config.transaction_timeout = avi::Timeout::noWait();
  result.invalid_config_result = invalid_bus.begin(invalid_config);

  uint64_t milliseconds = 0;
  result.timeout_api_valid = avi::Timeout::noWait().isNoWait() &&
                             avi::Timeout::milliseconds(2).isFinite() &&
                             avi::Timeout::milliseconds(2).millisecondsValue(
                                 milliseconds) &&
                             milliseconds == 2;

  esp_err_t first_error = ESP_OK;
  rememberFirst(result.encoder.begin_result, first_error);
  if (result.encoder.duplicate_begin_result != ESP_ERR_INVALID_STATE)
    rememberFirst(ESP_FAIL, first_error);
  rememberFirst(result.imu.begin_result, first_error);
  if (result.imu.duplicate_begin_result != ESP_ERR_INVALID_STATE)
    rememberFirst(ESP_FAIL, first_error);
  rememberFirst(result.imu.end_result, first_error);
  if (result.imu.second_end_result != ESP_ERR_INVALID_STATE)
    rememberFirst(ESP_FAIL, first_error);
  rememberFirst(result.encoder.end_result, first_error);
  if (result.encoder.second_end_result != ESP_ERR_INVALID_STATE)
    rememberFirst(ESP_FAIL, first_error);
  if (result.invalid_config_result != ESP_ERR_INVALID_ARG ||
      !result.timeout_api_valid)
    rememberFirst(ESP_FAIL, first_error);
  return first_error;
}

SPICREATE *SpiBringup::encoderBus() {
  return encoder_bus_.initialized() ? &encoder_bus_ : nullptr;
}

SPICREATE *SpiBringup::imuBus() {
  return imu_bus_.initialized() ? &imu_bus_ : nullptr;
}

bool SpiBringup::encoderBusInitialized() const {
  return encoder_bus_.initialized();
}

bool SpiBringup::imuBusInitialized() const { return imu_bus_.initialized(); }

} // 名前空間 bringup
