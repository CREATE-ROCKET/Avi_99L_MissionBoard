#pragma once

#include <atomic>
#include <cstdint>

#include "AS5047D.h"
#include "bringup/encoder_bringup.hpp"
#include "bringup/imu_bringup.hpp"
#include "bringup/spi_bringup.hpp"
#include "bringup/stream_protocol.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace bringup {

struct MotorTestResult {
  uint32_t requested_samples{0};
  uint32_t sample_count{0};
  uint32_t encoder_error_count{0};
  uint32_t imu_error_count{0};
  uint32_t adc_error_count{0};
  uint32_t stream_error_count{0};
  uint32_t deadline_miss_count{0};
  uint32_t dropped_frames{0};
  uint32_t output_errors{0};
  uint16_t maximum_lost_packets{0};
  uint32_t max_loop_latency_us{0};
  float maximum_abs_speed_rad_s{0.0F};
  float encoder_delta_rad{0.0F};
  bool prbs_ten_percent_executed{false};
  AS5047D::Status initial_encoder_status{};
  AS5047D::Status final_encoder_status{};
  AS5047D::ErrorFlags final_encoder_errors{};
};

class MotorBringup {
public:
  enum class TestKind : uint8_t {
    polarity,
    step,
    prbs,
    coast,
    brake,
    combined,
  };

  MotorBringup(SpiBringup &spi, EncoderBringup &encoder, ImuBringup &imu,
               StreamProtocol &stream)
      : spi_(spi), encoder_(encoder), imu_(imu), stream_(stream) {}

  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t arm();
  [[nodiscard]] esp_err_t disarm();
  [[nodiscard]] esp_err_t requestDisarm();

  [[nodiscard]] esp_err_t polarity(MotorTestResult &result);
  [[nodiscard]] esp_err_t step(MotorTestResult &result);
  [[nodiscard]] esp_err_t prbs(MotorTestResult &result);
  [[nodiscard]] esp_err_t coastTest(MotorTestResult &result);
  [[nodiscard]] esp_err_t brakeTest(MotorTestResult &result);
  [[nodiscard]] esp_err_t combinedMotorImuTest(MotorTestResult &result);

  [[nodiscard]] bool initialized() const { return initialized_.load(); }
  [[nodiscard]] bool armed() const { return armed_.load(); }
  [[nodiscard]] bool busy() const { return busy_.load(); }
  [[nodiscard]] uint32_t actualPwmFrequencyHz() const;

private:
  [[nodiscard]] esp_err_t run(TestKind kind, MotorTestResult &result);
  [[nodiscard]] esp_err_t setDuty(float signed_duty);
  [[nodiscard]] esp_err_t coast();
  [[nodiscard]] esp_err_t coastUnlocked(bool force_channels = false);
  [[nodiscard]] esp_err_t emergencyCoast();
  [[nodiscard]] esp_err_t brake();

  SpiBringup &spi_;
  EncoderBringup &encoder_;
  ImuBringup &imu_;
  StreamProtocol &stream_;
  std::atomic<bool> armed_{false};
  std::atomic<bool> busy_{false};
  std::atomic<bool> initialized_{false};
  StaticSemaphore_t output_lock_storage_{};
  SemaphoreHandle_t output_lock_{nullptr};
  std::atomic<bool> timer_configured_{false};
  std::atomic<bool> in1_configured_{false};
  std::atomic<bool> in2_configured_{false};
};

} // 名前空間 bringup
