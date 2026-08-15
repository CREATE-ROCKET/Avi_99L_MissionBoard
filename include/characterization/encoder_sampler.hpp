#pragma once

#include "characterization/characterization_types.hpp"
#include "characterization/platform_compat.hpp"
#include "characterization/spsc_ring.hpp"

#if defined(ESP_PLATFORM)
#include "bringup/encoder_bringup.hpp"
#include "bringup/spi_bringup.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif

#include <atomic>
#include <cstdint>

namespace avi::characterization {

class EncoderSampler {
public:
  EncoderSampler() = default;
  EncoderSampler(const EncoderSampler &) = delete;
  EncoderSampler &operator=(const EncoderSampler &) = delete;

  [[nodiscard]] esp_err_t begin(EncoderRate rate,
                                std::uint64_t epoch_zero_us);
  [[nodiscard]] esp_err_t stop();
  [[nodiscard]] SamplerStatistics statistics() const;
  [[nodiscard]] bool pop(RawEncoderSample &sample) noexcept;
  [[nodiscard]] bool running() const noexcept { return running_.load(); }
  [[nodiscard]] esp_err_t firstError() const noexcept {
    return first_error_.load();
  }
  [[nodiscard]] esp_err_t stopCleanupError() const noexcept {
    return stop_cleanup_error_.load();
  }
#if defined(ESP_PLATFORM)
  void setFailureNotificationTask(TaskHandle_t task) noexcept {
    failure_notification_task_.store(task);
    if (task != nullptr && first_error_.load() != ESP_OK)
      xTaskNotifyGive(task);
  }
#endif

private:
#if defined(ESP_PLATFORM)
  static void timerCallback(void *context);
  static void taskEntry(void *context);
  void taskLoop();
  void rememberFirst(esp_err_t error) noexcept;
  void acknowledgeStop() noexcept;
  [[nodiscard]] std::uint16_t statusFlags(
      const AS5047D::Status &status,
      const AS5047D::ErrorFlags &errors) const noexcept;

  bringup::SpiBringup spi_{};
  bringup::EncoderBringup encoder_{};
  esp_timer_handle_t timer_{nullptr};
  StaticTask_t task_tcb_{};
  StackType_t task_stack_[4096]{};
  TaskHandle_t task_{nullptr};
  StaticSemaphore_t stop_ack_storage_{};
  SemaphoreHandle_t stop_ack_{nullptr};
  std::uint64_t next_scheduled_us_{0};
  std::uint64_t epoch_zero_us_{0};
  std::uint64_t last_sampled_slot_{0};
  bool have_sampled_slot_{false};
  std::uint32_t period_us_{0};
  std::uint8_t samples_per_epoch_{0};
  std::uint64_t generation_{0};
  AS5047D::Status startup_status_{};
  bool pipeline_running_{false};
  std::atomic<bool> stop_waiting_{false};
  std::atomic<TaskHandle_t> failure_notification_task_{nullptr};
  SamplerStatistics statistics_{};
#endif
  SpscRing<RawEncoderSample, 128U> queue_{};
  std::atomic<bool> running_{false};
  std::atomic<esp_err_t> first_error_{ESP_OK};
  std::atomic<esp_err_t> stop_cleanup_error_{ESP_OK};
};

} // 名前空間 avi::characterization
