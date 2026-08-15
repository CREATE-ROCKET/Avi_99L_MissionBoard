#pragma once

#include "characterization/characterization_types.hpp"
#include "characterization/platform_compat.hpp"
#include "characterization/spsc_ring.hpp"

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif

#include <atomic>

namespace avi::characterization {

class PowerSampler {
public:
  [[nodiscard]] esp_err_t begin();
  [[nodiscard]] esp_err_t stop() noexcept;
  [[nodiscard]] bool latest(std::uint64_t snapshot_us,
                            PowerEvidence &evidence) noexcept;
  [[nodiscard]] esp_err_t firstError() const noexcept {
    return first_error_.load();
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
  static void taskEntry(void *context);
  void taskLoop();
  void rememberFirst(esp_err_t error) noexcept;
  StaticTask_t task_tcb_{};
  StackType_t task_stack_[3072]{};
  TaskHandle_t task_{nullptr};
  StaticSemaphore_t stop_ack_storage_{};
  SemaphoreHandle_t stop_ack_{nullptr};
  std::atomic<TaskHandle_t> failure_notification_task_{nullptr};
#endif
  SpscRing<PowerEvidence, 32U> queue_{};
  PowerEvidence latest_{};
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_waiting_{false};
  std::atomic<esp_err_t> first_error_{ESP_OK};
  bool have_latest_{false};
};

} // 名前空間 avi::characterization
