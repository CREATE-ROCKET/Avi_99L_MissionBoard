#pragma once

#include "characterization/characterization_types.hpp"
#include "characterization/log_format_v5.hpp"
#include "characterization/platform_compat.hpp"

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>

namespace avi::characterization {

class LogWriterV5 {
public:
  static constexpr std::size_t kQueueDepth = 128U;
  static constexpr std::size_t kBatchRecords = 16U;

  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] esp_err_t open(const LogHeaderV5 &header,
                               const char *base_name);
  [[nodiscard]] esp_err_t enqueue(const ImmutableLogRecord &record);
  [[nodiscard]] esp_err_t drainAndSync();
  [[nodiscard]] esp_err_t close(const LogFooterV5 &footer);
  [[nodiscard]] esp_err_t abortAndClose(LogFooterV5 footer = {});
#if defined(ESP_PLATFORM)
  void setFailureNotificationTask(TaskHandle_t task) noexcept {
    failure_notification_task_.store(task);
    if (task != nullptr && first_error_.load() != ESP_OK)
      xTaskNotifyGive(task);
  }
#endif

  [[nodiscard]] bool open() const noexcept { return file_ != nullptr; }
  [[nodiscard]] esp_err_t firstError() const noexcept {
    return first_error_.load();
  }
  [[nodiscard]] std::uint64_t recordsWritten() const noexcept {
    return records_written_.load();
  }
  [[nodiscard]] std::uint64_t writerQueueOverflows() const noexcept {
    return queue_overflows_.load();
  }
  [[nodiscard]] const char *currentPath() const noexcept {
    return current_path_.data();
  }

private:
#if defined(ESP_PLATFORM)
  enum class ControlKind : std::uint8_t { Sync = 0, Finalize = 1 };
  struct ControlRequest {
    ControlKind kind{ControlKind::Sync};
    LogFooterV5 footer{};
  };

  static void taskEntry(void *context);
  void taskLoop();
  void rememberFirst(esp_err_t error) noexcept;
  void processRecordBatch(ImmutableLogRecord first_record);
  void processControl(const ControlRequest &request);
  [[nodiscard]] esp_err_t submitControl(const ControlRequest &request);

  StaticQueue_t queue_control_{};
  std::array<std::uint8_t,
             kQueueDepth * sizeof(ImmutableLogRecord)>
      queue_storage_{};
  QueueHandle_t queue_{nullptr};
  StaticQueue_t control_queue_control_{};
  std::array<std::uint8_t, sizeof(ControlRequest)>
      control_queue_storage_{};
  QueueHandle_t control_queue_{nullptr};
  StaticSemaphore_t control_ack_storage_{};
  SemaphoreHandle_t control_ack_{nullptr};
  StaticTask_t task_tcb_{};
  StackType_t task_stack_[4096]{};
  TaskHandle_t task_{nullptr};
  std::array<ImmutableLogRecord, kBatchRecords> batch_{};
  std::array<std::uint8_t,
             wire_v5::kRecordBytes * kBatchRecords>
      encoded_batch_{};
  std::atomic<bool> control_pending_{false};
  std::atomic<esp_err_t> control_result_{ESP_OK};
  std::atomic<TaskHandle_t> failure_notification_task_{nullptr};
  std::atomic<std::uint32_t> queue_high_water_{0U};
  std::atomic<std::uint32_t> max_batch_records_{0U};
  std::atomic<std::uint32_t> max_validate_us_{0U};
  std::atomic<std::uint32_t> max_encode_us_{0U};
  std::atomic<std::uint32_t> max_fwrite_us_{0U};
#endif
  FILE *file_{nullptr};
  std::array<char, 128> current_path_{};
  std::atomic<bool> accepting_{false};
  std::atomic<bool> synced_{false};
  std::atomic<esp_err_t> first_error_{ESP_OK};
  std::atomic<std::uint64_t> records_written_{0U};
  std::atomic<std::uint64_t> first_sequence_{0U};
  std::atomic<std::uint64_t> last_sequence_{0U};
  std::atomic<std::uint64_t> queue_overflows_{0U};
  std::uint32_t file_crc32_{0U};
  bool initialized_{false};
};

} // 名前空間 avi::characterization
