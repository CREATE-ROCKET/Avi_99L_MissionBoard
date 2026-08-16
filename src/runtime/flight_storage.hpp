#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "runtime/flight_log.hpp"
#include "sdmmc_cmd.h"

namespace runtime::flight_storage {

class InternalFlashLog {
public:
  [[nodiscard]] esp_err_t prepareForFlight();
  [[nodiscard]] esp_err_t openExisting();
  [[nodiscard]] esp_err_t append(const flight_log::SerializedRecord &record);
  [[nodiscard]] esp_err_t read(uint32_t offset, uint8_t *destination,
                               std::size_t requested,
                               std::size_t &read_size) const;
  [[nodiscard]] esp_err_t readRaw(uint32_t offset, uint8_t *destination,
                                  std::size_t requested,
                                  std::size_t &read_size) const;
  [[nodiscard]] esp_err_t erase();
  [[nodiscard]] uint32_t size() const { return write_offset_; }
  [[nodiscard]] uint32_t capacity() const {
    return partition_ == nullptr ? 0U : partition_->size;
  }
  [[nodiscard]] bool ready() const { return partition_ != nullptr; }
  [[nodiscard]] bool hasData() const { return has_data_; }

private:
  [[nodiscard]] esp_err_t locatePartition();
  const esp_partition_t *partition_{};
  uint32_t write_offset_{};
  bool has_data_{};
  std::array<uint8_t, 4096> scan_buffer_{};
};

class SdFlightLog {
public:
  // 8 KiBをSD writeの上限目標とし、serialized recordを途中で分断しない。
  // schema v1 (128 B)では64 records = 8192 B、schema v2 (192 B)では
  // 42 records = 8064 Bとなる。record size変更時もcompile-timeで追従する。
  static constexpr std::size_t kWriteBatchTargetBytes = 8192U;
  static constexpr std::size_t kWriteBatchRecords =
      kWriteBatchTargetBytes / flight_log::kSerializedRecordBytes;
  static constexpr std::size_t kWriteBatchBytes =
      flight_log::kSerializedRecordBytes * kWriteBatchRecords;
  static constexpr std::size_t kPsramReserveBytes = 512U * 1024U;
  static constexpr std::size_t kMaxPsramStagingBytes = 8U * 1024U * 1024U;

  static_assert(kWriteBatchRecords > 0U);
  static_assert(kWriteBatchBytes <= kWriteBatchTargetBytes);
  static_assert(kWriteBatchTargetBytes - kWriteBatchBytes <
                flight_log::kSerializedRecordBytes);

  ~SdFlightLog();
  [[nodiscard]] esp_err_t prepareForFlight();
  [[nodiscard]] esp_err_t openExisting();
  [[nodiscard]] esp_err_t append(const flight_log::SerializedRecord &record);
  [[nodiscard]] esp_err_t flush();
  [[nodiscard]] esp_err_t read(uint32_t offset, uint8_t *destination,
                               std::size_t requested,
                               std::size_t &read_size);
  [[nodiscard]] esp_err_t exportRawFlashAndErase(InternalFlashLog &flash);
  [[nodiscard]] uint32_t size() const {
    return file_size_.load(std::memory_order_acquire);
  }
  [[nodiscard]] bool ready() const { return file_descriptor_ >= 0; }
  [[nodiscard]] bool writable() const { return writable_; }
  [[nodiscard]] std::size_t psramStagingBytes() const {
    return psram_staging_bytes_;
  }
  [[nodiscard]] std::size_t psramCapacityRecords() const {
    return psram_capacity_records_;
  }
  [[nodiscard]] uint32_t psramHighWaterRecords() const {
    return psram_high_water_records_.load(std::memory_order_relaxed);
  }

private:
  static constexpr std::size_t kWriterStackWords = 4096U;
  static constexpr UBaseType_t kWriterPriority = 6U;

  static void writerTaskEntry(void *context);
  void writerTaskLoop();
  [[nodiscard]] esp_err_t mount();
  [[nodiscard]] esp_err_t allocateStaging();
  void releaseStaging();
  [[nodiscard]] esp_err_t startWriter();
  void stopWriter();
  [[nodiscard]] esp_err_t requestWriterFlush();
  [[nodiscard]] bool drainOneBatch(bool flush_partial);
  [[nodiscard]] esp_err_t writeBatch(std::size_t record_count);
  void close();

  sdmmc_card_t *card_{};
  int file_descriptor_{-1};
  bool mounted_{};
  bool writable_{};
  std::atomic<uint32_t> file_size_{};

  flight_log::SerializedRecord *psram_stage_{};
  std::size_t psram_staging_bytes_{};
  std::size_t psram_capacity_records_{};
  std::atomic<uint64_t> psram_write_index_{};
  std::atomic<uint64_t> psram_read_index_{};
  std::atomic<uint32_t> psram_high_water_records_{};

  std::array<uint8_t, kWriteBatchBytes> sd_write_batch_{};

  StaticTask_t writer_task_control_{};
  std::array<StackType_t, kWriterStackWords> writer_task_stack_{};
  TaskHandle_t writer_task_{};

  StaticSemaphore_t flush_ack_storage_{};
  SemaphoreHandle_t flush_ack_{};
  StaticSemaphore_t stop_ack_storage_{};
  SemaphoreHandle_t stop_ack_{};

  std::atomic<bool> flush_requested_{};
  std::atomic<bool> stop_requested_{};
  std::atomic<esp_err_t> writer_error_{ESP_OK};
  std::atomic<esp_err_t> flush_result_{ESP_OK};
};

} // 名前空間 runtime::flight_storage
