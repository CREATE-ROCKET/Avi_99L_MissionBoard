#include "characterization/log_writer_v5.hpp"

#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION

#include "characterization/best_effort_file_close.hpp"
#include "characterization/rate_check_stage_diagnostics.hpp"
#include "characterization/record_validation.hpp"
#include "config/board_config.hpp"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace avi::characterization {
namespace {

constexpr const char *kMountPoint = "/sdcard";
constexpr const char *kLogDirectory = "/sdcard/characterization_v5";

void rememberOperation(esp_err_t operation, esp_err_t &first) noexcept {
  if (first == ESP_OK && operation != ESP_OK)
    first = operation;
}

std::uint32_t elapsedUs(std::int64_t started_at_us) noexcept {
  const std::int64_t finished_at_us = esp_timer_get_time();
  if (started_at_us < 0 || finished_at_us < started_at_us)
    return 0U;
  const std::uint64_t elapsed =
      static_cast<std::uint64_t>(finished_at_us - started_at_us);
  return static_cast<std::uint32_t>(
      elapsed > std::numeric_limits<std::uint32_t>::max()
          ? std::numeric_limits<std::uint32_t>::max()
          : elapsed);
}

void updateAtomicMaximum(std::atomic<std::uint32_t> &target,
                         std::uint32_t candidate) noexcept {
  std::uint32_t current = target.load(std::memory_order_relaxed);
  while (current < candidate &&
         !target.compare_exchange_weak(current, candidate,
                                       std::memory_order_relaxed,
                                       std::memory_order_relaxed)) {
  }
}

void addAtomicTotal(std::atomic<std::uint32_t> &target,
                    std::uint32_t value) noexcept {
  std::uint32_t current = target.load(std::memory_order_relaxed);
  for (;;) {
    const std::uint32_t next =
        value > std::numeric_limits<std::uint32_t>::max() - current
            ? std::numeric_limits<std::uint32_t>::max()
            : current + value;
    if (target.compare_exchange_weak(current, next,
                                     std::memory_order_relaxed,
                                     std::memory_order_relaxed))
      return;
  }
}

esp_err_t writeDescriptor(void *context, const std::uint8_t *data,
                          std::size_t size) noexcept {
  auto &descriptor = *static_cast<int *>(context);
  if (descriptor < 0 || (data == nullptr && size != 0U))
    return ESP_ERR_INVALID_STATE;
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t written = ::write(descriptor, data + offset, size - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR)
      continue;
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t flushDescriptor(void *) noexcept { return ESP_OK; }

esp_err_t syncDescriptor(void *context) noexcept {
  const int descriptor = *static_cast<int *>(context);
  if (descriptor < 0)
    return ESP_ERR_INVALID_STATE;
  return ::fsync(descriptor) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t truncateAndCloseDescriptor(void *context) noexcept {
  auto &descriptor = *static_cast<int *>(context);
  if (descriptor < 0)
    return ESP_ERR_INVALID_STATE;
  esp_err_t first = ESP_OK;
  const off_t end_position = ::lseek(descriptor, 0, SEEK_CUR);
  if (end_position < 0) {
    first = ESP_FAIL;
  } else if (::ftruncate(descriptor, end_position) != 0) {
    first = ESP_FAIL;
  }
  if (::fsync(descriptor) != 0)
    rememberOperation(ESP_FAIL, first);
  if (::close(descriptor) != 0)
    rememberOperation(ESP_FAIL, first);
  descriptor = -1;
  return first;
}

} // 無名名前空間

void LogWriterV5::rememberFirst(esp_err_t error) noexcept {
  if (error == ESP_OK)
    return;
  esp_err_t expected = ESP_OK;
  if (first_error_.compare_exchange_strong(expected, error)) {
    const TaskHandle_t task = failure_notification_task_.load();
    if (task != nullptr)
      xTaskNotifyGive(task);
  }
}

void LogWriterV5::taskEntry(void *context) {
  static_cast<LogWriterV5 *>(context)->taskLoop();
}

void LogWriterV5::sdTaskEntry(void *context) {
  static_cast<LogWriterV5 *>(context)->sdTaskLoop();
}

esp_err_t LogWriterV5::initializeStorage() noexcept {
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 4;
  slot.clk = board::kSdClk;
  slot.cmd = board::kSdCmd;
  slot.d0 = board::kSdDat0;
  slot.d1 = board::kSdDat1;
  slot.d2 = board::kSdDat2;
  slot.d3 = board::kSdDat3;
  esp_vfs_fat_sdmmc_mount_config_t mount{};
  mount.format_if_mount_failed = false;
  mount.max_files = 2;
  mount.allocation_unit_size = 16U * 1024U;
  sdmmc_card_t *card = nullptr;
  const esp_err_t result =
      esp_vfs_fat_sdmmc_mount(kMountPoint, &host, &slot, &mount, &card);
  if (result != ESP_OK)
    return result;
  if (::mkdir(kLogDirectory, 0775) != 0 && errno != EEXIST) {
    (void)esp_vfs_fat_sdcard_unmount(kMountPoint, card);
    return ESP_FAIL;
  }
  return ESP_OK;
}

void LogWriterV5::resetRunMetrics() noexcept {
  xQueueReset(queue_);
  xQueueReset(control_queue_);
  xQueueReset(sd_control_queue_);
  while (xSemaphoreTake(control_ack_, 0U) == pdTRUE) {
  }
  while (xSemaphoreTake(sd_control_ack_, 0U) == pdTRUE) {
  }
  control_pending_.store(false);
  first_error_.store(ESP_OK);
  sd_write_failed_.store(false);
  records_written_.store(0U);
  first_sequence_.store(0U);
  last_sequence_.store(0U);
  queue_overflows_.store(0U);
  queue_high_water_.store(0U);
  psram_write_index_.store(0U, std::memory_order_relaxed);
  psram_read_index_.store(0U, std::memory_order_relaxed);
  psram_high_water_.store(0U);
  psram_overflows_.store(0U);
  max_batch_records_.store(0U);
  max_validate_us_.store(0U);
  max_encode_us_.store(0U);
  max_fwrite_us_.store(0U);
  batch_count_.store(0U);
  total_validate_us_.store(0U);
  total_encode_us_.store(0U);
  total_fwrite_us_.store(0U);
}

void LogWriterV5::cleanupPreparedFile(bool remove_file) noexcept {
  accepting_.store(false);
  synced_.store(false);
  psram_write_index_.store(0U, std::memory_order_relaxed);
  psram_read_index_.store(0U, std::memory_order_relaxed);
  if (file_descriptor_ >= 0) {
    (void)::close(file_descriptor_);
    file_descriptor_ = -1;
  }
  if (remove_file && current_path_[0] != '\0')
    (void)::unlink(current_path_.data());
  planned_file_bytes_ = 0U;
  contiguous_preallocated_ = false;
  prepared_ = false;
}

bool LogWriterV5::stageEncodedRecord(
    std::uint64_t sequence, const wire_v5::RecordBytes &bytes) noexcept {
  if (psram_stage_ == nullptr)
    return false;

  const std::uint64_t write_index =
      psram_write_index_.load(std::memory_order_relaxed);
  const std::uint64_t read_index =
      psram_read_index_.load(std::memory_order_acquire);
  const std::uint64_t occupancy = write_index - read_index;
  if (occupancy >= kPsramStagingCapacity) {
    psram_overflows_.fetch_add(1U, std::memory_order_relaxed);
    rememberFirst(ESP_ERR_NO_MEM);
    return false;
  }

  StagedRecord &slot = psram_stage_[write_index % kPsramStagingCapacity];
  slot.sequence = sequence;
  std::memcpy(slot.bytes.data(), bytes.data(), bytes.size());
  psram_write_index_.store(write_index + 1U, std::memory_order_release);
  updateAtomicMaximum(
      psram_high_water_,
      static_cast<std::uint32_t>(std::min<std::uint64_t>(
          occupancy + 1U, std::numeric_limits<std::uint32_t>::max())));
  return true;
}

void LogWriterV5::processRecordBatch(ImmutableLogRecord current_record) {
  std::size_t count = 0U;
  for (;;) {
    // DRAM ingress queueから取り出したrecordをvalidation/encodeし、SD I/Oとは分離して
    // 4 MiB PSRAM ringへ退避する。SD stall中もこのtaskはrecordを吸収し続ける。
    const std::int64_t validate_started_us = esp_timer_get_time();
    const bool record_valid = !hasError(validateRecordStrict(current_record));
    const std::uint32_t validate_elapsed_us = elapsedUs(validate_started_us);
    updateAtomicMaximum(max_validate_us_, validate_elapsed_us);
    addAtomicTotal(total_validate_us_, validate_elapsed_us);
    if (!record_valid) {
      rememberFirst(ESP_ERR_INVALID_RESPONSE);
      break;
    }

    wire_v5::RecordBytes bytes{};
    const std::int64_t encode_started_us = esp_timer_get_time();
    const bool record_encoded = wire_v5::encodeRecord(current_record, bytes);
    const std::uint32_t encode_elapsed_us = elapsedUs(encode_started_us);
    updateAtomicMaximum(max_encode_us_, encode_elapsed_us);
    addAtomicTotal(total_encode_us_, encode_elapsed_us);
    if (!record_encoded) {
      rememberFirst(ESP_ERR_INVALID_RESPONSE);
      break;
    }

    if (!stageEncodedRecord(current_record.sequence, bytes))
      break;
    ++count;

    if (count >= kBatchRecords ||
        xQueueReceive(queue_, &current_record, 0U) != pdTRUE)
      break;
  }

  if (count != 0U && sd_task_ != nullptr)
    xTaskNotifyGive(sd_task_);
}

bool LogWriterV5::drainStagedBatch() noexcept {
  if (sd_write_failed_.load(std::memory_order_acquire))
    return false;
  if (psram_stage_ == nullptr || file_descriptor_ < 0)
    return false;

  const std::uint64_t read_index =
      psram_read_index_.load(std::memory_order_relaxed);
  const std::uint64_t write_index =
      psram_write_index_.load(std::memory_order_acquire);
  if (read_index == write_index)
    return false;

  const std::size_t count = static_cast<std::size_t>(
      std::min<std::uint64_t>(write_index - read_index, kBatchRecords));
  std::uint64_t first_sequence = 0U;
  std::uint64_t last_sequence = 0U;
  for (std::size_t index = 0U; index < count; ++index) {
    const StagedRecord &slot =
        psram_stage_[(read_index + index) % kPsramStagingCapacity];
    std::memcpy(sd_batch_.data() + index * wire_v5::kRecordBytes,
                slot.bytes.data(), slot.bytes.size());
    if (index == 0U)
      first_sequence = slot.sequence;
    last_sequence = slot.sequence;
  }

  const std::size_t byte_count = count * wire_v5::kRecordBytes;
  const std::int64_t fwrite_started_us = esp_timer_get_time();
  const esp_err_t write_result =
      writeDescriptor(&file_descriptor_, sd_batch_.data(), byte_count);
  const std::uint32_t fwrite_elapsed_us = elapsedUs(fwrite_started_us);
  updateAtomicMaximum(max_fwrite_us_, fwrite_elapsed_us);
  addAtomicTotal(total_fwrite_us_, fwrite_elapsed_us);
  updateAtomicMaximum(max_batch_records_, static_cast<std::uint32_t>(count));
  batch_count_.fetch_add(1U, std::memory_order_relaxed);

  if (write_result != ESP_OK) {
    sd_write_failed_.store(true, std::memory_order_release);
    rememberFirst(write_result);
    return false;
  }

  // SDへ完全に書けたbatchだけread indexを進める。PSRAMからのcopy中・write中は
  // そのslotを未消費扱いにしてproducerによる上書きを防ぐ。
  file_crc32_ = wire_v5::crc32(sd_batch_.data(), byte_count, file_crc32_);
  const std::uint64_t before = records_written_.fetch_add(count);
  if (before == 0U)
    first_sequence_.store(first_sequence);
  last_sequence_.store(last_sequence);
  psram_read_index_.store(read_index + count, std::memory_order_release);
  return true;
}

void LogWriterV5::processSdControl(const ControlRequest &request) {
  esp_err_t result = first_error_.load();
  if (request.kind == ControlKind::Sync) {
    bool synchronized = file_descriptor_ >= 0;
    if (file_descriptor_ < 0) {
      rememberOperation(ESP_ERR_INVALID_STATE, result);
    } else {
      const esp_err_t sync_result = syncDescriptor(&file_descriptor_);
      synchronized = sync_result == ESP_OK;
      rememberOperation(sync_result, result);
    }
    synced_.store(synchronized);
  } else if (file_descriptor_ < 0) {
    rememberOperation(ESP_ERR_INVALID_STATE, result);
  } else {
    if (!synced_.load())
      rememberOperation(ESP_ERR_INVALID_STATE, result);
    LogFooterV5 finalized = request.footer;
    finalized.total_records = records_written_.load();
    finalized.first_sequence = first_sequence_.load();
    finalized.last_sequence = last_sequence_.load();
    finalized.file_crc32 = file_crc32_;
    finalized.statistics.writer_queue_overflows = writerQueueOverflows();
    wire_v5::FooterBytes bytes{};
    const bool footer_valid = wire_v5::encodeFooter(finalized, bytes);
    if (!footer_valid)
      rememberOperation(ESP_ERR_INVALID_ARG, result);
    const FileCloseOperations operations{
        &file_descriptor_, writeDescriptor, flushDescriptor, syncDescriptor,
        truncateAndCloseDescriptor};
    result = bestEffortFinalizeFile(
        operations, footer_valid ? bytes.data() : nullptr,
        footer_valid ? bytes.size() : 0U, result);
    // close callbackが実record+footer位置へtruncateしてdescriptor ownershipを終了する。
    prepared_ = false;
    synced_.store(false);
    accepting_.store(false);
  }
  rememberFirst(result);
  sd_control_result_.store(result);
  (void)xSemaphoreGive(sd_control_ack_);
}

void LogWriterV5::processControl(const ControlRequest &request) {
  esp_err_t result = first_error_.load();
  while (xSemaphoreTake(sd_control_ack_, 0U) == pdTRUE) {
  }
  if (sd_task_ == nullptr ||
      xQueueSend(sd_control_queue_, &request, portMAX_DELAY) != pdTRUE) {
    rememberOperation(ESP_FAIL, result);
  } else {
    xTaskNotifyGive(sd_task_);
    if (xSemaphoreTake(sd_control_ack_, portMAX_DELAY) != pdTRUE)
      rememberOperation(ESP_FAIL, result);
    else
      rememberOperation(sd_control_result_.load(), result);
  }
  rememberFirst(result);
  control_result_.store(result);
  control_pending_.store(false);
  (void)xSemaphoreGive(control_ack_);
}

void LogWriterV5::taskLoop() {
  // このtaskはSDへ触れない。DRAM ingress queueを優先してPSRAMへ逃がし、
  // accepting=false後にqueueが空になってからcontrolをSD taskへ渡す。
  for (;;) {
    ImmutableLogRecord first{};
    if (xQueueReceive(queue_, &first, 0U) == pdTRUE) {
      processRecordBatch(first);
      continue;
    }
    ControlRequest request{};
    if (xQueueReceive(control_queue_, &request, 0U) == pdTRUE) {
      processControl(request);
      continue;
    }
    if (xQueueReceive(queue_, &first, pdMS_TO_TICKS(10)) == pdTRUE)
      processRecordBatch(first);
  }
}

void LogWriterV5::sdTaskLoop() {
  // SDMMC hostのinterrupt allocationをchar_runtimeと分離するため、mountは
  // Core 1の低優先度char_writer自身から実行する。
  const esp_err_t storage_result = initializeStorage();
  startup_result_.store(storage_result, std::memory_order_release);
  (void)xSemaphoreGive(startup_ack_);
  if (storage_result != ESP_OK) {
    for (;;)
      vTaskDelay(portMAX_DELAY);
  }

  for (;;) {
    bool drained = false;
    while (drainStagedBatch())
      drained = true;

    ControlRequest request{};
    if (xQueueReceive(sd_control_queue_, &request, 0U) == pdTRUE) {
      // controlはstagerがDRAM ingressを全てPSRAMへ移した後に到着する。
      // SD stallから復帰した後、残るPSRAM recordを全て書いてからsync/finalizeする。
      while (drainStagedBatch()) {
      }
      processSdControl(request);
      continue;
    }

    if (!drained)
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
  }
}

esp_err_t LogWriterV5::submitControl(const ControlRequest &request) {
  bool expected = false;
  if (!control_pending_.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;
  while (xSemaphoreTake(control_ack_, 0U) == pdTRUE) {
  }
  if (xQueueSend(control_queue_, &request, portMAX_DELAY) != pdTRUE) {
    control_pending_.store(false);
    return ESP_FAIL;
  }
  if (xSemaphoreTake(control_ack_, portMAX_DELAY) != pdTRUE) {
    control_pending_.store(false);
    return ESP_FAIL;
  }
  return control_result_.load();
}

esp_err_t LogWriterV5::initialize() {
  if (initialized_)
    return ESP_OK;
  if (task_ != nullptr || sd_task_ != nullptr) {
    const esp_err_t previous = startup_result_.load(std::memory_order_acquire);
    return previous == ESP_ERR_NOT_FINISHED ? ESP_ERR_INVALID_STATE : previous;
  }

  // PSRAM ringはrun中に確保しない。4 MiBをboot時に明示的にexternal RAMから確保し、
  // SDの数秒級tail latencyを吸収する。SDへ渡すbatchだけは内部DRAMに置く。
  psram_stage_ = static_cast<StagedRecord *>(heap_caps_malloc(
      kPsramStagingBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (psram_stage_ == nullptr)
    return ESP_ERR_NO_MEM;

  queue_ = xQueueCreateStatic(
      kQueueDepth, sizeof(ImmutableLogRecord), queue_storage_.data(),
      &queue_control_);
  control_queue_ = xQueueCreateStatic(
      1U, sizeof(ControlRequest), control_queue_storage_.data(),
      &control_queue_control_);
  sd_control_queue_ = xQueueCreateStatic(
      1U, sizeof(ControlRequest), sd_control_queue_storage_.data(),
      &sd_control_queue_control_);
  control_ack_ = xSemaphoreCreateBinaryStatic(&control_ack_storage_);
  sd_control_ack_ = xSemaphoreCreateBinaryStatic(&sd_control_ack_storage_);
  startup_ack_ = xSemaphoreCreateBinaryStatic(&startup_ack_storage_);
  if (queue_ == nullptr || control_queue_ == nullptr ||
      sd_control_queue_ == nullptr || control_ack_ == nullptr ||
      sd_control_ack_ == nullptr || startup_ack_ == nullptr) {
    heap_caps_free(psram_stage_);
    psram_stage_ = nullptr;
    return ESP_ERR_NO_MEM;
  }

  startup_result_.store(ESP_ERR_NOT_FINISHED, std::memory_order_release);
  // char_writerはSD I/O専用の低優先度task。write()が数秒blockしても、
  // char_log_stage(priority 12)とchar_encoder(priority 23)はCore 1で実行を続けられる。
  sd_task_ = xTaskCreateStaticPinnedToCore(
      sdTaskEntry, "char_writer", sizeof(sd_task_stack_), this, 10,
      sd_task_stack_, &sd_task_tcb_, 1);
  if (sd_task_ == nullptr) {
    heap_caps_free(psram_stage_);
    psram_stage_ = nullptr;
    return ESP_ERR_NO_MEM;
  }
  if (xSemaphoreTake(startup_ack_, portMAX_DELAY) != pdTRUE)
    return ESP_FAIL;

  const esp_err_t storage_result =
      startup_result_.load(std::memory_order_acquire);
  if (storage_result != ESP_OK)
    return storage_result;

  task_ = xTaskCreateStaticPinnedToCore(
      taskEntry, "char_log_stage", sizeof(task_stack_), this, 12,
      task_stack_, &task_tcb_, 1);
  if (task_ == nullptr)
    return ESP_ERR_NO_MEM;

  initialized_ = true;
  return ESP_OK;
}

esp_err_t LogWriterV5::prepare(const char *base_name,
                               std::uint32_t expected_records) {
  if (!initialized_ || file_descriptor_ >= 0 || prepared_ ||
      base_name == nullptr || base_name[0] == '\0')
    return ESP_ERR_INVALID_STATE;
  if (expected_records == 0U)
    return ESP_ERR_INVALID_ARG;

  const std::uint64_t record_bytes =
      static_cast<std::uint64_t>(expected_records) * wire_v5::kRecordBytes;
  if (record_bytes >
      std::numeric_limits<std::uint64_t>::max() - wire_v5::kHeaderBytes -
          wire_v5::kFooterBytes)
    return ESP_ERR_INVALID_SIZE;
  const std::uint64_t planned_bytes = wire_v5::kHeaderBytes + record_bytes +
                                      wire_v5::kFooterBytes;
  if (planned_bytes >
      static_cast<std::uint64_t>(std::numeric_limits<off_t>::max()))
    return ESP_ERR_INVALID_SIZE;

  bool path_reserved = false;
  for (unsigned index = 0U; index < 1'000U; ++index) {
    const int length = std::snprintf(
        current_path_.data(), current_path_.size(), "%s/%s_%03u.bin",
        kLogDirectory, base_name, index);
    if (length <= 0 ||
        static_cast<std::size_t>(length) >= current_path_.size())
      return ESP_ERR_INVALID_SIZE;
    const int placeholder =
        ::open(current_path_.data(), O_WRONLY | O_CREAT | O_EXCL, 0664);
    if (placeholder >= 0) {
      if (::close(placeholder) != 0) {
        (void)::unlink(current_path_.data());
        return ESP_FAIL;
      }
      path_reserved = true;
      break;
    }
    if (errno != EEXIST)
      return ESP_FAIL;
  }
  if (!path_reserved)
    return ESP_ERR_NOT_FOUND;

  const std::int64_t preallocation_started_us = esp_timer_get_time();
  const esp_err_t allocation_result = esp_vfs_fat_create_contiguous_file(
      kMountPoint, current_path_.data(), planned_bytes, true);
  if (allocation_result != ESP_OK) {
    (void)::unlink(current_path_.data());
    return allocation_result;
  }
  bool contiguous = false;
  const esp_err_t contiguous_result = esp_vfs_fat_test_contiguous_file(
      kMountPoint, current_path_.data(), &contiguous);
  if (contiguous_result != ESP_OK || !contiguous) {
    (void)::unlink(current_path_.data());
    return contiguous_result != ESP_OK ? contiguous_result
                                       : ESP_ERR_INVALID_STATE;
  }

  file_descriptor_ = ::open(current_path_.data(), O_RDWR);
  if (file_descriptor_ < 0) {
    (void)::unlink(current_path_.data());
    return ESP_FAIL;
  }
  if (::fsync(file_descriptor_) != 0 ||
      ::lseek(file_descriptor_, 0, SEEK_SET) != 0) {
    cleanupPreparedFile(true);
    return ESP_FAIL;
  }

  preallocation_us_.store(elapsedUs(preallocation_started_us));
  planned_file_bytes_ = planned_bytes;
  contiguous_preallocated_ = true;
  prepared_ = true;
  accepting_.store(false);
  synced_.store(true);
  return ESP_OK;
}

esp_err_t LogWriterV5::open(const LogHeaderV5 &header) {
  if (!initialized_ || !prepared_ || file_descriptor_ < 0 ||
      accepting_.load() || !contiguous_preallocated_)
    return ESP_ERR_INVALID_STATE;

  wire_v5::HeaderBytes bytes{};
  if (!wire_v5::encodeHeader(header, bytes)) {
    cleanupPreparedFile(true);
    return ESP_ERR_INVALID_ARG;
  }
  if (::lseek(file_descriptor_, 0, SEEK_SET) != 0 ||
      writeDescriptor(&file_descriptor_, bytes.data(), bytes.size()) != ESP_OK) {
    cleanupPreparedFile(true);
    return ESP_FAIL;
  }

  resetRunMetrics();
  file_crc32_ = wire_v5::crc32(bytes.data(), bytes.size());
  synced_.store(false);
  accepting_.store(true);
  beginRateCheckStageDiagnostics(header.encoder_rate, header.run_kind);
  return ESP_OK;
}

esp_err_t LogWriterV5::enqueue(const ImmutableLogRecord &record) {
  if (!accepting_.load() || file_descriptor_ < 0 ||
      first_error_.load() != ESP_OK)
    return ESP_ERR_INVALID_STATE;
  // realtime callerは完成済みimmutable recordをDRAM queueへ値copyするだけに留める。
  RateCheckStageScope timing(RateCheckStage::WriterEnqueue);
  if (xQueueSend(queue_, &record, 0U) != pdTRUE) {
    updateAtomicMaximum(queue_high_water_,
                        static_cast<std::uint32_t>(kQueueDepth));
    queue_overflows_.fetch_add(1U);
    rememberFirst(ESP_ERR_NO_MEM);
    return ESP_ERR_NO_MEM;
  }
  updateAtomicMaximum(
      queue_high_water_,
      static_cast<std::uint32_t>(uxQueueMessagesWaiting(queue_)));
  return ESP_OK;
}

esp_err_t LogWriterV5::drainAndSync() {
  if (file_descriptor_ < 0 || !prepared_)
    return ESP_ERR_INVALID_STATE;
  accepting_.store(false);
  ControlRequest request{};
  request.kind = ControlKind::Sync;
  return submitControl(request);
}

esp_err_t LogWriterV5::close(const LogFooterV5 &footer) {
  if (file_descriptor_ < 0 || !prepared_) {
    endRateCheckStageDiagnostics();
    return ESP_ERR_INVALID_STATE;
  }
  const std::uint64_t planned_bytes = planned_file_bytes_;
  const bool contiguous_preallocated = contiguous_preallocated_;
  accepting_.store(false);
  ControlRequest request{};
  request.kind = ControlKind::Finalize;
  request.footer = footer;
  const esp_err_t result = submitControl(request);

  const RateCheckStageDiagnostics diagnostics =
      rateCheckStageDiagnosticsSnapshot();
  if (diagnostics.active) {
    std::printf(
        "CHAR_RATE_STAGE rate=%u power-latest-max-us=%u "
        "encoder-drain-max-us=%u assembler-release-max-us=%u "
        "angle-convert-max-us=%u record-validate-max-us=%u "
        "writer-enqueue-max-us=%u encoder-read-max-us=%u\n",
        static_cast<unsigned>(diagnostics.rate),
        static_cast<unsigned>(diagnostics.power_latest_max_us),
        static_cast<unsigned>(diagnostics.encoder_drain_max_us),
        static_cast<unsigned>(diagnostics.assembler_release_max_us),
        static_cast<unsigned>(diagnostics.angle_convert_max_us),
        static_cast<unsigned>(diagnostics.record_validate_max_us),
        static_cast<unsigned>(diagnostics.writer_enqueue_max_us),
        static_cast<unsigned>(diagnostics.encoder_read_max_us));

    const std::uint64_t records = records_written_.load();
    const std::uint32_t batches = batch_count_.load();
    const std::uint32_t validate_total = total_validate_us_.load();
    const std::uint32_t encode_total = total_encode_us_.load();
    const std::uint32_t fwrite_total = total_fwrite_us_.load();
    const std::uint32_t validate_average =
        records == 0U ? 0U
                      : static_cast<std::uint32_t>(validate_total / records);
    const std::uint32_t encode_average =
        records == 0U ? 0U
                      : static_cast<std::uint32_t>(encode_total / records);
    const std::uint32_t fwrite_average =
        batches == 0U ? 0U : fwrite_total / batches;
    const std::uint64_t write_bytes = records * wire_v5::kRecordBytes;
    const std::uint64_t write_bps =
        fwrite_total == 0U
            ? 0U
            : write_bytes * 1'000'000ULL /
                  static_cast<std::uint64_t>(fwrite_total);
    std::printf(
        "CHAR_WRITER_TIMING rate=%u queue-depth=%u queue-high-water=%u "
        "psram-bytes=%u psram-capacity-records=%u psram-high-water=%u "
        "psram-overflow=%llu batch-capacity=%u max-batch-records=%u "
        "batch-count=%u preallocate-us=%u planned-bytes=%llu contiguous=%u "
        "validate-max-us=%u validate-total-us=%u validate-avg-us=%u "
        "encode-max-us=%u encode-total-us=%u encode-avg-us=%u "
        "fwrite-max-us=%u fwrite-total-us=%u fwrite-avg-us=%u "
        "write-bps=%llu records-written=%llu\n",
        static_cast<unsigned>(diagnostics.rate),
        static_cast<unsigned>(kQueueDepth),
        static_cast<unsigned>(queue_high_water_.load()),
        static_cast<unsigned>(kPsramStagingBytes),
        static_cast<unsigned>(kPsramStagingCapacity),
        static_cast<unsigned>(psram_high_water_.load()),
        static_cast<unsigned long long>(psram_overflows_.load()),
        static_cast<unsigned>(kBatchRecords),
        static_cast<unsigned>(max_batch_records_.load()),
        static_cast<unsigned>(batches),
        static_cast<unsigned>(preallocation_us_.load()),
        static_cast<unsigned long long>(planned_bytes),
        contiguous_preallocated ? 1U : 0U,
        static_cast<unsigned>(max_validate_us_.load()),
        static_cast<unsigned>(validate_total),
        static_cast<unsigned>(validate_average),
        static_cast<unsigned>(max_encode_us_.load()),
        static_cast<unsigned>(encode_total),
        static_cast<unsigned>(encode_average),
        static_cast<unsigned>(max_fwrite_us_.load()),
        static_cast<unsigned>(fwrite_total),
        static_cast<unsigned>(fwrite_average),
        static_cast<unsigned long long>(write_bps),
        static_cast<unsigned long long>(records));
  }
  planned_file_bytes_ = 0U;
  contiguous_preallocated_ = false;
  endRateCheckStageDiagnostics();
  return result;
}

esp_err_t LogWriterV5::abortAndClose(LogFooterV5 footer) {
  footer.completion = CompletionCode::Aborted;
  footer.unsupported_reason = UnsupportedReason::None;
  if (file_descriptor_ < 0) {
    endRateCheckStageDiagnostics();
    return ESP_OK;
  }
  esp_err_t result = drainAndSync();
  const esp_err_t close_result = close(footer);
  rememberOperation(close_result, result);
  return result;
}

} // 名前空間 avi::characterization

#endif
