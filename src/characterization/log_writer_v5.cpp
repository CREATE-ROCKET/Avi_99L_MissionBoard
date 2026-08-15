#include "characterization/log_writer_v5.hpp"

#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION

#include "characterization/best_effort_file_close.hpp"
#include "characterization/rate_check_stage_diagnostics.hpp"
#include "characterization/record_validation.hpp"
#include "config/board_config.hpp"
#include "driver/sdmmc_host.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/stat.h>
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

esp_err_t writeFile(void *context, const std::uint8_t *data,
                    std::size_t size) noexcept {
  auto *file = static_cast<FILE *>(context);
  return std::fwrite(data, 1U, size, file) == size ? ESP_OK : ESP_FAIL;
}

esp_err_t flushFile(void *context) noexcept {
  return std::fflush(static_cast<FILE *>(context)) == 0 ? ESP_OK
                                                        : ESP_FAIL;
}

esp_err_t syncFile(void *context) noexcept {
  auto *file = static_cast<FILE *>(context);
  return ::fsync(::fileno(file)) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t closeFile(void *context) noexcept {
  return std::fclose(static_cast<FILE *>(context)) == 0 ? ESP_OK
                                                        : ESP_FAIL;
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

void LogWriterV5::processRecordBatch(ImmutableLogRecord first_record) {
  batch_[0] = first_record;
  std::size_t count = 1U;
  while (count < batch_.size() &&
         xQueueReceive(queue_, &batch_[count], 0U) == pdTRUE)
    ++count;
  updateAtomicMaximum(max_batch_records_, static_cast<std::uint32_t>(count));

  bool encoded = true;
  for (std::size_t index = 0U; index < count; ++index) {
    // full record validationはwriter taskだけで行う。char_runtime側はmotor safety、
    // epoch成立、queue失敗を直接監視し、重いwire整合性確認を1 kHz pathへ置かない。
    const std::int64_t validate_started_us = esp_timer_get_time();
    const bool record_valid = !hasError(validateRecordStrict(batch_[index]));
    const std::uint32_t validate_elapsed_us = elapsedUs(validate_started_us);
    updateAtomicMaximum(max_validate_us_, validate_elapsed_us);
    addAtomicTotal(total_validate_us_, validate_elapsed_us);
    if (!record_valid) {
      encoded = false;
      rememberFirst(ESP_ERR_INVALID_RESPONSE);
      break;
    }

    wire_v5::RecordBytes bytes{};
    const std::int64_t encode_started_us = esp_timer_get_time();
    const bool record_encoded = wire_v5::encodeRecord(batch_[index], bytes);
    const std::uint32_t encode_elapsed_us = elapsedUs(encode_started_us);
    updateAtomicMaximum(max_encode_us_, encode_elapsed_us);
    addAtomicTotal(total_encode_us_, encode_elapsed_us);
    if (!record_encoded) {
      encoded = false;
      rememberFirst(ESP_ERR_INVALID_RESPONSE);
      break;
    }
    std::memcpy(encoded_batch_.data() + index * wire_v5::kRecordBytes,
                bytes.data(), bytes.size());
  }

  if (encoded && file_ != nullptr) {
    const std::size_t byte_count = count * wire_v5::kRecordBytes;
    const std::int64_t fwrite_started_us = esp_timer_get_time();
    const std::size_t written =
        std::fwrite(encoded_batch_.data(), 1U, byte_count, file_);
    const std::uint32_t fwrite_elapsed_us = elapsedUs(fwrite_started_us);
    updateAtomicMaximum(max_fwrite_us_, fwrite_elapsed_us);
    addAtomicTotal(total_fwrite_us_, fwrite_elapsed_us);
    batch_count_.fetch_add(1U, std::memory_order_relaxed);
    if (written != byte_count) {
      rememberFirst(ESP_FAIL);
    } else {
      for (std::size_t index = 0U; index < count; ++index)
        file_crc32_ = wire_v5::crc32(
            encoded_batch_.data() + index * wire_v5::kRecordBytes,
            wire_v5::kRecordBytes, file_crc32_);
      const std::uint64_t before = records_written_.fetch_add(count);
      if (before == 0U)
        first_sequence_.store(batch_[0].sequence);
      last_sequence_.store(batch_[count - 1U].sequence);
    }
  } else if (file_ == nullptr) {
    rememberFirst(ESP_ERR_INVALID_STATE);
  }
}

void LogWriterV5::processControl(const ControlRequest &request) {
  esp_err_t result = first_error_.load();
  if (request.kind == ControlKind::Sync) {
    bool synchronized = file_ != nullptr;
    if (file_ == nullptr) {
      rememberOperation(ESP_ERR_INVALID_STATE, result);
    } else {
      const esp_err_t flush_result = flushFile(file_);
      const esp_err_t sync_result = syncFile(file_);
      synchronized = flush_result == ESP_OK && sync_result == ESP_OK;
      rememberOperation(flush_result, result);
      rememberOperation(sync_result, result);
    }
    synced_.store(synchronized);
  } else if (file_ == nullptr) {
    rememberOperation(ESP_ERR_INVALID_STATE, result);
  } else {
    if (!synced_.load())
      rememberOperation(ESP_ERR_INVALID_STATE, result);
    LogFooterV5 finalized = request.footer;
    finalized.total_records = records_written_.load();
    finalized.first_sequence = first_sequence_.load();
    finalized.last_sequence = last_sequence_.load();
    finalized.file_crc32 = file_crc32_;
    finalized.statistics.writer_queue_overflows =
        queue_overflows_.load();
    wire_v5::FooterBytes bytes{};
    const bool footer_valid = wire_v5::encodeFooter(finalized, bytes);
    if (!footer_valid)
      rememberOperation(ESP_ERR_INVALID_ARG, result);
    FILE *const closing_file = file_;
    const FileCloseOperations operations{
        closing_file, writeFile, flushFile, syncFile, closeFile};
    result = bestEffortFinalizeFile(
        operations, footer_valid ? bytes.data() : nullptr,
        footer_valid ? bytes.size() : 0U, result);
    // close失敗時もhandleを再利用せず、FILE ownershipを確実に終了する。
    file_ = nullptr;
    synced_.store(false);
    accepting_.store(false);
  }
  rememberFirst(result);
  control_result_.store(result);
  control_pending_.store(false);
  (void)xSemaphoreGive(control_ack_);
}

void LogWriterV5::taskLoop() {
  // SDMMC hostのinterrupt allocationをchar_runtimeと分離するため、
  // mountをCore 1にpinされたchar_writer task自身から実行する。
  const esp_err_t storage_result = initializeStorage();
  startup_result_.store(storage_result, std::memory_order_release);
  (void)xSemaphoreGive(startup_ack_);
  if (storage_result != ESP_OK) {
    for (;;)
      vTaskDelay(portMAX_DELAY);
  }

  for (;;) {
    ImmutableLogRecord first{};
    if (xQueueReceive(queue_, &first, pdMS_TO_TICKS(10)) == pdTRUE) {
      processRecordBatch(first);
      continue;
    }
    ControlRequest request{};
    if (xQueueReceive(control_queue_, &request, 0U) == pdTRUE)
      processControl(request);
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
  // FILEへ触れるcallerはなく、writer taskのfinalize ackまで安全側で待機する。
  if (xSemaphoreTake(control_ack_, portMAX_DELAY) != pdTRUE) {
    control_pending_.store(false);
    return ESP_FAIL;
  }
  return control_result_.load();
}

esp_err_t LogWriterV5::initialize() {
  if (initialized_)
    return ESP_OK;
  if (task_ != nullptr) {
    const esp_err_t previous =
        startup_result_.load(std::memory_order_acquire);
    return previous == ESP_ERR_NOT_FINISHED ? ESP_ERR_INVALID_STATE : previous;
  }

  // queue storage/control block/task stackはobject lifetime中保持される。
  queue_ = xQueueCreateStatic(
      kQueueDepth, sizeof(ImmutableLogRecord), queue_storage_.data(),
      &queue_control_);
  if (queue_ == nullptr)
    return ESP_ERR_NO_MEM;
  control_queue_ = xQueueCreateStatic(
      1U, sizeof(ControlRequest), control_queue_storage_.data(),
      &control_queue_control_);
  control_ack_ = xSemaphoreCreateBinaryStatic(&control_ack_storage_);
  startup_ack_ = xSemaphoreCreateBinaryStatic(&startup_ack_storage_);
  if (control_queue_ == nullptr || control_ack_ == nullptr ||
      startup_ack_ == nullptr)
    return ESP_ERR_NO_MEM;

  startup_result_.store(ESP_ERR_NOT_FINISHED, std::memory_order_release);
  // Core 0はpriority 21のchar_runtime専用に近い状態へ保つ。
  // char_writerはCore 1へ置き、同coreのchar_encoder(priority 23)を常に優先する。
  // SDMMC mountもこのtaskの先頭で行い、storage ISRをCore 1側へ割り当てる。
  task_ = xTaskCreateStaticPinnedToCore(
      taskEntry, "char_writer", sizeof(task_stack_), this, 10,
      task_stack_, &task_tcb_, 1);
  if (task_ == nullptr)
    return ESP_ERR_NO_MEM;
  if (xSemaphoreTake(startup_ack_, portMAX_DELAY) != pdTRUE)
    return ESP_FAIL;

  const esp_err_t result = startup_result_.load(std::memory_order_acquire);
  if (result != ESP_OK)
    return result;
  initialized_ = true;
  return ESP_OK;
}

esp_err_t LogWriterV5::open(const LogHeaderV5 &header,
                            const char *base_name) {
  if (!initialized_ || file_ != nullptr || base_name == nullptr ||
      base_name[0] == '\0')
    return ESP_ERR_INVALID_STATE;
  wire_v5::HeaderBytes bytes{};
  if (!wire_v5::encodeHeader(header, bytes))
    return ESP_ERR_INVALID_ARG;

  int descriptor = -1;
  for (unsigned index = 0U; index < 1'000U; ++index) {
    const int length = std::snprintf(
        current_path_.data(), current_path_.size(), "%s/%s_%03u.bin",
        kLogDirectory, base_name, index);
    if (length <= 0 ||
        static_cast<std::size_t>(length) >= current_path_.size())
      return ESP_ERR_INVALID_SIZE;
    descriptor = ::open(current_path_.data(),
                        O_WRONLY | O_CREAT | O_EXCL, 0664);
    if (descriptor >= 0)
      break;
    if (errno != EEXIST)
      return ESP_FAIL;
  }
  if (descriptor < 0)
    return ESP_ERR_NOT_FOUND;

  file_ = ::fdopen(descriptor, "wb");
  if (file_ == nullptr) {
    (void)::close(descriptor);
    (void)::unlink(current_path_.data());
    return ESP_FAIL;
  }
  if (std::setvbuf(file_, nullptr, _IONBF, 0U) != 0 ||
      std::fwrite(bytes.data(), 1U, bytes.size(), file_) != bytes.size() ||
      std::fflush(file_) != 0 || ::fsync(::fileno(file_)) != 0) {
    (void)std::fclose(file_);
    file_ = nullptr;
    // O_EXCLでこのopenが作ったexact pathだけを失敗時に除去する。
    (void)::unlink(current_path_.data());
    return ESP_FAIL;
  }
  xQueueReset(queue_);
  xQueueReset(control_queue_);
  while (xSemaphoreTake(control_ack_, 0U) == pdTRUE) {
  }
  control_pending_.store(false);
  first_error_.store(ESP_OK);
  records_written_.store(0U);
  first_sequence_.store(0U);
  last_sequence_.store(0U);
  queue_overflows_.store(0U);
  queue_high_water_.store(0U);
  max_batch_records_.store(0U);
  max_validate_us_.store(0U);
  max_encode_us_.store(0U);
  max_fwrite_us_.store(0U);
  batch_count_.store(0U);
  total_validate_us_.store(0U);
  total_encode_us_.store(0U);
  total_fwrite_us_.store(0U);
  file_crc32_ = wire_v5::crc32(bytes.data(), bytes.size());
  synced_.store(false);
  accepting_.store(true);
  beginRateCheckStageDiagnostics(header.encoder_rate, header.run_kind);
  return ESP_OK;
}

esp_err_t LogWriterV5::enqueue(
    const ImmutableLogRecord &record) {
  if (!accepting_.load() || file_ == nullptr ||
      first_error_.load() != ESP_OK)
    return ESP_ERR_INVALID_STATE;
  // 完成済みimmutable recordを値copyするだけに留める。
  // strict validationはchar_writerのprocessRecordBatch()でencode直前に行う。
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
  if (file_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  accepting_.store(false);
  ControlRequest request{};
  request.kind = ControlKind::Sync;
  return submitControl(request);
}

esp_err_t LogWriterV5::close(const LogFooterV5 &footer) {
  if (file_ == nullptr) {
    endRateCheckStageDiagnostics();
    return ESP_ERR_INVALID_STATE;
  }
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
    std::printf(
        "CHAR_WRITER_TIMING rate=%u queue-depth=%u queue-high-water=%u "
        "max-batch-records=%u batch-count=%u validate-max-us=%u "
        "validate-total-us=%u validate-avg-us=%u encode-max-us=%u "
        "encode-total-us=%u encode-avg-us=%u fwrite-max-us=%u "
        "fwrite-total-us=%u fwrite-avg-us=%u records-written=%llu\n",
        static_cast<unsigned>(diagnostics.rate),
        static_cast<unsigned>(kQueueDepth),
        static_cast<unsigned>(queue_high_water_.load()),
        static_cast<unsigned>(max_batch_records_.load()),
        static_cast<unsigned>(batches),
        static_cast<unsigned>(max_validate_us_.load()),
        static_cast<unsigned>(validate_total),
        static_cast<unsigned>(validate_average),
        static_cast<unsigned>(max_encode_us_.load()),
        static_cast<unsigned>(encode_total),
        static_cast<unsigned>(encode_average),
        static_cast<unsigned>(max_fwrite_us_.load()),
        static_cast<unsigned>(fwrite_total),
        static_cast<unsigned>(fwrite_average),
        static_cast<unsigned long long>(records));
  }
  endRateCheckStageDiagnostics();
  return result;
}

esp_err_t LogWriterV5::abortAndClose(LogFooterV5 footer) {
  footer.completion = CompletionCode::Aborted;
  footer.unsupported_reason = UnsupportedReason::None;
  if (file_ == nullptr) {
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
