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

void LogWriterV5::processRecordBatch(ImmutableLogRecord first_record) {
  batch_[0] = first_record;
  std::size_t count = 1U;
  while (count < batch_.size() &&
         xQueueReceive(queue_, &batch_[count], 0U) == pdTRUE)
    ++count;

  bool encoded = true;
  for (std::size_t index = 0U; index < count; ++index) {
    wire_v5::RecordBytes bytes{};
    if (!wire_v5::encodeRecord(batch_[index], bytes)) {
      encoded = false;
      rememberFirst(ESP_ERR_INVALID_RESPONSE);
      break;
    }
    std::memcpy(encoded_batch_.data() + index * wire_v5::kRecordBytes,
                bytes.data(), bytes.size());
  }

  if (encoded && file_ != nullptr) {
    const std::size_t byte_count = count * wire_v5::kRecordBytes;
    if (std::fwrite(encoded_batch_.data(), 1U, byte_count, file_) !=
        byte_count) {
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
  esp_err_t result =
      esp_vfs_fat_sdmmc_mount(kMountPoint, &host, &slot, &mount, &card);
  if (result != ESP_OK)
    return result;
  if (::mkdir(kLogDirectory, 0775) != 0 && errno != EEXIST) {
    (void)esp_vfs_fat_sdcard_unmount(kMountPoint, card);
    return ESP_FAIL;
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
  if (control_queue_ == nullptr || control_ack_ == nullptr)
    return ESP_ERR_NO_MEM;
  task_ = xTaskCreateStaticPinnedToCore(
      taskEntry, "char_writer", sizeof(task_stack_), this, 10,
      task_stack_, &task_tcb_, 0);
  if (task_ == nullptr)
    return ESP_ERR_NO_MEM;
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
  if (hasError(validateRecord(record))) {
    rememberFirst(ESP_ERR_INVALID_RESPONSE);
    return ESP_ERR_INVALID_RESPONSE;
  }
  // queueへ完成済みrecordを値コピーし、writerはlive motor stateを読まない。
  RateCheckStageScope timing(RateCheckStage::WriterEnqueue);
  if (xQueueSend(queue_, &record, 0U) != pdTRUE) {
    queue_overflows_.fetch_add(1U);
    rememberFirst(ESP_ERR_NO_MEM);
    return ESP_ERR_NO_MEM;
  }
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