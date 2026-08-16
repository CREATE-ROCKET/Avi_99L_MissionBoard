#include "runtime/flight_storage.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

#include "config/board_config.hpp"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

namespace runtime::flight_storage {
namespace {

constexpr char kMountPoint[] = "/sdcard";
constexpr char kLatestFlightPath[] = "/sdcard/avi_99l_latest.bin";
constexpr char kRawFlashExportPath[] = "/sdcard/avi_99l_flash_raw.bin";
constexpr auto kFlightLogSubtype = static_cast<esp_partition_subtype_t>(0x40);
constexpr uint32_t kMinimumFlightLogBytes = 2U * 1024U * 1024U;
constexpr uint32_t kPsramCaps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

void updateAtomicMaximum(std::atomic<uint32_t> &target,
                         uint32_t candidate) noexcept {
  uint32_t current = target.load(std::memory_order_relaxed);
  while (current < candidate &&
         !target.compare_exchange_weak(current, candidate,
                                       std::memory_order_relaxed,
                                       std::memory_order_relaxed)) {
  }
}

} // 無名名前空間

esp_err_t InternalFlashLog::locatePartition() {
  if (partition_ != nullptr)
    return ESP_OK;
  partition_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                        kFlightLogSubtype, "flightlog");
  if (partition_ == nullptr || partition_->size < kMinimumFlightLogBytes) {
    partition_ = nullptr;
    return ESP_ERR_NOT_FOUND;
  }
  return ESP_OK;
}

esp_err_t InternalFlashLog::prepareForFlight() {
  const esp_err_t locate = locatePartition();
  if (locate != ESP_OK)
    return locate;
  const esp_err_t erased =
      esp_partition_erase_range(partition_, 0, partition_->size);
  if (erased != ESP_OK)
    return erased;
  write_offset_ = 0;
  has_data_ = false;
  return ESP_OK;
}

esp_err_t InternalFlashLog::openExisting() {
  const esp_err_t locate = locatePartition();
  if (locate != ESP_OK)
    return locate;
  write_offset_ = 0;
  has_data_ = false;
  bool valid_prefix = true;
  for (uint32_t block_offset = 0; block_offset < partition_->size;
       block_offset += static_cast<uint32_t>(scan_buffer_.size())) {
    const std::size_t block_size = std::min<std::size_t>(
        scan_buffer_.size(), partition_->size - block_offset);
    const esp_err_t read = esp_partition_read(partition_, block_offset,
                                              scan_buffer_.data(), block_size);
    if (read != ESP_OK)
      return read;
    has_data_ = has_data_ ||
                std::any_of(scan_buffer_.begin(),
                            scan_buffer_.begin() + block_size,
                            [](uint8_t value) { return value != 0xFFU; });
    if (!valid_prefix)
      continue;
    for (std::size_t local = 0;
         local + flight_log::kSerializedRecordBytes <= block_size;
         local += flight_log::kSerializedRecordBytes) {
      flight_log::SerializedRecord record{};
      std::memcpy(record.data(), scan_buffer_.data() + local, record.size());
      if (flight_log::erased(record) || !flight_log::validate(record)) {
        valid_prefix = false;
        break;
      }
      write_offset_ =
          block_offset + static_cast<uint32_t>(local + record.size());
    }
  }
  return ESP_OK;
}

esp_err_t
InternalFlashLog::append(const flight_log::SerializedRecord &record) {
  if (partition_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (!flight_log::validate(record))
    return ESP_ERR_INVALID_CRC;
  if (write_offset_ > partition_->size ||
      record.size() > partition_->size - write_offset_)
    return ESP_ERR_NO_MEM;
  const esp_err_t written =
      esp_partition_write(partition_, write_offset_, record.data(), record.size());
  if (written == ESP_OK) {
    write_offset_ += static_cast<uint32_t>(record.size());
    has_data_ = true;
  }
  return written;
}

esp_err_t InternalFlashLog::read(uint32_t offset, uint8_t *destination,
                                 std::size_t requested,
                                 std::size_t &read_size) const {
  read_size = 0;
  if (partition_ == nullptr || destination == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (offset >= write_offset_ || requested == 0)
    return ESP_OK;
  read_size = std::min<std::size_t>(requested, write_offset_ - offset);
  const esp_err_t result =
      esp_partition_read(partition_, offset, destination, read_size);
  if (result != ESP_OK)
    read_size = 0;
  return result;
}

esp_err_t InternalFlashLog::readRaw(uint32_t offset, uint8_t *destination,
                                    std::size_t requested,
                                    std::size_t &read_size) const {
  read_size = 0;
  if (partition_ == nullptr || destination == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (requested == 0 || offset >= partition_->size)
    return ESP_OK;
  read_size = std::min<std::size_t>(requested, partition_->size - offset);
  const esp_err_t result =
      esp_partition_read(partition_, offset, destination, read_size);
  if (result != ESP_OK)
    read_size = 0;
  return result;
}

esp_err_t InternalFlashLog::erase() {
  if (partition_ == nullptr) {
    const esp_err_t locate = locatePartition();
    if (locate != ESP_OK)
      return locate;
  }
  const esp_err_t result =
      esp_partition_erase_range(partition_, 0, partition_->size);
  if (result == ESP_OK) {
    write_offset_ = 0;
    has_data_ = false;
  }
  return result;
}

SdFlightLog::~SdFlightLog() { close(); }

esp_err_t SdFlightLog::mount() {
  if (mounted_)
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
  esp_vfs_fat_sdmmc_mount_config_t mount_config{};
  mount_config.format_if_mount_failed = false;
  mount_config.max_files = 2;
  mount_config.allocation_unit_size = 16U * 1024U;
  const esp_err_t result = esp_vfs_fat_sdmmc_mount(
      kMountPoint, &host, &slot, &mount_config, &card_);
  mounted_ = result == ESP_OK;
  return result;
}

esp_err_t SdFlightLog::allocateStaging() {
  if (psram_stage_ != nullptr)
    return ESP_OK;

  const std::size_t free_before = heap_caps_get_free_size(kPsramCaps);
  std::size_t allocated_bytes = 0U;

  // 他の通常heap allocationへ最低512 KiBを残し、その時点の最大連続領域から
  // 残りをflight log stagingへ割り当てる。最大値は搭載PSRAMと同じ8 MiBとする。
  for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
    const std::size_t largest = heap_caps_get_largest_free_block(kPsramCaps);
    if (largest <= kPsramReserveBytes + kWriteBatchBytes)
      break;

    std::size_t target = std::min<std::size_t>(
        kMaxPsramStagingBytes, largest - kPsramReserveBytes);
    target -= target % flight_log::kSerializedRecordBytes;
    if (target < kWriteBatchBytes)
      break;

    psram_stage_ = static_cast<flight_log::SerializedRecord *>(
        heap_caps_malloc(target, kPsramCaps));
    if (psram_stage_ != nullptr) {
      allocated_bytes = target;
      break;
    }
  }

  if (psram_stage_ == nullptr)
    return ESP_ERR_NO_MEM;

  psram_staging_bytes_ = allocated_bytes;
  psram_capacity_records_ =
      psram_staging_bytes_ / flight_log::kSerializedRecordBytes;
  psram_write_index_.store(0U, std::memory_order_relaxed);
  psram_read_index_.store(0U, std::memory_order_relaxed);
  psram_high_water_records_.store(0U, std::memory_order_relaxed);

  const std::size_t free_after = heap_caps_get_free_size(kPsramCaps);
  std::printf(
      "sd log psram: free_before=%u staging=%u reserve_target=%u "
      "free_after=%u records=%u batch=%u\n",
      static_cast<unsigned>(free_before),
      static_cast<unsigned>(psram_staging_bytes_),
      static_cast<unsigned>(kPsramReserveBytes), static_cast<unsigned>(free_after),
      static_cast<unsigned>(psram_capacity_records_),
      static_cast<unsigned>(kWriteBatchBytes));
  return ESP_OK;
}

void SdFlightLog::releaseStaging() {
  if (psram_stage_ != nullptr)
    heap_caps_free(psram_stage_);
  psram_stage_ = nullptr;
  psram_staging_bytes_ = 0U;
  psram_capacity_records_ = 0U;
  psram_write_index_.store(0U, std::memory_order_relaxed);
  psram_read_index_.store(0U, std::memory_order_relaxed);
  psram_high_water_records_.store(0U, std::memory_order_relaxed);
}

void SdFlightLog::writerTaskEntry(void *context) {
  static_cast<SdFlightLog *>(context)->writerTaskLoop();
}

esp_err_t SdFlightLog::startWriter() {
  if (writer_task_ != nullptr || file_descriptor_ < 0 ||
      psram_stage_ == nullptr || psram_capacity_records_ == 0U)
    return ESP_ERR_INVALID_STATE;

  flush_ack_ = xSemaphoreCreateBinaryStatic(&flush_ack_storage_);
  stop_ack_ = xSemaphoreCreateBinaryStatic(&stop_ack_storage_);
  if (flush_ack_ == nullptr || stop_ack_ == nullptr)
    return ESP_ERR_NO_MEM;

  while (xSemaphoreTake(flush_ack_, 0U) == pdTRUE) {
  }
  while (xSemaphoreTake(stop_ack_, 0U) == pdTRUE) {
  }

  flush_requested_.store(false, std::memory_order_relaxed);
  stop_requested_.store(false, std::memory_order_relaxed);
  writer_error_.store(ESP_OK, std::memory_order_relaxed);
  flush_result_.store(ESP_OK, std::memory_order_relaxed);

  // SdLogTaskはDRAM queue -> PSRAM stagingだけを担当する。
  // 実際のSD writeは低優先度helperへ隔離し、数秒級のSD stallを上位taskへ伝播させない。
  writer_task_ = xTaskCreateStatic(writerTaskEntry, "SdWriterTask",
                                   kWriterStackWords, this, kWriterPriority,
                                   writer_task_stack_.data(),
                                   &writer_task_control_);
  return writer_task_ == nullptr ? ESP_ERR_NO_MEM : ESP_OK;
}

void SdFlightLog::stopWriter() {
  if (writer_task_ == nullptr)
    return;

  if (stop_ack_ != nullptr) {
    while (xSemaphoreTake(stop_ack_, 0U) == pdTRUE) {
    }
  }
  stop_requested_.store(true, std::memory_order_release);
  xTaskNotifyGive(writer_task_);
  if (stop_ack_ != nullptr)
    (void)xSemaphoreTake(stop_ack_, portMAX_DELAY);
  writer_task_ = nullptr;
}

esp_err_t SdFlightLog::writeBatch(std::size_t record_count) {
  if (file_descriptor_ < 0 || record_count == 0U ||
      record_count > kWriteBatchRecords)
    return ESP_ERR_INVALID_ARG;

  const std::size_t byte_count =
      record_count * flight_log::kSerializedRecordBytes;
  const uint32_t current_size = file_size_.load(std::memory_order_relaxed);
  if (byte_count > static_cast<std::size_t>(
                       std::numeric_limits<uint32_t>::max() - current_size))
    return ESP_ERR_INVALID_SIZE;

  const off_t batch_start = ::lseek(file_descriptor_, 0, SEEK_CUR);
  if (batch_start < 0)
    return ESP_FAIL;

  std::size_t offset = 0U;
  while (offset < byte_count) {
    const ssize_t written = ::write(file_descriptor_,
                                    sd_write_batch_.data() + offset,
                                    byte_count - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR)
      continue;

    // 部分writeで壊れた末尾を残さず、最後に完全に書けたrecord境界へ戻す。
    (void)::ftruncate(file_descriptor_, batch_start);
    (void)::lseek(file_descriptor_, batch_start, SEEK_SET);
    return ESP_FAIL;
  }

  file_size_.store(current_size + static_cast<uint32_t>(byte_count),
                   std::memory_order_release);
  return ESP_OK;
}

bool SdFlightLog::drainOneBatch(bool flush_partial) {
  if (psram_stage_ == nullptr || psram_capacity_records_ == 0U)
    return false;

  const uint64_t read_index = psram_read_index_.load(std::memory_order_relaxed);
  const uint64_t write_index = psram_write_index_.load(std::memory_order_acquire);
  if (write_index < read_index) {
    writer_error_.store(ESP_ERR_INVALID_STATE, std::memory_order_release);
    return false;
  }

  const uint64_t available = write_index - read_index;
  if (available == 0U ||
      (!flush_partial && available < kWriteBatchRecords))
    return false;

  const std::size_t record_count = static_cast<std::size_t>(
      std::min<uint64_t>(available, kWriteBatchRecords));
  for (std::size_t index = 0U; index < record_count; ++index) {
    const auto &record =
        psram_stage_[(read_index + index) % psram_capacity_records_];
    std::memcpy(sd_write_batch_.data() +
                    index * flight_log::kSerializedRecordBytes,
                record.data(), record.size());
  }

  const esp_err_t result = writeBatch(record_count);
  if (result != ESP_OK) {
    writer_error_.store(result, std::memory_order_release);
    return false;
  }

  // SDへ完全に書けたbatchだけを消費済みにする。copy/write中はproducerが
  // 同じslotを再利用できないため、SD stall中も未書込recordを保持できる。
  psram_read_index_.store(read_index + record_count, std::memory_order_release);
  return true;
}

void SdFlightLog::writerTaskLoop() {
  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

    const bool flush = flush_requested_.load(std::memory_order_acquire);
    if (writer_error_.load(std::memory_order_acquire) == ESP_OK) {
      while (drainOneBatch(flush)) {
      }
    }

    if (flush) {
      esp_err_t result = writer_error_.load(std::memory_order_acquire);
      if (result == ESP_OK && ::fsync(file_descriptor_) != 0)
        result = ESP_FAIL;
      if (result != ESP_OK)
        writer_error_.store(result, std::memory_order_release);
      flush_result_.store(result, std::memory_order_release);
      flush_requested_.store(false, std::memory_order_release);
      if (flush_ack_ != nullptr)
        (void)xSemaphoreGive(flush_ack_);
    }

    if (stop_requested_.load(std::memory_order_acquire))
      break;
  }

  if (stop_ack_ != nullptr)
    (void)xSemaphoreGive(stop_ack_);
  vTaskDelete(nullptr);
}

esp_err_t SdFlightLog::requestWriterFlush() {
  if (writer_task_ == nullptr || flush_ack_ == nullptr)
    return ESP_ERR_INVALID_STATE;

  bool expected = false;
  if (!flush_requested_.compare_exchange_strong(
          expected, true, std::memory_order_acq_rel,
          std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;

  while (xSemaphoreTake(flush_ack_, 0U) == pdTRUE) {
  }
  xTaskNotifyGive(writer_task_);
  if (xSemaphoreTake(flush_ack_, portMAX_DELAY) != pdTRUE) {
    flush_requested_.store(false, std::memory_order_release);
    return ESP_FAIL;
  }
  return flush_result_.load(std::memory_order_acquire);
}

esp_err_t SdFlightLog::prepareForFlight() {
  const esp_err_t mounted = mount();
  if (mounted != ESP_OK)
    return mounted;
  if (file_descriptor_ >= 0)
    return ESP_ERR_INVALID_STATE;

  const esp_err_t staging = allocateStaging();
  if (staging != ESP_OK) {
    close();
    return staging;
  }

  file_descriptor_ =
      ::open(kLatestFlightPath, O_RDWR | O_CREAT | O_TRUNC, 0664);
  if (file_descriptor_ < 0) {
    close();
    return ESP_FAIL;
  }

  file_size_.store(0U, std::memory_order_relaxed);
  const esp_err_t writer = startWriter();
  if (writer != ESP_OK) {
    close();
    return writer;
  }

  writable_ = true;
  return ESP_OK;
}

esp_err_t SdFlightLog::openExisting() {
  const esp_err_t mounted = mount();
  if (mounted != ESP_OK)
    return mounted;
  if (file_descriptor_ >= 0)
    return ESP_ERR_INVALID_STATE;

  file_descriptor_ = ::open(kLatestFlightPath, O_RDONLY);
  if (file_descriptor_ < 0) {
    const esp_err_t result = errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    close();
    return result;
  }

  struct stat status {};
  if (::fstat(file_descriptor_, &status) != 0 || status.st_size < 0 ||
      static_cast<uint64_t>(status.st_size) >
          std::numeric_limits<uint32_t>::max()) {
    close();
    return ESP_ERR_INVALID_SIZE;
  }

  file_size_.store(static_cast<uint32_t>(status.st_size),
                   std::memory_order_release);
  writable_ = false;
  return ESP_OK;
}

esp_err_t SdFlightLog::append(const flight_log::SerializedRecord &record) {
  if (file_descriptor_ < 0 || !writable_ || writer_task_ == nullptr ||
      psram_stage_ == nullptr || psram_capacity_records_ == 0U)
    return ESP_ERR_INVALID_STATE;
  const esp_err_t writer_error = writer_error_.load(std::memory_order_acquire);
  if (writer_error != ESP_OK)
    return writer_error;
  if (!flight_log::validate(record))
    return ESP_ERR_INVALID_CRC;

  const uint64_t write_index = psram_write_index_.load(std::memory_order_relaxed);
  const uint64_t read_index = psram_read_index_.load(std::memory_order_acquire);
  if (write_index < read_index)
    return ESP_ERR_INVALID_STATE;

  const uint64_t occupancy = write_index - read_index;
  if (occupancy >= psram_capacity_records_)
    return ESP_ERR_NO_MEM;

  psram_stage_[write_index % psram_capacity_records_] = record;
  psram_write_index_.store(write_index + 1U, std::memory_order_release);

  const uint64_t next_occupancy = occupancy + 1U;
  updateAtomicMaximum(
      psram_high_water_records_,
      static_cast<uint32_t>(std::min<uint64_t>(
          next_occupancy, std::numeric_limits<uint32_t>::max())));

  // 通常writeは64 records = 8192 B単位。端数はflush時だけwriterが回収する。
  if (next_occupancy >= kWriteBatchRecords)
    xTaskNotifyGive(writer_task_);
  return ESP_OK;
}

esp_err_t SdFlightLog::flush() {
  if (file_descriptor_ < 0)
    return ESP_ERR_INVALID_STATE;
  if (!writable_)
    return ESP_OK;
  return requestWriterFlush();
}

esp_err_t SdFlightLog::read(uint32_t offset, uint8_t *destination,
                            std::size_t requested, std::size_t &read_size) {
  read_size = 0;
  if (file_descriptor_ < 0 || destination == nullptr)
    return ESP_ERR_INVALID_STATE;

  if (writable_) {
    const esp_err_t flushed = flush();
    if (flushed != ESP_OK)
      return flushed;
  }

  const uint32_t current_size = file_size_.load(std::memory_order_acquire);
  if (offset >= current_size || requested == 0U)
    return ESP_OK;

  const std::size_t wanted =
      std::min<std::size_t>(requested, current_size - offset);
  if (::lseek(file_descriptor_, static_cast<off_t>(offset), SEEK_SET) < 0)
    return ESP_FAIL;

  std::size_t completed = 0U;
  while (completed < wanted) {
    const ssize_t received =
        ::read(file_descriptor_, destination + completed, wanted - completed);
    if (received > 0) {
      completed += static_cast<std::size_t>(received);
      continue;
    }
    if (received < 0 && errno == EINTR)
      continue;
    read_size = 0U;
    if (writable_)
      (void)::lseek(file_descriptor_, 0, SEEK_END);
    return ESP_FAIL;
  }

  read_size = completed;
  if (writable_ && ::lseek(file_descriptor_, 0, SEEK_END) < 0) {
    read_size = 0U;
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t SdFlightLog::exportRawFlashAndErase(InternalFlashLog &flash) {
  if (!mounted_ || !flash.ready() || flash.capacity() == 0)
    return ESP_ERR_INVALID_STATE;
  if (writable_ && flush() != ESP_OK)
    return ESP_FAIL;

  FILE *raw = std::fopen(kRawFlashExportPath, "wb+");
  if (raw == nullptr)
    return ESP_FAIL;
  auto closeWith = [&](esp_err_t result) {
    if (std::fclose(raw) != 0 && result == ESP_OK)
      return ESP_FAIL;
    return result;
  };

  std::array<uint8_t, 4096> flash_bytes{};
  std::array<uint8_t, 4096> sd_bytes{};
  const uint32_t capacity = flash.capacity();
  for (uint32_t offset = 0; offset < capacity;) {
    const std::size_t requested =
        std::min<std::size_t>(flash_bytes.size(), capacity - offset);
    std::size_t read_size = 0;
    const esp_err_t read =
        flash.readRaw(offset, flash_bytes.data(), requested, read_size);
    if (read != ESP_OK || read_size != requested)
      return closeWith(read == ESP_OK ? ESP_FAIL : read);
    if (std::fwrite(flash_bytes.data(), 1, read_size, raw) != read_size)
      return closeWith(ESP_FAIL);
    offset += static_cast<uint32_t>(read_size);
  }
  if (std::fflush(raw) != 0 || ::fsync(::fileno(raw)) != 0)
    return closeWith(ESP_FAIL);
  if (std::fseek(raw, 0, SEEK_END) != 0)
    return closeWith(ESP_FAIL);
  const long raw_size = std::ftell(raw);
  if (raw_size < 0 || static_cast<uint32_t>(raw_size) != capacity)
    return closeWith(ESP_ERR_INVALID_SIZE);
  if (std::fseek(raw, 0, SEEK_SET) != 0)
    return closeWith(ESP_FAIL);

  for (uint32_t offset = 0; offset < capacity;) {
    const std::size_t requested =
        std::min<std::size_t>(flash_bytes.size(), capacity - offset);
    if (std::fread(sd_bytes.data(), 1, requested, raw) != requested)
      return closeWith(ESP_FAIL);
    std::size_t read_size = 0;
    const esp_err_t read =
        flash.readRaw(offset, flash_bytes.data(), requested, read_size);
    if (read != ESP_OK || read_size != requested)
      return closeWith(read == ESP_OK ? ESP_FAIL : read);
    if (!std::equal(sd_bytes.begin(), sd_bytes.begin() + requested,
                    flash_bytes.begin()))
      return closeWith(ESP_ERR_INVALID_CRC);
    offset += static_cast<uint32_t>(requested);
  }

  if (std::fclose(raw) != 0)
    return ESP_FAIL;
  return flash.erase();
}

void SdFlightLog::close() {
  if (file_descriptor_ >= 0 && writable_)
    (void)flush();

  stopWriter();

  if (file_descriptor_ >= 0) {
    (void)::close(file_descriptor_);
    file_descriptor_ = -1;
  }

  writable_ = false;
  file_size_.store(0U, std::memory_order_relaxed);
  releaseStaging();

  if (mounted_) {
    (void)esp_vfs_fat_sdcard_unmount(kMountPoint, card_);
    card_ = nullptr;
    mounted_ = false;
  }
}

} // 名前空間 runtime::flight_storage
