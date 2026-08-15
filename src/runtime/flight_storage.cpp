#include "runtime/flight_storage.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <sys/unistd.h>

#include "config/board_config.hpp"
#include "driver/sdmmc_host.h"
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

esp_err_t SdFlightLog::prepareForFlight() {
  const esp_err_t mounted = mount();
  if (mounted != ESP_OK)
    return mounted;
  if (file_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  file_ = std::fopen(kLatestFlightPath, "wb+");
  if (file_ == nullptr)
    return ESP_FAIL;
  if (std::setvbuf(file_, io_buffer_.data(), _IOFBF, io_buffer_.size()) != 0) {
    close();
    return ESP_FAIL;
  }
  writable_ = true;
  file_size_ = 0;
  return ESP_OK;
}

esp_err_t SdFlightLog::openExisting() {
  const esp_err_t mounted = mount();
  if (mounted != ESP_OK)
    return mounted;
  if (file_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  file_ = std::fopen(kLatestFlightPath, "rb");
  if (file_ == nullptr)
    return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
  if (std::setvbuf(file_, io_buffer_.data(), _IOFBF, io_buffer_.size()) != 0) {
    close();
    return ESP_FAIL;
  }
  if (std::fseek(file_, 0, SEEK_END) != 0) {
    close();
    return ESP_FAIL;
  }
  const long size = std::ftell(file_);
  if (size < 0 || static_cast<unsigned long>(size) > 0xFFFF'FFFFUL) {
    close();
    return ESP_ERR_INVALID_SIZE;
  }
  file_size_ = static_cast<uint32_t>(size);
  if (std::fseek(file_, 0, SEEK_SET) != 0) {
    close();
    return ESP_FAIL;
  }
  writable_ = false;
  return ESP_OK;
}

esp_err_t SdFlightLog::append(const flight_log::SerializedRecord &record) {
  if (file_ == nullptr || !writable_)
    return ESP_ERR_INVALID_STATE;
  if (!flight_log::validate(record))
    return ESP_ERR_INVALID_CRC;
  if (std::fwrite(record.data(), 1, record.size(), file_) != record.size())
    return ESP_FAIL;
  if (file_size_ > 0xFFFF'FFFFU - record.size())
    return ESP_ERR_INVALID_SIZE;
  file_size_ += static_cast<uint32_t>(record.size());
  return ESP_OK;
}

esp_err_t SdFlightLog::flush() {
  if (file_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (std::fflush(file_) != 0)
    return ESP_FAIL;
  return ::fsync(::fileno(file_)) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t SdFlightLog::read(uint32_t offset, uint8_t *destination,
                            std::size_t requested, std::size_t &read_size) {
  read_size = 0;
  if (file_ == nullptr || destination == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (writable_ && flush() != ESP_OK)
    return ESP_FAIL;
  if (offset >= file_size_ || requested == 0)
    return ESP_OK;
  const std::size_t wanted =
      std::min<std::size_t>(requested, file_size_ - offset);
  if (std::fseek(file_, static_cast<long>(offset), SEEK_SET) != 0)
    return ESP_FAIL;
  read_size = std::fread(destination, 1, wanted, file_);
  if (read_size != wanted) {
    read_size = 0;
    return ESP_FAIL;
  }
  if (writable_ && std::fseek(file_, 0, SEEK_END) != 0) {
    read_size = 0;
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
  if (file_ != nullptr) {
    if (writable_)
      (void)flush();
    (void)std::fclose(file_);
    file_ = nullptr;
  }
  writable_ = false;
  file_size_ = 0;
  if (mounted_) {
    (void)esp_vfs_fat_sdcard_unmount(kMountPoint, card_);
    card_ = nullptr;
    mounted_ = false;
  }
}

} // 名前空間 runtime::flight_storage
