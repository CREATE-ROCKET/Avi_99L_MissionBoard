#include "bringup/storage_bringup.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/unistd.h>

#include "config/board_config.hpp"
#include "driver/sdmmc_host.h"
#include "esp_crc.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

namespace bringup::storage {
namespace {
constexpr char kMountPoint[] = "/sdcard";
constexpr char kTestPath[] = "/sdcard/avi_99l_bringup.bin";
constexpr uint32_t kSdTestBytes = 1024U * 1024U;
constexpr uint32_t kIoChunkBytes = 4096U;
constexpr uint32_t kFlashTestBytes = 4096U;
constexpr uint32_t kFlightLogAddress = 0x420000U;
constexpr uint32_t kFlightLogMinimumSize = 2U * 1024U * 1024U;
constexpr auto kFlightLogSubtype =
    static_cast<esp_partition_subtype_t>(0x40);
constexpr std::array<uint8_t, 8> kFlashMagic{
    'A', 'V', 'I', '9', '9', 'L', 'F', '1'};

std::array<uint8_t, kIoChunkBytes> io_buffer{};
std::atomic<bool> test_running{false};
// 再起動せずに永続性確認を完了したと誤認しないためのRAM専用状態。
bool flash_prepared_this_boot = false;

constexpr uint8_t sdPatternByte(uint32_t offset) {
  return static_cast<uint8_t>(
      ((offset * 73U + (offset >> 8U) * 19U + 0x5AU) ^ (offset >> 3U)) &
      0xFFU);
}

constexpr uint8_t flashPatternByte(uint32_t offset) {
  return offset < kFlashMagic.size()
             ? kFlashMagic[offset]
             : sdPatternByte(offset ^ 0x99'4C'4601U);
}

static_assert(sdPatternByte(0) == 0x5A);
static_assert(sdPatternByte(1) == 0xA3);
static_assert(flashPatternByte(0) == 'A');

void updateMaxLatency(int64_t started_us, uint64_t &maximum_us) {
  const int64_t elapsed = esp_timer_get_time() - started_us;
  if (elapsed > 0)
    maximum_us = std::max(maximum_us, static_cast<uint64_t>(elapsed));
}

void fillSdPattern(uint32_t offset, std::size_t size) {
  for (std::size_t index = 0; index < size; ++index)
    io_buffer[index] = sdPatternByte(offset + static_cast<uint32_t>(index));
}

void fillFlashPattern() {
  for (uint32_t index = 0; index < kFlashTestBytes; ++index)
    io_buffer[index] = flashPatternByte(index);
}

bool flashContentMatches() {
  for (uint32_t index = 0; index < kFlashTestBytes; ++index) {
    if (io_buffer[index] != flashPatternByte(index))
      return false;
  }
  return true;
}

bool flashContentErased() {
  return std::all_of(io_buffer.begin(),
                     io_buffer.begin() + kFlashTestBytes,
                     [](uint8_t value) { return value == 0xFFU; });
}

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

esp_err_t closeFile(FILE *&file, uint64_t &max_block_us, int &io_errno) {
  if (file == nullptr)
    return ESP_OK;
  const int64_t started = esp_timer_get_time();
  const int close_result = std::fclose(file);
  updateMaxLatency(started, max_block_us);
  file = nullptr;
  if (close_result == 0)
    return ESP_OK;
  io_errno = errno;
  return ESP_FAIL;
}

float throughputMiB(uint32_t bytes, uint64_t duration_us) {
  if (bytes == 0 || duration_us == 0)
    return 0.0F;
  return static_cast<float>(bytes) * 1'000'000.0F /
         (static_cast<float>(duration_us) * 1024.0F * 1024.0F);
}

esp_err_t runSdFileTest(SdTestResult &result) {
  int64_t operation_started = esp_timer_get_time();
  FILE *file = std::fopen(kTestPath, "wb");
  updateMaxLatency(operation_started, result.max_block_us);
  if (file == nullptr) {
    result.io_errno = errno;
    return ESP_FAIL;
  }

  esp_err_t first = ESP_OK;
  const int64_t write_started = esp_timer_get_time();
  uint32_t crc = 0;
  for (uint32_t offset = 0; offset < kSdTestBytes;) {
    const std::size_t size =
        std::min<std::size_t>(io_buffer.size(), kSdTestBytes - offset);
    fillSdPattern(offset, size);
    const int64_t operation_started = esp_timer_get_time();
    const std::size_t written = std::fwrite(io_buffer.data(), 1, size, file);
    updateMaxLatency(operation_started, result.max_block_us);
    result.written_bytes += static_cast<uint32_t>(written);
    if (written != size) {
      result.io_errno = errno;
      first = ESP_FAIL;
      break;
    }
    crc = esp_crc32_le(crc, io_buffer.data(), static_cast<uint32_t>(size));
    offset += static_cast<uint32_t>(size);
  }
  result.expected_crc32 = crc;

  if (first == ESP_OK) {
    const int64_t operation_started = esp_timer_get_time();
    if (std::fflush(file) != 0) {
      result.io_errno = errno;
      first = ESP_FAIL;
    }
    updateMaxLatency(operation_started, result.max_block_us);
  }
  if (first == ESP_OK) {
    const int64_t operation_started = esp_timer_get_time();
    if (::fsync(::fileno(file)) != 0) {
      result.io_errno = errno;
      first = ESP_FAIL;
    }
    updateMaxLatency(operation_started, result.max_block_us);
  }
  rememberFirst(closeFile(file, result.max_block_us, result.io_errno), first);
  result.write_duration_us =
      static_cast<uint64_t>(esp_timer_get_time() - write_started);
  result.write_mib_per_second =
      throughputMiB(result.written_bytes, result.write_duration_us);
  if (first != ESP_OK)
    return first;

  operation_started = esp_timer_get_time();
  file = std::fopen(kTestPath, "rb");
  updateMaxLatency(operation_started, result.max_block_us);
  if (file == nullptr) {
    result.io_errno = errno;
    return ESP_FAIL;
  }

  const int64_t read_started = esp_timer_get_time();
  crc = 0;
  result.content_matches = true;
  for (uint32_t offset = 0; offset < kSdTestBytes;) {
    const std::size_t size =
        std::min<std::size_t>(io_buffer.size(), kSdTestBytes - offset);
    const int64_t operation_started = esp_timer_get_time();
    const std::size_t read = std::fread(io_buffer.data(), 1, size, file);
    updateMaxLatency(operation_started, result.max_block_us);
    result.read_bytes += static_cast<uint32_t>(read);
    if (read != size) {
      result.io_errno = std::ferror(file) != 0 ? errno : 0;
      first = ESP_FAIL;
      break;
    }
    crc = esp_crc32_le(crc, io_buffer.data(), static_cast<uint32_t>(size));
    for (std::size_t index = 0; index < size; ++index) {
      if (io_buffer[index] !=
          sdPatternByte(offset + static_cast<uint32_t>(index))) {
        result.content_matches = false;
        first = ESP_ERR_INVALID_CRC;
        break;
      }
    }
    if (first != ESP_OK)
      break;
    offset += static_cast<uint32_t>(size);
  }
  result.read_crc32 = crc;
  if (first == ESP_OK && crc != result.expected_crc32) {
    result.content_matches = false;
    first = ESP_ERR_INVALID_CRC;
  }
  rememberFirst(closeFile(file, result.max_block_us, result.io_errno), first);
  result.read_duration_us =
      static_cast<uint64_t>(esp_timer_get_time() - read_started);
  result.read_mib_per_second =
      throughputMiB(result.read_bytes, result.read_duration_us);
  return first;
}

esp_err_t findAndVerifyFlightLog(const esp_partition_t *&partition) {
  partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                       kFlightLogSubtype,
                                       board::kFlightLogPartitionLabel);
  if (partition == nullptr)
    return ESP_ERR_NOT_FOUND;
  if (partition->type != ESP_PARTITION_TYPE_DATA ||
      partition->subtype != kFlightLogSubtype ||
      std::strcmp(partition->label, board::kFlightLogPartitionLabel) != 0 ||
      partition->address != kFlightLogAddress ||
      partition->size < kFlightLogMinimumSize || partition->erase_size == 0 ||
      kFlashTestBytes % partition->erase_size != 0)
    return ESP_ERR_INVALID_SIZE;
  return ESP_OK;
}

esp_err_t timedPartitionRead(const esp_partition_t *partition,
                             uint64_t &max_block_us) {
  const int64_t started = esp_timer_get_time();
  const esp_err_t result = esp_partition_read(
      partition, 0, io_buffer.data(), kFlashTestBytes);
  updateMaxLatency(started, max_block_us);
  return result;
}

esp_err_t timedPartitionErase(const esp_partition_t *partition,
                              uint64_t &max_block_us) {
  const int64_t started = esp_timer_get_time();
  const esp_err_t result =
      esp_partition_erase_range(partition, 0, kFlashTestBytes);
  updateMaxLatency(started, max_block_us);
  return result;
}

esp_err_t timedPartitionWrite(const esp_partition_t *partition,
                              uint64_t &max_block_us) {
  const int64_t started = esp_timer_get_time();
  const esp_err_t result = esp_partition_write(
      partition, 0, io_buffer.data(), kFlashTestBytes);
  updateMaxLatency(started, max_block_us);
  return result;
}

esp_err_t runFlashTest(FlashTestResult &result) {
  const esp_partition_t *partition = nullptr;
  esp_err_t status = findAndVerifyFlightLog(partition);
  if (status != ESP_OK)
    return status;
  result.partition_verified = true;
  result.tested_bytes = kFlashTestBytes;

  status = timedPartitionRead(partition, result.max_block_us);
  if (status != ESP_OK)
    return status;
  const bool prepared_pattern = flashContentMatches();
  result.read_crc32 =
      esp_crc32_le(0, io_buffer.data(), kFlashTestBytes);

  fillFlashPattern();
  result.expected_crc32 =
      esp_crc32_le(0, io_buffer.data(), kFlashTestBytes);

  if (prepared_pattern) {
    result.content_matches = true;
    result.phase = FlashTestPhase::reboot_required;
    if (flash_prepared_this_boot)
      return ESP_ERR_INVALID_STATE;

    status = timedPartitionErase(partition, result.max_block_us);
    if (status != ESP_OK)
      return status;
    status = timedPartitionRead(partition, result.max_block_us);
    if (status != ESP_OK)
      return status;
    result.erased = flashContentErased();
    if (!result.erased)
      return ESP_ERR_INVALID_CRC;
    result.phase = FlashTestPhase::reboot_verified_and_erased;
    return ESP_OK;
  }

  status = timedPartitionErase(partition, result.max_block_us);
  if (status != ESP_OK)
    return status;
  fillFlashPattern();
  status = timedPartitionWrite(partition, result.max_block_us);
  if (status != ESP_OK)
    return status;
  // write成功直後から同一bootの再試験をreboot後確認と誤認しない。
  flash_prepared_this_boot = true;
  std::fill(io_buffer.begin(), io_buffer.end(), 0U);
  status = timedPartitionRead(partition, result.max_block_us);
  if (status != ESP_OK)
    return status;
  result.read_crc32 =
      esp_crc32_le(0, io_buffer.data(), kFlashTestBytes);
  result.content_matches = flashContentMatches() &&
                           result.read_crc32 == result.expected_crc32;
  if (!result.content_matches)
    return ESP_ERR_INVALID_CRC;

  result.phase = FlashTestPhase::reboot_required;
  return ESP_OK;
}
} // 無名名前空間

esp_err_t sdTest(SdTestResult &result) {
  result = {};
  result.requested_bytes = kSdTestBytes;
  bool expected = false;
  if (!test_running.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;

  std::printf("SD idle level: DAT1=%d DAT0=%d CMD=%d DAT3=%d DAT2=%d CLK=%d\n",
              gpio_get_level(board::kSdDat1),
              gpio_get_level(board::kSdDat0),
              gpio_get_level(board::kSdCmd), gpio_get_level(board::kSdDat3),
              gpio_get_level(board::kSdDat2), gpio_get_level(board::kSdClk));

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 4;
  slot.clk = board::kSdClk;
  slot.cmd = board::kSdCmd;
  slot.d0 = board::kSdDat0;
  slot.d1 = board::kSdDat1;
  slot.d2 = board::kSdDat2;
  slot.d3 = board::kSdDat3;

  esp_vfs_fat_mount_config_t mount{};
  mount.format_if_mount_failed = false;
  mount.max_files = 2;
  mount.allocation_unit_size = 0;

  sdmmc_card_t *card = nullptr;
  const int64_t mount_started = esp_timer_get_time();
  esp_err_t first =
      esp_vfs_fat_sdmmc_mount(kMountPoint, &host, &slot, &mount, &card);
  updateMaxLatency(mount_started, result.max_block_us);
  if (first == ESP_OK) {
    result.mounted = true;
    first = runSdFileTest(result);
    const int64_t unmount_started = esp_timer_get_time();
    const esp_err_t unmount = esp_vfs_fat_sdcard_unmount(kMountPoint, card);
    updateMaxLatency(unmount_started, result.max_block_us);
    result.unmounted = unmount == ESP_OK;
    rememberFirst(unmount, first);
  }

  test_running.store(false);
  return first;
}

esp_err_t flashTest(FlashTestResult &result) {
  result = {};
  bool expected = false;
  if (!test_running.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;
  const esp_err_t status = runFlashTest(result);
  test_running.store(false);
  return status;
}

} // 名前空間 bringup::storage
