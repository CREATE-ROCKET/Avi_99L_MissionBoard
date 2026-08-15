from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old[:80]!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


# Internal Flashはrecordの解釈状態に依存せずraw evidenceを保持する。
Path("src/runtime/flight_storage.hpp").write_text(
    r'''#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "esp_err.h"
#include "esp_partition.h"
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
  ~SdFlightLog();
  [[nodiscard]] esp_err_t prepareForFlight();
  [[nodiscard]] esp_err_t openExisting();
  [[nodiscard]] esp_err_t append(const flight_log::SerializedRecord &record);
  [[nodiscard]] esp_err_t flush();
  [[nodiscard]] esp_err_t read(uint32_t offset, uint8_t *destination,
                               std::size_t requested,
                               std::size_t &read_size);
  [[nodiscard]] esp_err_t exportRawFlashAndErase(InternalFlashLog &flash);
  [[nodiscard]] uint32_t size() const { return file_size_; }
  [[nodiscard]] bool ready() const { return file_ != nullptr; }
  [[nodiscard]] bool writable() const { return writable_; }

private:
  [[nodiscard]] esp_err_t mount();
  void close();
  sdmmc_card_t *card_{};
  FILE *file_{};
  bool mounted_{};
  bool writable_{};
  uint32_t file_size_{};
  std::array<char, 4096> io_buffer_{};
};

} // 名前空間 runtime::flight_storage
''',
    encoding="utf-8",
)

Path("src/runtime/flight_storage.cpp").write_text(
    r'''#include "runtime/flight_storage.hpp"

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
''',
    encoding="utf-8",
)

# 未qualificationのMotorProfileはnormal Startのvalid gateを通さない。
replace_once(
    "src/config/board_config.hpp",
    "constexpr MotorProfile kFlightMotorA{1, MotorPolarity::positive_in1, true,\n"
    "                                     3.48F, 0.00855F, 1120.0F, 0.60F,\n"
    "                                     2.0F, 1.21208F};\n"
    "// TODO(HW_TEST): SpareMotorB実測値\n"
    "constexpr MotorProfile kSpareMotorB{2, MotorPolarity::unconfigured, false,\n"
    "                                    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};",
    "constexpr MotorProfile kFlightMotorA{1, MotorPolarity::positive_in1, true,\n"
    "                                     3.48F, 0.00855F, 1120.0F, 0.60F,\n"
    "                                     2.0F, 1.21208F};\n"
    "// TODO(HW_TEST): flight用qualification完了時だけtrueへ変更する。\n"
    "constexpr bool kFlightMotorAFlightQualified = false;\n"
    "// TODO(HW_TEST): SpareMotorB実測値\n"
    "constexpr MotorProfile kSpareMotorB{2, MotorPolarity::unconfigured, false,\n"
    "                                    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};\n"
    "constexpr bool kSpareMotorBFlightQualified = false;",
)

replace_once(
    "src/config/flight_config.hpp",
    "#elif AVI_99L_MOTOR_PROFILE_ID == 1\n"
    "inline constexpr const board::MotorProfile &kActiveFlightMotorProfile =\n"
    "    board::kFlightMotorA;\n"
    "#elif AVI_99L_MOTOR_PROFILE_ID == 2\n"
    "inline constexpr const board::MotorProfile &kActiveFlightMotorProfile =\n"
    "    board::kSpareMotorB;",
    "#elif AVI_99L_MOTOR_PROFILE_ID == 1\n"
    "inline constexpr const board::MotorProfile &kActiveFlightMotorProfile =\n"
    "    board::kFlightMotorA;\n"
    "inline constexpr bool kActiveFlightMotorProfileQualified =\n"
    "    board::kFlightMotorAFlightQualified;\n"
    "#elif AVI_99L_MOTOR_PROFILE_ID == 2\n"
    "inline constexpr const board::MotorProfile &kActiveFlightMotorProfile =\n"
    "    board::kSpareMotorB;\n"
    "inline constexpr bool kActiveFlightMotorProfileQualified =\n"
    "    board::kSpareMotorBFlightQualified;",
)
replace_once(
    "src/config/flight_config.hpp",
    "// TODO(HW_TEST): 初期HILではbring-upと同じ15%へ制限し、実機同定後に確定する。\n"
    "inline constexpr double kProductionMotorMaximumDuty = 0.15;",
    "// productionではTorqueMapperのcurrent/torque/angle制限を維持したまま、\n"
    "// PWM dutyだけを追加clampせず100%まで許可する。\n"
    "inline constexpr double kProductionMotorMaximumDuty = 1.0;",
)
replace_once(
    "src/config/flight_config.hpp",
    "  return kActiveFlightMotorProfile.parameters_valid &&\n"
    "         kActiveFlightMotorProfile.polarity != board::MotorPolarity::unconfigured;",
    "  return kActiveFlightMotorProfile.parameters_valid &&\n"
    "         kActiveFlightMotorProfile.polarity != board::MotorPolarity::unconfigured &&\n"
    "         kActiveFlightMotorProfileQualified;",
)

runtime = "src/runtime/production_runtime.cpp"

replace_once(
    runtime,
    "struct RecoveryDumpCursor {\n"
    "  bool active{};\n"
    "  protocol::RecoveryControl control{};\n"
    "  uint32_t next_offset{};\n"
    "  uint32_t remaining{};\n"
    "  uint8_t sequence{};\n"
    "};\n\n"
    "struct TimeState {",
    "struct RecoveryDumpCursor {\n"
    "  bool active{};\n"
    "  protocol::RecoveryControl control{};\n"
    "  uint32_t next_offset{};\n"
    "  uint32_t remaining{};\n"
    "  uint8_t sequence{};\n"
    "};\n\n"
    "struct StorageExportRequest {\n"
    "  uint8_t transaction_id{};\n"
    "};\n\n"
    "struct TimeState {",
)

replace_once(
    runtime,
    "StaticQueue_t sd_recovery_queue_storage;\n"
    "StaticQueue_t flash_log_queue_storage;",
    "StaticQueue_t sd_recovery_queue_storage;\n"
    "StaticQueue_t storage_export_queue_storage;\n"
    "StaticQueue_t flash_log_queue_storage;",
)
replace_once(
    runtime,
    "std::array<uint8_t, sizeof(protocol::RecoveryControl) * 4>\n"
    "    sd_recovery_queue_buffer{};\n"
    "std::array<uint8_t, sizeof(flight_log::SerializedRecord) * 32>",
    "std::array<uint8_t, sizeof(protocol::RecoveryControl) * 4>\n"
    "    sd_recovery_queue_buffer{};\n"
    "std::array<uint8_t, sizeof(StorageExportRequest) * 2>\n"
    "    storage_export_queue_buffer{};\n"
    "std::array<uint8_t, sizeof(flight_log::SerializedRecord) * 32>",
)
replace_once(
    runtime,
    "QueueHandle_t sd_recovery_queue{};\n"
    "QueueHandle_t flash_log_queue{};",
    "QueueHandle_t sd_recovery_queue{};\n"
    "QueueHandle_t storage_export_queue{};\n"
    "QueueHandle_t flash_log_queue{};",
)
replace_once(
    runtime,
    "std::atomic<uint32_t> pressure_deployment_epoch{};\n"
    "TimeState time_state;",
    "std::atomic<uint32_t> pressure_deployment_epoch{};\n"
    "std::atomic<uint32_t> preflight_calibration_generation{};\n"
    "std::atomic<bool> preflight_calibration_active{false};\n"
    "TimeState time_state;",
)

# Command workerはFin commandをowner taskへ渡し、device可否はRealtimeTaskで判定する。
replace_once(
    runtime,
    "      context.fin_available =\n"
    "          encoder_ready.load(std::memory_order_acquire) &&\n"
    "          motor_ready.load(std::memory_order_acquire) &&\n"
    "          fin_zero_configured.load(std::memory_order_acquire);\n"
    "      // 入口ではSTS接続状態ではなく、owner taskへの経路だけを判定する。\n"
    "      context.parachute_available = parachute_command_queue != nullptr;\n"
    "      context.fin_safe_commands_supported = false;\n"
    "      context.calibration_supported = true;",
    "      // FinFree/SetFinZeroはmotor/zero未準備でも意味を持つため、\n"
    "      // 個別device gateはMissionRealtimeTaskで判定する。\n"
    "      context.fin_available = true;\n"
    "      // 入口ではSTS接続状態ではなく、owner taskへの経路だけを判定する。\n"
    "      context.parachute_available = parachute_command_queue != nullptr;\n"
    "      context.fin_safe_commands_supported = true;\n"
    "      context.calibration_supported = true;\n"
    "      context.storage_export_supported = storage_export_queue != nullptr;",
)
replace_once(
    runtime,
    "      QueueHandle_t destination = transition_queue;\n"
    "      ParachuteCommandRequest parachute_command{};\n"
    "      const bool parachute_domain =\n"
    "          envelope.decision.domain == mission::CommandDomain::parachute;\n"
    "      if (parachute_domain) {\n"
    "        parachute_command = {ParachuteCommandRequest::Kind::generic,\n"
    "                             envelope.request, {}};\n"
    "        destination = parachute_command_queue;\n"
    "      }\n"
    "      const BaseType_t queued =\n"
    "          !envelope.decision.execute\n"
    "              ? pdTRUE\n"
    "              : (parachute_domain\n"
    "                     ? xQueueSend(destination, &parachute_command, 0)\n"
    "                     : xQueueSend(destination, &envelope, 0));",
    "      ParachuteCommandRequest parachute_command{};\n"
    "      const bool parachute_domain =\n"
    "          envelope.decision.domain == mission::CommandDomain::parachute;\n"
    "      const bool storage_domain =\n"
    "          envelope.decision.domain == mission::CommandDomain::storage;\n"
    "      if (parachute_domain)\n"
    "        parachute_command = {ParachuteCommandRequest::Kind::generic,\n"
    "                             envelope.request, {}};\n"
    "      const StorageExportRequest storage_request{\n"
    "          envelope.request.transaction_id};\n"
    "      BaseType_t queued = pdTRUE;\n"
    "      if (envelope.decision.execute) {\n"
    "        if (parachute_domain)\n"
    "          queued = xQueueSend(parachute_command_queue, &parachute_command, 0);\n"
    "        else if (storage_domain)\n"
    "          queued = xQueueSend(storage_export_queue, &storage_request, 0);\n"
    "        else\n"
    "          queued = xQueueSend(transition_queue, &envelope, 0);\n"
    "      }",
)

# normal bootは前flight raw backupを自動eraseしない。
replace_once(
    runtime,
    "  const bool recovery_boot_mode =\n"
    "      recovery_only_mode.load(std::memory_order_acquire);\n"
    "  const esp_err_t flash_log_result =\n"
    "      recovery_boot_mode ? internal_flash_log.openExisting()\n"
    "                         : internal_flash_log.prepareForFlight();\n"
    "  flash_log_ready.store(flash_log_result == ESP_OK, std::memory_order_release);",
    "  const esp_err_t flash_log_result = internal_flash_log.openExisting();\n"
    "  flash_log_ready.store(\n"
    "      flash_log_result == ESP_OK && !internal_flash_log.hasData(),\n"
    "      std::memory_order_release);",
)

# MissionRealtimeTaskにCommandReceive用Fin操作stateを追加する。
replace_once(
    runtime,
    "  bool fin_angle_available = false;\n"
    "  bool fin_zero_available = false;\n"
    "  double previous_wrapped_fin_rad = 0.0;",
    "  bool fin_angle_available = false;\n"
    "  bool fin_zero_available = false;\n"
    "  enum class CommandFinMode : uint8_t {\n"
    "    free,\n"
    "    zero_hold,\n"
    "    position_hold,\n"
    "    relative_move,\n"
    "  };\n"
    "  CommandFinMode command_fin_mode = CommandFinMode::free;\n"
    "  double command_fin_target_rad = 0.0;\n"
    "  struct PendingFinMove {\n"
    "    bool active{};\n"
    "    uint8_t transaction_id{};\n"
    "    uint64_t deadline_us{};\n"
    "    double target_rad{};\n"
    "  } pending_fin_move;\n"
    "  double previous_wrapped_fin_rad = 0.0;",
)

replace_once(
    runtime,
    "        if (decision.execute) {\n"
    "          actuator_output_inhibited.store(true, std::memory_order_release);\n"
    "          (void)motor_driver.coast();",
    "        if (decision.execute) {\n"
    "          actuator_output_inhibited.store(true, std::memory_order_release);\n"
    "          pending_fin_move = {};\n"
    "          command_fin_mode = CommandFinMode::free;\n"
    "          (void)motor_driver.coast();",
)

# Fin commandは設定と保持を分離し、relative moveだけ非同期完了とする。
replace_once(
    runtime,
    "      } else if (code == mission::CommandCode::disable_fin_control) {\n"
    "        transition = state_machine.disableFinControl();\n"
    "      } else if (code == mission::CommandCode::enter_recovery) {",
    "      } else if (code == mission::CommandCode::disable_fin_control) {\n"
    "        transition = state_machine.disableFinControl();\n"
    "      } else if (code == mission::CommandCode::fin_free) {\n"
    "        pending_fin_move = {};\n"
    "        command_fin_mode = CommandFinMode::free;\n"
    "        actuator_output_inhibited.store(false, std::memory_order_release);\n"
    "        transition = mission::TransitionResult::completed;\n"
    "      } else if (code == mission::CommandCode::set_fin_zero) {\n"
    "        if (!encoder_ready.load(std::memory_order_acquire) ||\n"
    "            !fin_angle_available) {\n"
    "          direct_reason = protocol::CommandReason::device_unavailable;\n"
    "        } else {\n"
    "          fin_zero_reference_rad = unwrapped_fin_rad;\n"
    "          fin_zero_available = true;\n"
    "          fin_angle_rad = 0.0;\n"
    "          fin_zero_configured.store(true, std::memory_order_release);\n"
    "          fin_zero_hold_valid.store(false, std::memory_order_release);\n"
    "          zero_hold_controller.resetValidity();\n"
    "          pending_fin_move = {};\n"
    "          command_fin_mode = CommandFinMode::free;\n"
    "          actuator_output_inhibited.store(false, std::memory_order_release);\n"
    "          transition = mission::TransitionResult::completed;\n"
    "        }\n"
    "      } else if (code == mission::CommandCode::start_fin_zero_hold) {\n"
    "        const bool usable =\n"
    "            fin_zero_available &&\n"
    "            fin_zero_configured.load(std::memory_order_acquire) &&\n"
    "            encoder_ready.load(std::memory_order_acquire) && fin_rate_valid &&\n"
    "            motor_ready.load(std::memory_order_acquire) &&\n"
    "            std::isfinite(fin_angle_rad) && std::isfinite(fin_rate_rad_s);\n"
    "        if (!fin_zero_available ||\n"
    "            !fin_zero_configured.load(std::memory_order_acquire)) {\n"
    "          direct_reason = protocol::CommandReason::not_configured;\n"
    "        } else if (!usable) {\n"
    "          direct_reason = protocol::CommandReason::device_unavailable;\n"
    "        } else {\n"
    "          pending_fin_move = {};\n"
    "          command_fin_target_rad = 0.0;\n"
    "          command_fin_mode = CommandFinMode::zero_hold;\n"
    "          actuator_output_inhibited.store(false, std::memory_order_release);\n"
    "          transition = mission::TransitionResult::completed;\n"
    "        }\n"
    "      } else if (code == mission::CommandCode::fin_move_relative) {\n"
    "        const bool usable =\n"
    "            fin_zero_available &&\n"
    "            fin_zero_configured.load(std::memory_order_acquire) &&\n"
    "            encoder_ready.load(std::memory_order_acquire) && fin_rate_valid &&\n"
    "            motor_ready.load(std::memory_order_acquire) &&\n"
    "            std::isfinite(fin_angle_rad) && std::isfinite(fin_rate_rad_s);\n"
    "        if (!fin_zero_available ||\n"
    "            !fin_zero_configured.load(std::memory_order_acquire)) {\n"
    "          direct_reason = protocol::CommandReason::not_configured;\n"
    "        } else if (!usable) {\n"
    "          direct_reason = protocol::CommandReason::device_unavailable;\n"
    "        } else {\n"
    "          const uint16_t raw =\n"
    "              static_cast<uint16_t>(command_envelope.request.arguments[0]) |\n"
    "              static_cast<uint16_t>(command_envelope.request.arguments[1]) << 8U;\n"
    "          const auto deci_degrees = static_cast<int16_t>(raw);\n"
    "          constexpr double kDeciDegreeToRad =\n"
    "              0.0017453292519943296;\n"
    "          const double target =\n"
    "              fin_angle_rad + static_cast<double>(deci_degrees) *\n"
    "                                  kDeciDegreeToRad;\n"
    "          if (!board::kFinSoftwareLimits.configured ||\n"
    "              target < board::kFinSoftwareLimits.minimum_rad ||\n"
    "              target > board::kFinSoftwareLimits.maximum_rad) {\n"
    "            direct_reason = protocol::CommandReason::invalid_argument;\n"
    "          } else {\n"
    "            command_fin_target_rad = target;\n"
    "            command_fin_mode = CommandFinMode::relative_move;\n"
    "            pending_fin_move =\n"
    "                {true, command_envelope.request.transaction_id,\n"
    "                 static_cast<uint64_t>(esp_timer_get_time()) + 10'000'000,\n"
    "                 target};\n"
    "            actuator_output_inhibited.store(false,\n"
    "                                             std::memory_order_release);\n"
    "            asynchronous_transition = true;\n"
    "          }\n"
    "        }\n"
    "      } else if (code == mission::CommandCode::enter_recovery) {",
)

# PreflightCalibration開始時にSSC zeroも必ずinvalid化し、AirDataTaskへ新attemptを通知する。
replace_once(
    runtime,
    "          // 最新attemptだけを有効にする。途中失敗時に古い値へrollbackしない。\n"
    "          preflight_gyro_bias_valid = false;\n"
    "          gravity_reference_valid = false;\n"
    "          asynchronous_transition = true;",
    "          // 最新attemptだけを有効にする。途中失敗時に古い値へrollbackしない。\n"
    "          preflight_gyro_bias_valid = false;\n"
    "          gravity_reference_valid = false;\n"
    "          latest_air_data.ssc_zero_valid = false;\n"
    "          latest_air_data.airspeed_valid = false;\n"
    "          uint32_t next_generation =\n"
    "              preflight_calibration_generation.fetch_add(\n"
    "                  1, std::memory_order_acq_rel) +\n"
    "              1;\n"
    "          if (next_generation == 0) {\n"
    "            next_generation = 1;\n"
    "            preflight_calibration_generation.store(\n"
    "                next_generation, std::memory_order_release);\n"
    "          }\n"
    "          preflight_calibration_active.store(true,\n"
    "                                             std::memory_order_release);\n"
    "          asynchronous_transition = true;",
)

# 起動時の最初のencoder sampleを勝手にFinZeroへ昇格させない。
replace_once(
    runtime,
    "        if (!fin_zero_available) {\n"
    "          // TODO(HW_TEST): 起動時の物理直進位置を0 degとみなす暫定zeroを、\n"
    "          // NVS値または明示calibrationへ置換する。\n"
    "          fin_zero_reference_rad = unwrapped_fin_rad;\n"
    "          fin_zero_available = true;\n"
    "          fin_zero_configured.store(true, std::memory_order_release);\n"
    "        }\n"
    "        fin_angle_rad = unwrapped_fin_rad - fin_zero_reference_rad;",
    "        if (fin_zero_available)\n"
    "          fin_angle_rad = unwrapped_fin_rad - fin_zero_reference_rad;",
)

# relative moveの終端判定は新しいencoder sampleを使い、失敗後は駆動を止める。
replace_once(
    runtime,
    "    fin_zero_hold_valid.store(zero_hold_valid, std::memory_order_release);\n\n"
    "    AirDataSnapshot air_data{};",
    "    fin_zero_hold_valid.store(zero_hold_valid, std::memory_order_release);\n\n"
    "    if (pending_fin_move.active) {\n"
    "      constexpr double kMoveToleranceRad = 0.008726646259971648;\n"
    "      constexpr double kMoveRateToleranceRadS = 0.03490658503988659;\n"
    "      const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());\n"
    "      const bool reached =\n"
    "          fin_sample_valid &&\n"
    "          std::abs(fin_angle_rad - pending_fin_move.target_rad) <=\n"
    "              kMoveToleranceRad &&\n"
    "          std::abs(fin_rate_rad_s) <= kMoveRateToleranceRadS;\n"
    "      const bool timed_out = now_us >= pending_fin_move.deadline_us;\n"
    "      if (reached)\n"
    "        command_fin_mode = CommandFinMode::position_hold;\n"
    "      else if (timed_out)\n"
    "        command_fin_mode = CommandFinMode::free;\n"
    "      if ((reached || timed_out) &&\n"
    "          xSemaphoreTake(executor_mutex, 0) == pdTRUE) {\n"
    "        const auto result = command_executor.finish(\n"
    "            pending_fin_move.transaction_id,\n"
    "            reached ? protocol::CommandPhase::completed\n"
    "                    : protocol::CommandPhase::failed,\n"
    "            reached ? protocol::CommandReason::none\n"
    "                    : protocol::CommandReason::timeout);\n"
    "        xSemaphoreGive(executor_mutex);\n"
    "        enqueueResult(result, false);\n"
    "        pending_fin_move = {};\n"
    "      }\n"
    "    }\n\n"
    "    AirDataSnapshot air_data{};",
)

# CommandReceiveでも明示Fin modeに従って駆動できるようにする。
replace_once(
    runtime,
    "    if (output_inhibited ||\n"
    "        mission_snapshot.state == protocol::MissionState::command_receive) {\n"
    "      motor_output_result = motor_driver.coast();\n"
    "      motor_output_coasting = true;\n"
    "      if (mission_snapshot.reset_invalidated)\n"
    "        torque_error = protocol::quantization::TorqueError::reset_invalidated;\n"
    "    } else if (!motor_ready.load(std::memory_order_acquire) ||",
    "    if (output_inhibited) {\n"
    "      motor_output_result = motor_driver.coast();\n"
    "      motor_output_coasting = true;\n"
    "      if (mission_snapshot.reset_invalidated)\n"
    "        torque_error = protocol::quantization::TorqueError::reset_invalidated;\n"
    "    } else if (mission_snapshot.state ==\n"
    "               protocol::MissionState::command_receive) {\n"
    "      if (command_fin_mode == CommandFinMode::free) {\n"
    "        motor_output_result = motor_driver.coast();\n"
    "        motor_output_coasting = true;\n"
    "      } else if (!motor_ready.load(std::memory_order_acquire) ||\n"
    "                 !motor_driver.initialized() || !fin_sample_valid) {\n"
    "        motor_output_result = motor_driver.brake();\n"
    "        motor_output_braking = true;\n"
    "        torque_error =\n"
    "            protocol::quantization::TorqueError::controller_input_invalid;\n"
    "      } else {\n"
    "        const double target =\n"
    "            command_fin_mode == CommandFinMode::zero_hold\n"
    "                ? 0.0\n"
    "                : command_fin_target_rad;\n"
    "        const auto request = zero_hold_controller.compute(\n"
    "            fin_angle_rad - target, fin_rate_rad_s);\n"
    "        motor_output_result = applyTorque(request);\n"
    "        motor_output_braking = !request.valid || !motor_command.valid;\n"
    "      }\n"
    "    } else if (!motor_ready.load(std::memory_order_acquire) ||",
)
replace_once(
    runtime,
    "    if (motor_output_coasting)\n"
    "      status.fin_mode = protocol::FinMode::free;\n"
    "    else if (motor_output_braking)\n"
    "      status.fin_mode = protocol::FinMode::brake;",
    "    if (motor_output_coasting)\n"
    "      status.fin_mode = protocol::FinMode::free;\n"
    "    else if (motor_output_braking)\n"
    "      status.fin_mode = protocol::FinMode::brake;\n"
    "    else if (mission_snapshot.state ==\n"
    "             protocol::MissionState::command_receive) {\n"
    "      if (command_fin_mode == CommandFinMode::zero_hold)\n"
    "        status.fin_mode = protocol::FinMode::zero_hold;\n"
    "      else if (command_fin_mode == CommandFinMode::position_hold)\n"
    "        status.fin_mode = protocol::FinMode::position_hold;\n"
    "      else if (command_fin_mode == CommandFinMode::relative_move)\n"
    "        status.fin_mode = protocol::FinMode::relative_move;\n"
    "    }",
)

# calibration完了時にzeroをfreezeする。
replace_once(
    runtime,
    "      uint32_t detail = 0;\n"
    "      if (!preflight_gyro_bias_valid)",
    "      preflight_calibration_active.store(false, std::memory_order_release);\n"
    "      uint32_t detail = 0;\n"
    "      if (!preflight_gyro_bias_valid)",
)

# AirDataTaskでは明示PreflightCalibration中だけSSC zeroを蓄積する。
replace_once(
    runtime,
    "  uint64_t last_ssc_us = 0;\n"
    "  uint64_t last_lps_us = 0;\n\n"
    "  auto updateMissionSnapshot = [&]() {",
    "  uint64_t last_ssc_us = 0;\n"
    "  uint64_t last_lps_us = 0;\n"
    "  uint32_t calibration_generation = 0;\n\n"
    "  auto updateMissionSnapshot = [&]() {",
)
replace_once(
    runtime,
    "  for (;;) {\n"
    "    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());\n"
    "    if (now_us - last_ssc_us >= 2'500) {",
    "  for (;;) {\n"
    "    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());\n"
    "    const bool calibration_active =\n"
    "        preflight_calibration_active.load(std::memory_order_acquire);\n"
    "    const uint32_t requested_generation =\n"
    "        preflight_calibration_generation.load(std::memory_order_acquire);\n"
    "    if (calibration_active && requested_generation != 0 &&\n"
    "        requested_generation != calibration_generation) {\n"
    "      pressure_conditioner.reset();\n"
    "      snapshot.ssc_zero_valid = false;\n"
    "      snapshot.airspeed_valid = false;\n"
    "      calibration_generation = requested_generation;\n"
    "    }\n"
    "    if (now_us - last_ssc_us >= 2'500) {",
)
replace_once(
    runtime,
    "          // TODO(HW_TEST): 静止・無風のCommandReceive起動約1秒をzero取得に\n"
    "          // 使用する暫定実装を、明示PreflightCalibrationへ置換する。\n"
    "          (void)pressure_conditioner.updateZero(\n"
    "              data.differential_pressure_pa,\n"
    "              mission_snapshot.state ==\n"
    "                  protocol::MissionState::command_receive);",
    "          const bool capture_zero =\n"
    "              preflight_calibration_active.load(std::memory_order_acquire) &&\n"
    "              calibration_generation != 0 &&\n"
    "              preflight_calibration_generation.load(\n"
    "                  std::memory_order_acquire) == calibration_generation;\n"
    "          (void)pressure_conditioner.updateZero(\n"
    "              data.differential_pressure_pa, capture_zero);",
)

# SdLogTaskへraw export ownerを追加する。成功時だけFlashをerase済みreadyへ遷移する。
replace_once(
    runtime,
    "  RecoveryDumpCursor sd_dump{};\n"
    "  for (;;) {\n"
    "    flight_log::SerializedRecord record{};",
    "  RecoveryDumpCursor sd_dump{};\n"
    "  for (;;) {\n"
    "    StorageExportRequest export_request{};\n"
    "    while (xQueueReceive(storage_export_queue, &export_request, 0) == pdTRUE) {\n"
    "      protocol::CommandReason reason = protocol::CommandReason::none;\n"
    "      esp_err_t export_result = ESP_ERR_INVALID_STATE;\n"
    "      if (!sd_flight_log.ready() || !internal_flash_log.ready()) {\n"
    "        reason = protocol::CommandReason::device_unavailable;\n"
    "      } else {\n"
    "        export_result =\n"
    "            sd_flight_log.exportRawFlashAndErase(internal_flash_log);\n"
    "        if (export_result != ESP_OK)\n"
    "          reason = protocol::CommandReason::persistence_error;\n"
    "      }\n"
    "      if (reason == protocol::CommandReason::none)\n"
    "        flash_log_ready.store(true, std::memory_order_release);\n"
    "      if (xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {\n"
    "        const auto result = command_executor.finish(\n"
    "            export_request.transaction_id,\n"
    "            reason == protocol::CommandReason::none\n"
    "                ? protocol::CommandPhase::completed\n"
    "                : protocol::CommandPhase::failed,\n"
    "            reason, static_cast<uint32_t>(export_result));\n"
    "        xSemaphoreGive(executor_mutex);\n"
    "        enqueueResult(result, false);\n"
    "      } else {\n"
    "        (void)xQueueSendToFront(storage_export_queue, &export_request, 0);\n"
    "        break;\n"
    "      }\n"
    "    }\n\n"
    "    flight_log::SerializedRecord record{};",
)

# queueを確保する。
replace_once(
    runtime,
    "  sd_recovery_queue = xQueueCreateStatic(\n"
    "      4, sizeof(protocol::RecoveryControl), sd_recovery_queue_buffer.data(),\n"
    "      &sd_recovery_queue_storage);\n"
    "  flash_log_queue = xQueueCreateStatic(",
    "  sd_recovery_queue = xQueueCreateStatic(\n"
    "      4, sizeof(protocol::RecoveryControl), sd_recovery_queue_buffer.data(),\n"
    "      &sd_recovery_queue_storage);\n"
    "  storage_export_queue = xQueueCreateStatic(\n"
    "      2, sizeof(StorageExportRequest), storage_export_queue_buffer.data(),\n"
    "      &storage_export_queue_storage);\n"
    "  flash_log_queue = xQueueCreateStatic(",
)
replace_once(
    runtime,
    "      recovery_log_data_queue == nullptr || sd_recovery_queue == nullptr ||\n"
    "      flash_log_queue == nullptr || sd_log_queue == nullptr ||",
    "      recovery_log_data_queue == nullptr || sd_recovery_queue == nullptr ||\n"
    "      storage_export_queue == nullptr || flash_log_queue == nullptr ||\n"
    "      sd_log_queue == nullptr ||",
)

print("Vault storage/FinZero/Calibration/MotorProfile/PWM patch applied")
