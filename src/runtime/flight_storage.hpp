#pragma once

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
  [[nodiscard]] esp_err_t erase();
  [[nodiscard]] uint32_t size() const { return write_offset_; }
  [[nodiscard]] bool ready() const { return partition_ != nullptr; }

private:
  [[nodiscard]] esp_err_t locatePartition();
  const esp_partition_t *partition_{};
  uint32_t write_offset_{};
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
