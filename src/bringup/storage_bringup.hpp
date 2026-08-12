#pragma once

#include <cstdint>

#include "esp_err.h"

namespace bringup::storage {

struct SdTestResult {
  uint32_t requested_bytes{0};
  uint32_t written_bytes{0};
  uint32_t read_bytes{0};
  uint32_t expected_crc32{0};
  uint32_t read_crc32{0};
  uint64_t write_duration_us{0};
  uint64_t read_duration_us{0};
  uint64_t max_block_us{0};
  float write_mib_per_second{0.0F};
  float read_mib_per_second{0.0F};
  int io_errno{0};
  bool mounted{false};
  bool content_matches{false};
  bool unmounted{false};
};

enum class FlashTestPhase : uint8_t {
  none,
  reboot_required,
  reboot_verified_and_erased,
};

struct FlashTestResult {
  FlashTestPhase phase{FlashTestPhase::none};
  uint32_t tested_bytes{0};
  uint32_t expected_crc32{0};
  uint32_t read_crc32{0};
  uint64_t max_block_us{0};
  bool partition_verified{false};
  bool content_matches{false};
  bool erased{false};
};

[[nodiscard]] esp_err_t sdTest(SdTestResult &result);
[[nodiscard]] esp_err_t flashTest(FlashTestResult &result);

} // 名前空間 bringup::storage
