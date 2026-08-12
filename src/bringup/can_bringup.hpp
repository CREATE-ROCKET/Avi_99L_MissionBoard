#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

namespace bringup {

class CanBringup {
public:
  [[nodiscard]] esp_err_t test();
  [[nodiscard]] esp_err_t loadTest(uint32_t frequency_hz,
                                   uint32_t duration_seconds);
  [[nodiscard]] bool busy() const { return busy_.load(); }

private:
  std::atomic<bool> busy_{false};
};

} // 名前空間 bringup
