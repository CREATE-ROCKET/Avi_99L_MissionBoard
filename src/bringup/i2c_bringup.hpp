#pragma once

#include <atomic>

#include "esp_err.h"

namespace bringup {

class I2cBringup {
public:
  [[nodiscard]] esp_err_t probe();
  [[nodiscard]] bool busy() const { return busy_.load(); }

private:
  std::atomic<bool> busy_{false};
};

} // 名前空間 bringup
