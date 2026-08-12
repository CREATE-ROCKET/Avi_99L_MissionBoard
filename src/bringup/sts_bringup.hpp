#pragma once

#include <atomic>

#include "esp_err.h"

namespace bringup {

class StsBringup {
public:
  [[nodiscard]] esp_err_t probe();
  [[nodiscard]] esp_err_t read();
  [[nodiscard]] esp_err_t free();
  [[nodiscard]] esp_err_t hold();
  [[nodiscard]] esp_err_t smallMove(float delta_degrees);
  [[nodiscard]] bool busy() const { return busy_.load(); }

private:
  std::atomic<bool> busy_{false};
};

} // 名前空間 bringup
