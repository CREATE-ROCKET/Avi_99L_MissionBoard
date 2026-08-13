#pragma once

#include <atomic>

#include "STSCREATE.h"
#include "esp_err.h"

namespace bringup {

class StsBringup {
public:
  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] bool initialized() const { return bus_.initialized(); }
  [[nodiscard]] esp_err_t initializationResult() const {
    return initialization_result_;
  }
  [[nodiscard]] esp_err_t probe();
  [[nodiscard]] esp_err_t read();
  [[nodiscard]] esp_err_t free();
  [[nodiscard]] esp_err_t hold();
  [[nodiscard]] esp_err_t smallMove(float delta_degrees);
  [[nodiscard]] bool busy() const { return busy_.load(); }

private:
  STSCREATE bus_{};
  esp_err_t initialization_result_{ESP_ERR_INVALID_STATE};
  std::atomic<bool> busy_{false};
};

} // 名前空間 bringup
