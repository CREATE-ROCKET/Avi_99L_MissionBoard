#pragma once

#include "esp_err.h"

namespace runtime {

class ProductionRuntime {
public:
  explicit ProductionRuntime(bool recovery_only = false,
                             bool recovery_wake_valid = false)
      : recovery_only_(recovery_only),
        recovery_wake_valid_(recovery_wake_valid) {}
  [[nodiscard]] esp_err_t start();
  [[nodiscard]] bool flightEnabled() const { return flight_enabled_; }

private:
  bool flight_enabled_{};
  bool recovery_only_{};
  bool recovery_wake_valid_{};
};

} // 名前空間 runtime
