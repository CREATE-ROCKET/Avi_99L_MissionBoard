#pragma once

#include "control/control_pipeline.hpp"
#include "esp_err.h"

namespace actuators {

class ProductionMotorDriver {
public:
  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] esp_err_t apply(const control::MotorCommand &command);
  [[nodiscard]] esp_err_t brake();
  [[nodiscard]] esp_err_t coast();
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  [[nodiscard]] esp_err_t setChannelDuty(bool positive_in1, double duty);

  bool initialized_{};
  bool timer_configured_{};
  bool in1_configured_{};
  bool in2_configured_{};
};

} // 名前空間 actuators
