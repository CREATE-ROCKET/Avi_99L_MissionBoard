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
  // Characterizationの1 kHz rate-checkではCoastを毎epoch要求する。
  // 既にHi-Zであることがdriver自身により確認済みなら、同じLEDC停止操作を
  // 繰り返してrealtime deadlineを消費しない。
  bool coast_known_{};
};

} // 名前空間 actuators
