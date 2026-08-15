#include "actuators/production_motor.hpp"

#include <algorithm>
#include <cmath>

#include "bringup/safe_outputs.hpp"
#include "config/board_config.hpp"
#include "config/flight_config.hpp"
#include "driver/ledc.h"

namespace actuators {
namespace {

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kIn1Channel = LEDC_CHANNEL_0;
constexpr ledc_channel_t kIn2Channel = LEDC_CHANNEL_1;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kMaximumDutyCount = (1U << 10U) - 1U;

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

} // 無名名前空間

esp_err_t ProductionMotorDriver::initialize() {
  if (initialized_)
    return ESP_OK;
  if (!bringup::safe_outputs::initialized())
    return ESP_ERR_INVALID_STATE;

  coast_known_ = false;
  ledc_timer_config_t timer{};
  timer.speed_mode = kLedcMode;
  timer.duty_resolution = kDutyResolution;
  timer.timer_num = kLedcTimer;
  timer.freq_hz = board::kMotorPwmFrequencyHz;
  timer.clk_cfg = LEDC_USE_APB_CLK;
  esp_err_t result = ledc_timer_config(&timer);
  if (result != ESP_OK) {
    (void)coast();
    return result;
  }
  timer_configured_ = true;

  ledc_channel_config_t in1{};
  in1.gpio_num = board::kMotorIn1;
  in1.speed_mode = kLedcMode;
  in1.channel = kIn1Channel;
  in1.timer_sel = kLedcTimer;
  in1.duty = 0;
  in1.hpoint = 0;
  in1.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
  result = ledc_channel_config(&in1);
  if (result != ESP_OK) {
    (void)coast();
    return result;
  }
  in1_configured_ = true;

  ledc_channel_config_t in2 = in1;
  in2.gpio_num = board::kMotorIn2;
  in2.channel = kIn2Channel;
  result = ledc_channel_config(&in2);
  if (result != ESP_OK) {
    (void)coast();
    return result;
  }
  in2_configured_ = true;
  initialized_ = true;
  result = coast();
  if (result != ESP_OK)
    initialized_ = false;
  return result;
}

esp_err_t
ProductionMotorDriver::apply(const control::MotorCommand &command) {
  if (!initialized_ || !command.valid || command.brake ||
      !std::isfinite(command.pwm_duty) || command.pwm_duty < 0.0 ||
      command.pwm_duty > 1.0) {
    const esp_err_t fallback = coast();
    return fallback == ESP_OK ? ESP_ERR_INVALID_ARG : fallback;
  }
  const double duty = std::min(command.pwm_duty,
                               flight_config::kProductionMotorMaximumDuty);
  return setChannelDuty(command.positive_in1, duty);
}

esp_err_t ProductionMotorDriver::setChannelDuty(bool positive_in1,
                                                 double duty) {
  const auto active = positive_in1 ? kIn1Channel : kIn2Channel;
  const auto inactive = positive_in1 ? kIn2Channel : kIn1Channel;
  const uint32_t count = static_cast<uint32_t>(
      std::lround(std::clamp(duty, 0.0, 1.0) * kMaximumDutyCount));
  esp_err_t result = ledc_set_duty(kLedcMode, inactive, 0);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, inactive);
  if (result == ESP_OK)
    result = ledc_set_duty(kLedcMode, active, count);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, active);
  if (result == ESP_OK) {
    coast_known_ = false;
  } else {
    rememberFirst(coast(), result);
  }
  return result;
}

esp_err_t ProductionMotorDriver::brake() {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  // TODO(HW_TEST): TB67H450と実機motorで両入力HIGH brakeの電流・温度を確認する。
  esp_err_t result = ledc_set_duty(kLedcMode, kIn1Channel,
                                   kMaximumDutyCount);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, kIn1Channel);
  if (result == ESP_OK)
    result = ledc_set_duty(kLedcMode, kIn2Channel,
                           kMaximumDutyCount);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, kIn2Channel);
  if (result == ESP_OK) {
    coast_known_ = false;
  } else {
    rememberFirst(coast(), result);
  }
  return result;
}

esp_err_t ProductionMotorDriver::coast() {
#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION
  // rate-checkは同じCoastを1 kHzで再要求する。initialize()または直前の
  // coast()でHi-Z化に成功しており、その後Drive/Brakeしていない場合は
  // LEDC stopを再発行しない。production flight buildの挙動は変更しない。
  if (initialized_ && coast_known_)
    return ESP_OK;
#endif

  esp_err_t result = ESP_OK;
  if (timer_configured_ && in1_configured_)
    rememberFirst(ledc_stop(kLedcMode, kIn1Channel, 0), result);
  if (timer_configured_ && in2_configured_)
    rememberFirst(ledc_stop(kLedcMode, kIn2Channel, 0), result);
  rememberFirst(bringup::safe_outputs::motorCoast(), result);
  coast_known_ = result == ESP_OK;
  return result;
}

} // 名前空間 actuators
