#pragma once

#if defined(ESP_PLATFORM)

#include "driver/gptimer.h"
#include "esp_err.h"
#include "esp_timer.h"

#include <cstdint>
#include <limits>

namespace avi::characterization {

// esp_timer_get_time() と同じ 1 us 単位へ GPTimer を同期し、
// 最初の alarm を絶対時刻で指定した後は hardware auto-reload で周期を維持する。
// 初期化・同期・停止は task context だけで行い、ISR callback は利用側が登録する。
class AbsolutePeriodicTimer {
public:
  AbsolutePeriodicTimer() = default;
  AbsolutePeriodicTimer(const AbsolutePeriodicTimer &) = delete;
  AbsolutePeriodicTimer &operator=(const AbsolutePeriodicTimer &) = delete;

  [[nodiscard]] esp_err_t initialize(gptimer_alarm_cb_t callback,
                                     void *user_context) noexcept {
    if (callback == nullptr)
      return ESP_ERR_INVALID_ARG;
    if (timer_ != nullptr) {
      return callback_ == callback && user_context_ == user_context
                 ? ESP_OK
                 : ESP_ERR_INVALID_STATE;
    }

    callback_ = callback;
    user_context_ = user_context;

    gptimer_config_t config{};
    config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    config.direction = GPTIMER_COUNT_UP;
    config.resolution_hz = kResolutionHz;
    config.intr_priority = 0;

    esp_err_t result = gptimer_new_timer(&config, &timer_);
    if (result != ESP_OK) {
      callback_ = nullptr;
      user_context_ = nullptr;
      return result;
    }

    gptimer_event_callbacks_t callbacks{};
    callbacks.on_alarm = alarmCallback;
    result = gptimer_register_event_callbacks(timer_, &callbacks, this);
    if (result != ESP_OK) {
      (void)gptimer_del_timer(timer_);
      timer_ = nullptr;
      callback_ = nullptr;
      user_context_ = nullptr;
      return result;
    }

    result = gptimer_enable(timer_);
    if (result != ESP_OK) {
      (void)gptimer_del_timer(timer_);
      timer_ = nullptr;
      callback_ = nullptr;
      user_context_ = nullptr;
      return result;
    }
    enabled_ = true;
    return ESP_OK;
  }

  [[nodiscard]] esp_err_t start(std::uint64_t first_alarm_us,
                                std::uint32_t period_us) noexcept {
    if (timer_ == nullptr || !enabled_ || running_)
      return ESP_ERR_INVALID_STATE;
    if (period_us == 0U || first_alarm_us <= period_us)
      return ESP_ERR_INVALID_ARG;

    const std::int64_t current = esp_timer_get_time();
    if (current < 0 || first_alarm_us <= static_cast<std::uint64_t>(current))
      return ESP_ERR_INVALID_ARG;

    gptimer_alarm_config_t alarm{};
    alarm.alarm_count = first_alarm_us;
    alarm.reload_count = first_alarm_us - period_us;
    alarm.flags.auto_reload_on_alarm = true;
    esp_err_t result = gptimer_set_alarm_action(timer_, &alarm);
    if (result != ESP_OK)
      return result;

    period_us_ = period_us;
    result = gptimer_start(timer_);
    if (result != ESP_OK) {
      period_us_ = 0U;
      return result;
    }
    running_ = true;

    result = synchronizeToEspTimer();
    if (result == ESP_OK) {
      const std::int64_t synchronized_now = esp_timer_get_time();
      if (synchronized_now < 0 ||
          first_alarm_us <= static_cast<std::uint64_t>(synchronized_now))
        result = ESP_ERR_TIMEOUT;
    }
    if (result != ESP_OK) {
      const esp_err_t stop_result = gptimer_stop(timer_);
      running_ = false;
      period_us_ = 0U;
      return stop_result == ESP_OK ? result : stop_result;
    }
    return ESP_OK;
  }

  [[nodiscard]] esp_err_t stop() noexcept {
    if (timer_ == nullptr || !enabled_)
      return ESP_OK;
    esp_err_t result = ESP_OK;
    if (running_) {
      result = gptimer_stop(timer_);
      running_ = false;
    }
    period_us_ = 0U;
    const esp_err_t alarm_result = gptimer_set_alarm_action(timer_, nullptr);
    return result == ESP_OK ? alarm_result : result;
  }

  [[nodiscard]] bool initialized() const noexcept { return timer_ != nullptr; }
  [[nodiscard]] bool running() const noexcept { return running_; }

private:
  static bool alarmCallback(gptimer_handle_t timer,
                            const gptimer_alarm_event_data_t *event,
                            void *context) noexcept {
    auto &self = *static_cast<AbsolutePeriodicTimer *>(context);
    if (self.callback_ == nullptr)
      return false;
    if (event == nullptr || self.period_us_ == 0U ||
        event->alarm_value < self.period_us_) {
      return self.callback_(timer, event, self.user_context_);
    }

    gptimer_alarm_event_data_t normalized = *event;
    const std::uint64_t reload_value =
        event->alarm_value - static_cast<std::uint64_t>(self.period_us_);
    std::uint64_t lateness_us = 0U;

    // ESP-IDF 6.0.1のGPTimerはauto-reload alarm発生後のcounterをISRでcaptureする。
    // 通常はreload_valueからISR captureまで進んだcountがalarm遅延となる。
    // countがalarm_value以上の場合はpre-reload captureとの両実装を許容し、
    // その差を使用する。1周期以上のISR遅延は既存schedule validationで失敗扱いになる。
    if (event->count_value >= event->alarm_value) {
      lateness_us = event->count_value - event->alarm_value;
    } else if (event->count_value >= reload_value) {
      lateness_us = event->count_value - reload_value;
    }

    if (lateness_us >
        std::numeric_limits<std::uint64_t>::max() - event->alarm_value) {
      normalized.count_value = std::numeric_limits<std::uint64_t>::max();
    } else {
      normalized.count_value = event->alarm_value + lateness_us;
    }
    return self.callback_(timer, &normalized, self.user_context_);
  }

  [[nodiscard]] esp_err_t synchronizeToEspTimer() noexcept {
    // set_raw_count() が長くpreemptされた場合はその試行を採用せず再同期する。
    // 5 kHz slot中心から境界まで100 usなので、25 us以内のclock offsetを要求する。
    constexpr std::uint64_t kMaximumSyncBracketUs = 50U;
    constexpr std::uint64_t kMaximumClockOffsetUs = 25U;
    constexpr std::uint32_t kMaximumAttempts = 8U;

    for (std::uint32_t attempt = 0; attempt < kMaximumAttempts; ++attempt) {
      const std::int64_t set_time = esp_timer_get_time();
      if (set_time < 0)
        return ESP_FAIL;
      esp_err_t result = gptimer_set_raw_count(
          timer_, static_cast<std::uint64_t>(set_time));
      if (result != ESP_OK)
        return result;

      const std::int64_t before = esp_timer_get_time();
      if (before < 0)
        return ESP_FAIL;
      std::uint64_t raw = 0U;
      result = gptimer_get_raw_count(timer_, &raw);
      const std::int64_t after = esp_timer_get_time();
      if (result != ESP_OK)
        return result;
      if (after < before)
        continue;

      const std::uint64_t bracket =
          static_cast<std::uint64_t>(after - before);
      if (bracket > kMaximumSyncBracketUs)
        continue;
      const std::uint64_t midpoint =
          static_cast<std::uint64_t>(before) + bracket / 2U;
      const std::uint64_t offset =
          midpoint >= raw ? midpoint - raw : raw - midpoint;
      if (offset <= kMaximumClockOffsetUs)
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
  }

  static constexpr std::uint32_t kResolutionHz = 1'000'000U;
  gptimer_handle_t timer_{nullptr};
  gptimer_alarm_cb_t callback_{nullptr};
  void *user_context_{nullptr};
  std::uint32_t period_us_{0U};
  bool enabled_{false};
  bool running_{false};
};

} // 名前空間 avi::characterization

#endif
