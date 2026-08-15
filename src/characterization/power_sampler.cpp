#include "characterization/power_sampler.hpp"

#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION

#include "bringup/power_bringup.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace avi::characterization {

void PowerSampler::rememberFirst(esp_err_t error) noexcept {
  if (error == ESP_OK)
    return;
  esp_err_t expected = ESP_OK;
  if (first_error_.compare_exchange_strong(expected, error)) {
    const TaskHandle_t task = failure_notification_task_.load();
    if (task != nullptr)
      xTaskNotifyGive(task);
  }
}

void PowerSampler::taskEntry(void *context) {
  static_cast<PowerSampler *>(context)->taskLoop();
}

void PowerSampler::taskLoop() {
  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (running_.load()) {
      if (!running_.load()) {
        break;
      }
      bringup::power::PowerSample sample{};
      const esp_err_t result = bringup::power::read(sample);
      PowerEvidence evidence{};
      evidence.capture_timestamp_us =
          sample.timestamp_us > 0
              ? static_cast<std::uint64_t>(sample.timestamp_us)
              : 0U;
      const bool valid =
          result == ESP_OK && sample.motor.calibrated_valid &&
          std::isfinite(sample.motor.source_voltage_v) &&
          sample.motor.source_voltage_v > 0.0F;
      evidence.read_result =
          valid ? ESP_OK
                : (result == ESP_OK ? ESP_ERR_INVALID_RESPONSE : result);
      evidence.valid = valid;
      if (valid) {
        evidence.motor_millivolts =
            static_cast<std::uint16_t>(std::clamp<long>(
                std::lround(sample.motor.source_voltage_v * 1'000.0F),
                1L, std::numeric_limits<std::uint16_t>::max()));
      } else {
        rememberFirst(evidence.read_result);
      }
      if (!queue_.push(evidence)) {
        rememberFirst(ESP_ERR_NO_MEM);
        running_.store(false);
      }
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
    if (stop_waiting_.exchange(false)) {
      while (ulTaskNotifyTake(pdTRUE, 0U) != 0U) {
      }
      (void)xSemaphoreGive(stop_ack_);
    }
  }
}

esp_err_t PowerSampler::begin() {
  if (running_.load() || !bringup::power::initialized())
    return ESP_ERR_INVALID_STATE;
  queue_.reset();
  latest_ = {};
  have_latest_ = false;
  first_error_.store(ESP_OK);
  if (task_ == nullptr) {
    // task/stackはobjectのstatic lifetime中保持し、ADCはこのtaskだけが読む。
    task_ = xTaskCreateStaticPinnedToCore(
        taskEntry, "char_vbus", sizeof(task_stack_), this, 12,
        task_stack_, &task_tcb_, 0);
    if (task_ == nullptr)
      return ESP_ERR_NO_MEM;
  }
  if (stop_ack_ == nullptr) {
    stop_ack_ = xSemaphoreCreateBinaryStatic(&stop_ack_storage_);
    if (stop_ack_ == nullptr)
      return ESP_ERR_NO_MEM;
  }
  running_.store(true);
  xTaskNotifyGive(task_);
  return ESP_OK;
}

esp_err_t PowerSampler::stop() noexcept {
  running_.store(false);
  if (task_ != nullptr && stop_ack_ != nullptr) {
    while (xSemaphoreTake(stop_ack_, 0U) == pdTRUE) {
    }
    stop_waiting_.store(true);
    xTaskNotifyGive(task_);
    // ADC taskがreadを完了しnotificationをdrainするまで次runを開始しない。
    if (xSemaphoreTake(stop_ack_, portMAX_DELAY) != pdTRUE)
      return ESP_FAIL;
  }
  return first_error_.load();
}

bool PowerSampler::latest(std::uint64_t snapshot_us,
                          PowerEvidence &evidence) noexcept {
  PowerEvidence candidate{};
  while (queue_.pop(candidate)) {
    latest_ = candidate;
    have_latest_ = true;
  }
  if (!have_latest_) {
    evidence = {};
    evidence.read_result = ESP_ERR_NOT_FOUND;
    return false;
  }
  evidence = latest_;
  if (evidence.capture_timestamp_us == 0U ||
      evidence.capture_timestamp_us > snapshot_us ||
      snapshot_us - evidence.capture_timestamp_us > 100'000U) {
    evidence.valid = false;
    evidence.read_result = evidence.capture_timestamp_us > snapshot_us
                               ? ESP_ERR_INVALID_RESPONSE
                               : ESP_ERR_TIMEOUT;
  }
  return evidence.valid;
}

} // 名前空間 avi::characterization

#endif
