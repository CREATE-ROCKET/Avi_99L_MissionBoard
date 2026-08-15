#include "characterization/encoder_sampler.hpp"

#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors/as5047d_health.hpp"

#include <algorithm>
#include <limits>

namespace avi::characterization {
namespace {

std::uint32_t periodUs(EncoderRate rate) noexcept {
  return 1'000'000U / static_cast<std::uint32_t>(rate);
}

void rememberOperation(esp_err_t operation, esp_err_t &first) noexcept {
  if (first == ESP_OK && operation != ESP_OK)
    first = operation;
}

bool hasErrorFlags(const AS5047D::ErrorFlags &flags) noexcept {
  return flags.parity_error || flags.invalid_command ||
         flags.framing_error;
}

} // 無名名前空間

void EncoderSampler::rememberFirst(esp_err_t error) noexcept {
  if (error == ESP_OK)
    return;
  esp_err_t expected = ESP_OK;
  if (first_error_.compare_exchange_strong(expected, error)) {
    statistics_.first_error = error;
    const TaskHandle_t task = failure_notification_task_.load();
    if (task != nullptr)
      xTaskNotifyGive(task);
  }
}

std::uint16_t EncoderSampler::statusFlags(
    const AS5047D::Status &status,
    const AS5047D::ErrorFlags &errors) const noexcept {
  std::uint16_t flags = 0U;
  if (errors.parity_error)
    flags |= DiagnosticParityError;
  if (errors.invalid_command)
    flags |= DiagnosticInvalidCommand;
  if (errors.framing_error)
    flags |= DiagnosticFramingError;
  if (status.magnetic_too_low)
    flags |= DiagnosticMagneticTooLow;
  if (status.magnetic_too_high)
    flags |= DiagnosticMagneticTooHigh;
  if (status.cordic_overflow)
    flags |= DiagnosticCordicOverflow;
  if (status.offset_compensation_finished)
    flags |= DiagnosticOffsetCompensationFinished;
  return flags;
}

void EncoderSampler::timerCallback(void *context) {
  auto &sampler = *static_cast<EncoderSampler *>(context);
  // callbackは固定lifetimeのtask handleへ通知するだけで、SPI/fileへ触れない。
  if (sampler.running_.load() && sampler.task_ != nullptr)
    xTaskNotifyGive(sampler.task_);
}

void EncoderSampler::taskEntry(void *context) {
  static_cast<EncoderSampler *>(context)->taskLoop();
}

void EncoderSampler::acknowledgeStop() noexcept {
  if (!stop_waiting_.exchange(false))
    return;
  while (ulTaskNotifyTake(pdTRUE, 0U) != 0U) {
  }
  (void)xSemaphoreGive(stop_ack_);
}

void EncoderSampler::taskLoop() {
  for (;;) {
    const std::uint32_t notifications =
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!running_.load() || notifications == 0U) {
      acknowledgeStop();
      continue;
    }

    const std::uint64_t now_us = static_cast<std::uint64_t>(
        std::max<std::int64_t>(esp_timer_get_time(), 0));
    if (now_us < epoch_zero_us_)
      continue;
    const std::uint64_t ideal_slot =
        (now_us - epoch_zero_us_) / period_us_;
    if (have_sampled_slot_ && ideal_slot <= last_sampled_slot_) {
      statistics_.trigger_coalesced_or_missed += notifications;
      continue;
    }
    const std::uint64_t elapsed_missed =
        have_sampled_slot_ ? ideal_slot - last_sampled_slot_ - 1U
                           : ideal_slot;
    const std::uint64_t notification_missed =
        notifications > 1U ? notifications - 1U : 0U;
    statistics_.trigger_coalesced_or_missed +=
        std::max(elapsed_missed, notification_missed);
    last_sampled_slot_ = ideal_slot;
    have_sampled_slot_ = true;
    if (ideal_slot >
        (std::numeric_limits<std::uint64_t>::max() - epoch_zero_us_) /
            period_us_) {
      rememberFirst(ESP_ERR_INVALID_SIZE);
      running_.store(false);
      continue;
    }
    next_scheduled_us_ = epoch_zero_us_ + ideal_slot * period_us_;

    if (!running_.load()) {
      acknowledgeStop();
      continue;
    }
    bringup::EncoderSample captured{};
    esp_err_t read_result = encoder_.readPipelined(captured);

    RawEncoderSample sample{};
    sample.generation = ++generation_;
    sample.scheduled_timestamp_us = next_scheduled_us_;
    sample.capture_timestamp_us =
        captured.host_timestamp_us != 0U
            ? captured.host_timestamp_us
            : static_cast<std::uint64_t>(
                  std::max<std::int64_t>(esp_timer_get_time(), 0));
    sample.angle_raw = captured.angle_raw;
    AS5047D::ErrorFlags errors{};
    AS5047D::Status status{};
    status.offset_compensation_finished =
        startup_status_.offset_compensation_finished;
    if (read_result != ESP_OK) {
      // driverはEF処理時にERRFLを既にclearするため、保持済み証拠を読む。
      pipeline_running_ = false;
      ++statistics_.encoder_transport_errors;
      rememberFirst(read_result);
      errors = encoder_.lastErrorFlags();
      if (read_result == ESP_ERR_INVALID_CRC)
        errors.parity_error = true;
      const esp_err_t status_result = encoder_.getStatus(status);
      if (status_result != ESP_OK) {
        ++statistics_.encoder_transport_errors;
        rememberFirst(status_result);
      }
      if (sensors::as5047d_health::statusFaulted(status)) {
        ++statistics_.encoder_status_faults;
        rememberFirst(ESP_ERR_INVALID_STATE);
      }
      running_.store(false);
    } else if (samples_per_epoch_ != 0U &&
               ideal_slot % samples_per_epoch_ ==
                   samples_per_epoch_ - 1U) {
      // 各1 ms epoch末に同じsampler ownerがsensor healthを確認する。
      esp_err_t health_result = encoder_.stopPipelinedRead();
      pipeline_running_ = false;
      if (health_result != ESP_OK)
        ++statistics_.encoder_transport_errors;

      esp_err_t status_result = ESP_ERR_NOT_FINISHED;
      esp_err_t flags_result = ESP_ERR_NOT_FINISHED;
      if (health_result == ESP_OK) {
        status_result = encoder_.getStatus(status);
        if (status_result != ESP_OK) {
          ++statistics_.encoder_transport_errors;
          health_result = status_result;
        }
      }
      if (status_result == ESP_OK &&
          sensors::as5047d_health::statusFaulted(status)) {
        ++statistics_.encoder_status_faults;
        health_result = ESP_ERR_INVALID_STATE;
      }
      if (status_result == ESP_OK) {
        flags_result = encoder_.readAndClearErrorFlags(errors);
        if (flags_result != ESP_OK) {
          ++statistics_.encoder_transport_errors;
          health_result = flags_result;
        } else if (hasErrorFlags(errors)) {
          ++statistics_.encoder_transport_errors;
          health_result = ESP_ERR_INVALID_RESPONSE;
        }
      }
      if (health_result == ESP_OK) {
        const esp_err_t restart_result = encoder_.startPipelinedRead();
        if (restart_result == ESP_OK) {
          pipeline_running_ = true;
          startup_status_ = status;
        } else {
          ++statistics_.encoder_transport_errors;
          health_result = restart_result;
        }
      }
      if (health_result != ESP_OK) {
        read_result = health_result;
        rememberFirst(health_result);
        running_.store(false);
      }
    }
    sample.read_result_code = read_result;
    sample.valid = read_result == ESP_OK && captured.valid;
    sample.diagnostic_flags = statusFlags(status, errors);
    if (!sample.valid)
      ++statistics_.invalid_samples;
    if (!queue_.push(sample)) {
      ++statistics_.raw_queue_overflows;
      rememberFirst(ESP_ERR_NO_MEM);
      running_.store(false);
    }
    if (!running_.load())
      acknowledgeStop();
  }
}

esp_err_t EncoderSampler::begin(EncoderRate rate,
                                std::uint64_t epoch_zero_us) {
  if (running_.load() || !isSupportedEncoderRate(rate) ||
      epoch_zero_us < periodUs(rate))
    return ESP_ERR_INVALID_ARG;

  queue_.reset();
  statistics_ = {};
  first_error_.store(ESP_OK);
  stop_cleanup_error_.store(ESP_OK);
  generation_ = 0U;
  period_us_ = periodUs(rate);
  samples_per_epoch_ = expectedSamplesPerEpoch(rate);
  epoch_zero_us_ = epoch_zero_us;
  next_scheduled_us_ = epoch_zero_us - period_us_;
  last_sampled_slot_ = 0U;
  have_sampled_slot_ = false;
  startup_status_ = {};
  pipeline_running_ = false;

  esp_err_t result = spi_.begin();
  if (result != ESP_OK)
    return result;
  result = encoder_.begin(spi_);
  if (result != ESP_OK) {
    (void)spi_.end();
    return result;
  }
  result = encoder_.getStatus(startup_status_);
  if (result == ESP_OK &&
      sensors::as5047d_health::statusFaulted(startup_status_)) {
    ++statistics_.encoder_status_faults;
    result = ESP_ERR_INVALID_STATE;
  }
  AS5047D::ErrorFlags startup_errors{};
  if (result == ESP_OK)
    result = encoder_.readAndClearErrorFlags(startup_errors);
  if (result == ESP_OK &&
      (startup_errors.parity_error || startup_errors.invalid_command ||
       startup_errors.framing_error))
    result = ESP_ERR_INVALID_RESPONSE;
  if (result == ESP_OK)
    result = encoder_.startPipelinedRead();
  if (result != ESP_OK) {
    (void)encoder_.end();
    (void)spi_.end();
    return result;
  }
  pipeline_running_ = true;

  if (task_ == nullptr) {
    // task/stackはobjectのstatic lifetime内にあり、run中に解放されない。
    task_ = xTaskCreateStaticPinnedToCore(
        taskEntry, "char_encoder", sizeof(task_stack_), this, 23,
        task_stack_, &task_tcb_, 1);
    if (task_ == nullptr) {
      (void)stop();
      return ESP_ERR_NO_MEM;
    }
  }
  if (stop_ack_ == nullptr) {
    stop_ack_ = xSemaphoreCreateBinaryStatic(&stop_ack_storage_);
    if (stop_ack_ == nullptr) {
      (void)stop();
      return ESP_ERR_NO_MEM;
    }
  }
  (void)xTaskNotifyStateClear(task_);
  (void)ulTaskNotifyValueClear(task_,
                               std::numeric_limits<std::uint32_t>::max());
  if (timer_ == nullptr) {
    esp_timer_create_args_t arguments{};
    arguments.callback = timerCallback;
    arguments.arg = this;
    arguments.dispatch_method = ESP_TIMER_TASK;
    arguments.name = "char_encoder";
    arguments.skip_unhandled_events = true;
    result = esp_timer_create(&arguments, &timer_);
    if (result != ESP_OK) {
      (void)stop();
      return result;
    }
  }

  const std::int64_t timer_start_us =
      static_cast<std::int64_t>(epoch_zero_us - period_us_);
  while (esp_timer_get_time() + 2'000 < timer_start_us)
    vTaskDelay(1U);
  while (esp_timer_get_time() < timer_start_us)
    taskYIELD();
  running_.store(true);
  result = esp_timer_start_periodic(timer_, period_us_);
  if (result != ESP_OK) {
    running_.store(false);
    (void)stop();
  }
  return result;
}

esp_err_t EncoderSampler::stop() {
  esp_err_t diagnostic_error = first_error_.load();
  esp_err_t cleanup_error = ESP_OK;
  running_.store(false);
  if (timer_ != nullptr && esp_timer_is_active(timer_))
    rememberOperation(esp_timer_stop(timer_), cleanup_error);
  if (task_ != nullptr && stop_ack_ != nullptr) {
    while (xSemaphoreTake(stop_ack_, 0U) == pdTRUE) {
    }
    stop_waiting_.store(true);
    xTaskNotifyGive(task_);
    // sampler taskがnotificationをdrainしencoder accessを終えるまでdriverを閉じない。
    if (xSemaphoreTake(stop_ack_, portMAX_DELAY) != pdTRUE)
      rememberOperation(ESP_FAIL, cleanup_error);
  }

  if (pipeline_running_) {
    const esp_err_t pipeline_result = encoder_.stopPipelinedRead();
    rememberOperation(pipeline_result, cleanup_error);
    rememberOperation(pipeline_result, diagnostic_error);
    pipeline_running_ = false;
  }
  if (encoder_.initialized()) {
    AS5047D::Status final_status{};
    const esp_err_t status_result = encoder_.getStatus(final_status);
    rememberOperation(status_result, diagnostic_error);
    if (sensors::as5047d_health::statusFaulted(final_status)) {
      ++statistics_.encoder_status_faults;
      rememberOperation(ESP_ERR_INVALID_STATE, diagnostic_error);
    }
    AS5047D::ErrorFlags final_errors{};
    const esp_err_t flags_result =
        encoder_.readAndClearErrorFlags(final_errors);
    rememberOperation(flags_result, diagnostic_error);
    if (flags_result == ESP_OK && hasErrorFlags(final_errors)) {
      ++statistics_.encoder_transport_errors;
      rememberOperation(ESP_ERR_INVALID_RESPONSE, diagnostic_error);
    }
    const esp_err_t end_result = encoder_.end();
    rememberOperation(end_result, cleanup_error);
    rememberOperation(end_result, diagnostic_error);
  }
  if (spi_.encoderBusInitialized() || spi_.imuBusInitialized()) {
    const esp_err_t spi_result = spi_.end();
    rememberOperation(spi_result, cleanup_error);
    rememberOperation(spi_result, diagnostic_error);
  }
  stop_cleanup_error_.store(cleanup_error);
  rememberFirst(diagnostic_error);
  return diagnostic_error;
}

SamplerStatistics EncoderSampler::statistics() const {
  return statistics_;
}

bool EncoderSampler::pop(RawEncoderSample &sample) noexcept {
  return queue_.pop(sample);
}

} // 名前空間 avi::characterization

#endif
