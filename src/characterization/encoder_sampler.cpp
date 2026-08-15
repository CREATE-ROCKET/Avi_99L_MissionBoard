#include "characterization/encoder_sampler.hpp"

#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION

#include "characterization/rate_check_stage_diagnostics.hpp"
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

static_assert(1'000'000U /
                  static_cast<std::uint32_t>(EncoderRate::Hz1000) /
                  2U ==
              500U);
static_assert(1'000'000U /
                  static_cast<std::uint32_t>(EncoderRate::Hz2000) /
                  2U ==
              250U);
static_assert(1'000'000U /
                  static_cast<std::uint32_t>(EncoderRate::Hz5000) /
                  2U ==
              100U);

void rememberOperation(esp_err_t operation, esp_err_t &first) noexcept {
  if (first == ESP_OK && operation != ESP_OK)
    first = operation;
}

bool hasErrorFlags(const AS5047D::ErrorFlags &flags) noexcept {
  return flags.parity_error || flags.invalid_command ||
         flags.framing_error;
}

std::uint32_t saturateU32(std::uint64_t value) noexcept {
  return static_cast<std::uint32_t>(std::min<std::uint64_t>(
      value, std::numeric_limits<std::uint32_t>::max()));
}

void updateAtomicMaximum(std::atomic<std::uint32_t> &target,
                         std::uint32_t candidate) noexcept {
  std::uint32_t current = target.load(std::memory_order_relaxed);
  while (current < candidate &&
         !target.compare_exchange_weak(current, candidate,
                                       std::memory_order_relaxed,
                                       std::memory_order_relaxed)) {
  }
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

bool EncoderSampler::timerCallback(gptimer_handle_t,
                                   const gptimer_alarm_event_data_t *event,
                                   void *context) {
  auto &sampler = *static_cast<EncoderSampler *>(context);
  // GPTimer ISRから固定lifetimeのtaskへ通知するだけに留める。
  // SAFETY: task_はstatic taskでEncoderSamplerのlifetime中に削除されない。
  // ISRではSPI、heap、file I/O、blocking APIを一切呼ばない。
  if (!sampler.running_.load(std::memory_order_acquire) ||
      sampler.task_ == nullptr)
    return false;

  if (event != nullptr) {
    const std::uint64_t lateness =
        event->count_value >= event->alarm_value
            ? event->count_value - event->alarm_value
            : 0U;
    const std::uint32_t lateness_us = saturateU32(lateness);
    portENTER_CRITICAL_ISR(&sampler.timing_lock_);
    sampler.last_alarm_lateness_us_ = lateness_us;
    sampler.max_alarm_lateness_us_ =
        std::max(sampler.max_alarm_lateness_us_, lateness_us);
    portEXIT_CRITICAL_ISR(&sampler.timing_lock_);
  }

  BaseType_t task_awoken = pdFALSE;
  vTaskNotifyGiveFromISR(sampler.task_, &task_awoken);
  return task_awoken == pdTRUE;
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
    if (now_us < first_sample_us_)
      continue;
    const std::uint64_t ideal_slot =
        (now_us - first_sample_us_) / period_us_;
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
        (std::numeric_limits<std::uint64_t>::max() - first_sample_us_) /
            period_us_) {
      rememberFirst(ESP_ERR_INVALID_SIZE);
      running_.store(false);
      continue;
    }
    next_scheduled_us_ = first_sample_us_ + ideal_slot * period_us_;

    if (now_us >= next_scheduled_us_) {
      const std::uint32_t task_wake_lateness_us =
          saturateU32(now_us - next_scheduled_us_);
      std::uint32_t alarm_lateness_us = 0U;
      portENTER_CRITICAL(&timing_lock_);
      alarm_lateness_us = last_alarm_lateness_us_;
      portEXIT_CRITICAL(&timing_lock_);
      const std::uint32_t isr_to_task_us =
          task_wake_lateness_us >= alarm_lateness_us
              ? task_wake_lateness_us - alarm_lateness_us
              : 0U;
      updateAtomicMaximum(max_isr_to_task_us_, isr_to_task_us);
    }

    if (!running_.load()) {
      acknowledgeStop();
      continue;
    }
    bringup::EncoderSample captured{};
    const bool timing_enabled =
        rate_check_timing_enabled_.load(std::memory_order_acquire);
    const std::int64_t read_started_us =
        timing_enabled ? esp_timer_get_time() : 0;
    const esp_err_t read_result = encoder_.readPipelined(captured);
    if (timing_enabled && read_started_us > 0) {
      const std::int64_t read_finished_us = esp_timer_get_time();
      if (read_finished_us >= read_started_us) {
        max_read_duration_us_ = std::max(
            max_read_duration_us_,
            saturateU32(static_cast<std::uint64_t>(read_finished_us -
                                                   read_started_us)));
      }
    }

    RawEncoderSample sample{};
    sample.generation = ++generation_;
    sample.scheduled_timestamp_us = next_scheduled_us_;
    sample.capture_timestamp_us =
        captured.host_timestamp_us != 0U
            ? captured.host_timestamp_us
            : static_cast<std::uint64_t>(
                  std::max<std::int64_t>(esp_timer_get_time(), 0));
    if (sample.capture_timestamp_us >= sample.scheduled_timestamp_us) {
      updateAtomicMaximum(
          max_capture_lateness_us_,
          saturateU32(sample.capture_timestamp_us -
                      sample.scheduled_timestamp_us));
    }
    sample.angle_raw = captured.angle_raw;
    AS5047D::ErrorFlags errors{};
    AS5047D::Status status{};
    status.offset_compensation_finished =
        startup_status_.offset_compensation_finished;

    if (read_result != ESP_OK) {
      // readPipelined自身が返したtransport/EF異常はそのsampleで即時fatalにする。
      // driverはEF処理時にERRFLをclearするため、保持済みerror flagsも証拠化する。
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
    }

    // Vault仕様どおり、capture中はdiagnostic目的でpipelineを定期停止しない。
    // 5 kHzでは1 msごとのstop/status/ERRFL/restart自体が200 us sampleを
    // coalesceさせ得るため、healthはbegin前とstop後で確認する。
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
  const std::uint32_t period = periodUs(rate);
  if (running_.load() || !isSupportedEncoderRate(rate) ||
      epoch_zero_us < period ||
      epoch_zero_us >
          std::numeric_limits<std::uint64_t>::max() - period / 2U)
    return ESP_ERR_INVALID_ARG;

  queue_.reset();
  statistics_ = {};
  first_error_.store(ESP_OK);
  stop_cleanup_error_.store(ESP_OK);
  portENTER_CRITICAL(&timing_lock_);
  last_alarm_lateness_us_ = 0U;
  max_alarm_lateness_us_ = 0U;
  portEXIT_CRITICAL(&timing_lock_);
  max_isr_to_task_us_.store(0U, std::memory_order_relaxed);
  max_capture_lateness_us_.store(0U, std::memory_order_relaxed);
  drain_timing_started_us_ = 0U;
  drain_timing_active_ = false;
  rate_check_timing_enabled_.store(rateCheckStageDiagnosticsActive(),
                                   std::memory_order_release);
  max_read_duration_us_ = 0U;
  generation_ = 0U;
  period_us_ = period;
  samples_per_epoch_ = expectedSamplesPerEpoch(rate);
  epoch_zero_us_ = epoch_zero_us;
  // raw sampleを各slot中央へ置き、1 ms epoch終端のconsumer releaseと
  // encoder taskを同時刻に起こさない。epoch所属はcapture timestampで
  // 従来どおり半開区間へ厳密に割り当てる。
  first_sample_us_ = epoch_zero_us_ + period_us_ / 2U;
  next_scheduled_us_ = first_sample_us_ - period_us_;
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
  if (result == ESP_OK && hasErrorFlags(startup_errors))
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
  if (!timer_.initialized()) {
    result = timer_.initialize(timerCallback, this);
    if (result != ESP_OK) {
      (void)stop();
      return result;
    }
  }

  // GPTimerのfirst alarmをslot中心の絶対時刻へ直接設定する。
  // 相対start APIの呼出し遅延をsampling phaseへ混入させない。
  running_.store(true, std::memory_order_release);
  result = timer_.start(first_sample_us_, period_us_);
  if (result != ESP_OK) {
    running_.store(false, std::memory_order_release);
    (void)stop();
  }
  return result;
}

esp_err_t EncoderSampler::stop() {
  esp_err_t diagnostic_error = first_error_.load();
  esp_err_t cleanup_error = ESP_OK;
  running_.store(false, std::memory_order_release);
  rememberOperation(timer_.stop(), cleanup_error);
  if (task_ != nullptr && stop_ack_ != nullptr) {
    while (xSemaphoreTake(stop_ack_, 0U) == pdTRUE) {
    }
    stop_waiting_.store(true);
    xTaskNotifyGive(task_);
    // sampler taskがnotificationをdrainしencoder accessを終えるまでdriverを閉じない。
    if (xSemaphoreTake(stop_ack_, portMAX_DELAY) != pdTRUE)
      rememberOperation(ESP_FAIL, cleanup_error);
  }

  if (rate_check_timing_enabled_.load(std::memory_order_acquire))
    recordRateCheckStageDuration(RateCheckStage::EncoderRead,
                                 max_read_duration_us_);
  rate_check_timing_enabled_.store(false, std::memory_order_release);
  drain_timing_active_ = false;
  drain_timing_started_us_ = 0U;

  if (pipeline_running_) {
    const esp_err_t pipeline_result = encoder_.stopPipelinedRead();
    rememberOperation(pipeline_result, cleanup_error);
    rememberOperation(pipeline_result, diagnostic_error);
    pipeline_running_ = false;
  }
  if (encoder_.initialized()) {
    // capture中はpipelineを止めない代わりに、終了後のhealthを必ず取得する。
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

EncoderTimingDiagnostics EncoderSampler::timingDiagnostics() const noexcept {
  EncoderTimingDiagnostics diagnostics{};
  portENTER_CRITICAL(&timing_lock_);
  diagnostics.max_alarm_lateness_us = max_alarm_lateness_us_;
  portEXIT_CRITICAL(&timing_lock_);
  diagnostics.max_isr_to_task_us =
      max_isr_to_task_us_.load(std::memory_order_relaxed);
  diagnostics.max_capture_lateness_us =
      max_capture_lateness_us_.load(std::memory_order_relaxed);
  return diagnostics;
}

bool EncoderSampler::pop(RawEncoderSample &sample) noexcept {
  const bool popped = queue_.pop(sample);
  if (!rate_check_timing_enabled_.load(std::memory_order_acquire) ||
      !running_.load(std::memory_order_acquire)) {
    drain_timing_active_ = false;
    drain_timing_started_us_ = 0U;
    return popped;
  }

  if (popped) {
    if (!drain_timing_active_) {
      drain_timing_started_us_ = rateCheckStageNowUs();
      drain_timing_active_ = drain_timing_started_us_ != 0U;
    }
  } else if (drain_timing_active_) {
    const std::uint64_t finished_us = rateCheckStageNowUs();
    if (finished_us >= drain_timing_started_us_) {
      recordRateCheckStageDuration(RateCheckStage::EncoderDrain,
                                   finished_us - drain_timing_started_us_);
    }
    drain_timing_active_ = false;
    drain_timing_started_us_ = 0U;
  }
  return popped;
}

} // 名前空間 avi::characterization

#endif
