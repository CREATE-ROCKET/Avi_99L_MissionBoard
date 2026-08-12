#include "bringup/power_bringup.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>

#include "config/board_config.hpp"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace bringup::power {
namespace {
constexpr uint32_t kSampleRateHz = 100;
constexpr uint32_t kSamplePeriodMs = 1'000U / kSampleRateHz;
constexpr uint32_t kLogicRawValid = 1U << 0;
constexpr uint32_t kLogicCalibratedValid = 1U << 1;
constexpr uint32_t kMotorRawValid = 1U << 2;
constexpr uint32_t kMotorCalibratedValid = 1U << 3;

struct ChannelState {
  adc_channel_t channel{ADC_CHANNEL_0};
  adc_cali_handle_t calibration{nullptr};
  esp_err_t calibration_status{ESP_ERR_INVALID_STATE};
};

struct SummaryAccumulator {
  int32_t raw_min{std::numeric_limits<int32_t>::max()};
  int32_t raw_max{std::numeric_limits<int32_t>::min()};
  float pin_min{std::numeric_limits<float>::infinity()};
  float pin_max{-std::numeric_limits<float>::infinity()};
  float source_min{std::numeric_limits<float>::infinity()};
  float source_max{-std::numeric_limits<float>::infinity()};
  double pin_sum{0.0};
  double source_sum{0.0};
};

adc_oneshot_unit_handle_t adc_unit = nullptr;
ChannelState logic_channel{};
ChannelState motor_channel{};
std::atomic<bool> stream_running{false};

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

esp_err_t createCalibration(adc_unit_t unit, adc_channel_t channel,
                            adc_cali_handle_t &handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t config{};
  config.unit_id = unit;
  config.chan = channel;
  config.atten = ADC_ATTEN_DB_12;
  config.bitwidth = ADC_BITWIDTH_DEFAULT;
  return adc_cali_create_scheme_curve_fitting(&config, &handle);
#else
  (void)unit;
  (void)channel;
  (void)handle;
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t deleteCalibration(adc_cali_handle_t &handle) {
  if (handle == nullptr)
    return ESP_OK;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  const esp_err_t result = adc_cali_delete_scheme_curve_fitting(handle);
  if (result == ESP_OK)
    handle = nullptr;
  return result;
#else
  handle = nullptr;
  return ESP_OK;
#endif
}

void resetSummary(AdcChannelSummary &summary,
                  esp_err_t calibration_status) {
  summary = {};
  summary.calibration_status = calibration_status;
}

AdcReading readSample(const ChannelState &channel) {
  AdcReading sample{};
  int raw = 0;
  sample.status = adc_oneshot_read(adc_unit, channel.channel, &raw);
  if (sample.status != ESP_OK)
    return sample;

  sample.raw = raw;
  sample.raw_valid = true;
  if (channel.calibration == nullptr) {
    sample.status = channel.calibration_status;
    return sample;
  }

  int pin_mv = 0;
  sample.status =
      adc_cali_raw_to_voltage(channel.calibration, raw, &pin_mv);
  if (sample.status != ESP_OK)
    return sample;

  sample.pin_voltage_v = static_cast<float>(pin_mv) / 1'000.0F;
  sample.source_voltage_v =
      sample.pin_voltage_v * board::kVoltageDividerRatio;
  if (!std::isfinite(sample.pin_voltage_v) ||
      !std::isfinite(sample.source_voltage_v)) {
    sample.status = ESP_ERR_INVALID_RESPONSE;
    sample.pin_voltage_v = std::numeric_limits<float>::quiet_NaN();
    sample.source_voltage_v = std::numeric_limits<float>::quiet_NaN();
    return sample;
  }
  sample.calibrated_valid = true;
  return sample;
}

void accumulate(const AdcReading &sample, AdcChannelSummary &summary,
                SummaryAccumulator &accumulator) {
  if (sample.raw_valid) {
    ++summary.raw_valid_samples;
    accumulator.raw_min = std::min(accumulator.raw_min, sample.raw);
    accumulator.raw_max = std::max(accumulator.raw_max, sample.raw);
  }
  if (!sample.calibrated_valid)
    return;

  ++summary.calibrated_valid_samples;
  accumulator.pin_min =
      std::min(accumulator.pin_min, sample.pin_voltage_v);
  accumulator.pin_max =
      std::max(accumulator.pin_max, sample.pin_voltage_v);
  accumulator.source_min =
      std::min(accumulator.source_min, sample.source_voltage_v);
  accumulator.source_max =
      std::max(accumulator.source_max, sample.source_voltage_v);
  accumulator.pin_sum += sample.pin_voltage_v;
  accumulator.source_sum += sample.source_voltage_v;
}

void finalize(AdcChannelSummary &summary,
              const SummaryAccumulator &accumulator) {
  if (summary.raw_valid_samples != 0) {
    summary.raw_min = accumulator.raw_min;
    summary.raw_max = accumulator.raw_max;
  }
  if (summary.calibrated_valid_samples == 0) {
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    summary.pin_voltage_min_v = invalid;
    summary.pin_voltage_max_v = invalid;
    summary.pin_voltage_mean_v = invalid;
    summary.source_voltage_min_v = invalid;
    summary.source_voltage_max_v = invalid;
    summary.source_voltage_mean_v = invalid;
    return;
  }

  const float count = static_cast<float>(summary.calibrated_valid_samples);
  summary.pin_voltage_min_v = accumulator.pin_min;
  summary.pin_voltage_max_v = accumulator.pin_max;
  summary.pin_voltage_mean_v =
      static_cast<float>(accumulator.pin_sum / count);
  summary.source_voltage_min_v = accumulator.source_min;
  summary.source_voltage_max_v = accumulator.source_max;
  summary.source_voltage_mean_v =
      static_cast<float>(accumulator.source_sum / count);
}

esp_err_t encodeAndSend(StreamProtocol &stream, int64_t timestamp_us,
                        const AdcReading &logic, const AdcReading &motor) {
  uint8_t flags = 0;
  if (logic.raw_valid)
    flags |= kLogicRawValid;
  if (logic.calibrated_valid)
    flags |= kLogicCalibratedValid;
  if (motor.raw_valid)
    flags |= kMotorRawValid;
  if (motor.calibrated_valid)
    flags |= kMotorCalibratedValid;

  StreamPayload payload;
  const bool encoded =
      payload.u64(static_cast<uint64_t>(timestamp_us)) && payload.u8(flags) &&
      payload.i32(logic.raw) && payload.f32(logic.pin_voltage_v) &&
      payload.f32(logic.source_voltage_v) && payload.i32(logic.status) &&
      payload.i32(motor.raw) && payload.f32(motor.pin_voltage_v) &&
      payload.f32(motor.source_voltage_v) && payload.i32(motor.status);
  return encoded ? stream.send(StreamRecordType::adc, payload)
                 : ESP_ERR_INVALID_SIZE;
}
} // 無名名前空間

esp_err_t initialize() {
  if (adc_unit != nullptr)
    return ESP_OK;

  adc_unit_t logic_unit = ADC_UNIT_1;
  adc_unit_t motor_unit = ADC_UNIT_1;
  adc_channel_t logic_adc_channel = ADC_CHANNEL_0;
  adc_channel_t motor_adc_channel = ADC_CHANNEL_0;
  esp_err_t status = adc_oneshot_io_to_channel(
      board::kLogicVoltageAdc, &logic_unit, &logic_adc_channel);
  if (status != ESP_OK)
    return status;
  status = adc_oneshot_io_to_channel(
      board::kMotorVoltageAdc, &motor_unit, &motor_adc_channel);
  if (status != ESP_OK)
    return status;
  if (logic_unit != motor_unit)
    return ESP_ERR_NOT_SUPPORTED;

  adc_oneshot_unit_handle_t new_unit = nullptr;
  adc_oneshot_unit_init_cfg_t unit_config{};
  unit_config.unit_id = logic_unit;
  unit_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
  unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;
  status = adc_oneshot_new_unit(&unit_config, &new_unit);
  if (status != ESP_OK)
    return status;

  adc_oneshot_chan_cfg_t channel_config{};
  channel_config.atten = ADC_ATTEN_DB_12;
  channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;
  status = adc_oneshot_config_channel(new_unit, logic_adc_channel,
                                      &channel_config);
  if (status == ESP_OK)
    status = adc_oneshot_config_channel(new_unit, motor_adc_channel,
                                        &channel_config);
  if (status != ESP_OK) {
    (void)adc_oneshot_del_unit(new_unit);
    return status;
  }

  adc_cali_handle_t logic_calibration = nullptr;
  adc_cali_handle_t motor_calibration = nullptr;
  const esp_err_t logic_calibration_status = createCalibration(
      logic_unit, logic_adc_channel, logic_calibration);
  if (logic_calibration_status != ESP_OK &&
      logic_calibration_status != ESP_ERR_NOT_SUPPORTED) {
    (void)adc_oneshot_del_unit(new_unit);
    return logic_calibration_status;
  }
  const esp_err_t motor_calibration_status = createCalibration(
      motor_unit, motor_adc_channel, motor_calibration);
  if (motor_calibration_status != ESP_OK &&
      motor_calibration_status != ESP_ERR_NOT_SUPPORTED) {
    (void)deleteCalibration(logic_calibration);
    (void)adc_oneshot_del_unit(new_unit);
    return motor_calibration_status;
  }

  adc_unit = new_unit;
  logic_channel.channel = logic_adc_channel;
  logic_channel.calibration = logic_calibration;
  logic_channel.calibration_status = logic_calibration_status;
  motor_channel.channel = motor_adc_channel;
  motor_channel.calibration = motor_calibration;
  motor_channel.calibration_status = motor_calibration_status;
  return ESP_OK;
}

esp_err_t end() {
  if (stream_running.load())
    return ESP_ERR_INVALID_STATE;
  esp_err_t first = deleteCalibration(logic_channel.calibration);
  rememberFirst(deleteCalibration(motor_channel.calibration), first);
  if (first != ESP_OK)
    return first;
  if (adc_unit != nullptr) {
    const esp_err_t result = adc_oneshot_del_unit(adc_unit);
    if (result == ESP_OK)
      adc_unit = nullptr;
    rememberFirst(result, first);
  }
  if (first == ESP_OK) {
    logic_channel = {};
    motor_channel = {};
  }
  return first;
}

bool initialized() { return adc_unit != nullptr; }

esp_err_t read(PowerSample &sample) {
  sample = {};
  if (!initialized() || stream_running.load())
    return ESP_ERR_INVALID_STATE;

  sample.timestamp_us = esp_timer_get_time();
  sample.logic = readSample(logic_channel);
  sample.motor = readSample(motor_channel);
  if (!sample.logic.raw_valid)
    return sample.logic.status;
  if (!sample.motor.raw_valid)
    return sample.motor.status;
  if (!sample.logic.calibrated_valid)
    return sample.logic.status;
  if (!sample.motor.calibrated_valid)
    return sample.motor.status;
  return ESP_OK;
}

esp_err_t adcStream(uint32_t seconds, StreamProtocol &stream,
                    AdcStreamResult &result) {
  result = {};
  if (seconds == 0 || seconds >
                          std::numeric_limits<uint32_t>::max() / kSampleRateHz)
    return ESP_ERR_INVALID_ARG;
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  bool expected = false;
  if (!stream_running.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;

  result.requested_samples = seconds * kSampleRateHz;
  resetSummary(result.logic, logic_channel.calibration_status);
  resetSummary(result.motor, motor_channel.calibration_status);
  SummaryAccumulator logic_accumulator{};
  SummaryAccumulator motor_accumulator{};

  esp_err_t first = stream.initialize();
  if (first == ESP_OK)
    first = stream.beginCapture();
  if (first != ESP_OK) {
    stream_running.store(false);
    return first;
  }

  if (logic_channel.calibration_status != ESP_OK)
    rememberFirst(logic_channel.calibration_status, first);
  if (motor_channel.calibration_status != ESP_OK)
    rememberFirst(motor_channel.calibration_status, first);

  const int64_t started_us = esp_timer_get_time();
  TickType_t next_wake = xTaskGetTickCount();
  for (uint32_t index = 0; index < result.requested_samples; ++index) {
    const int64_t sample_started_us = esp_timer_get_time();
    const AdcReading logic = readSample(logic_channel);
    const AdcReading motor = readSample(motor_channel);
    accumulate(logic, result.logic, logic_accumulator);
    accumulate(motor, result.motor, motor_accumulator);
    if ((!logic.raw_valid ||
         (logic_channel.calibration != nullptr &&
          !logic.calibrated_valid)) ||
        (!motor.raw_valid ||
         (motor_channel.calibration != nullptr &&
          !motor.calibrated_valid))) {
      ++result.adc_error_count;
      if (logic.status != ESP_OK && logic.status != ESP_ERR_NOT_SUPPORTED)
        rememberFirst(logic.status, first);
      if (motor.status != ESP_OK && motor.status != ESP_ERR_NOT_SUPPORTED)
        rememberFirst(motor.status, first);
    }

    const esp_err_t send_status =
        encodeAndSend(stream, sample_started_us, logic, motor);
    if (send_status != ESP_OK) {
      ++result.stream_error_count;
      rememberFirst(send_status, first);
    }
    ++result.sample_count;

    const int64_t latency = esp_timer_get_time() - sample_started_us;
    if (latency > 0) {
      result.max_sample_latency_us =
          std::max(result.max_sample_latency_us,
                   static_cast<uint64_t>(latency));
      if (latency > static_cast<int64_t>(kSamplePeriodMs) * 1'000)
        ++result.deadline_miss_count;
    }
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(kSamplePeriodMs));
  }
  result.duration_us =
      static_cast<uint64_t>(esp_timer_get_time() - started_us);

  const esp_err_t finish_status = stream.finish();
  rememberFirst(finish_status, first);
  result.dropped_frames = stream.droppedFrames();
  result.output_errors = stream.outputErrors();
  finalize(result.logic, logic_accumulator);
  finalize(result.motor, motor_accumulator);
  if (result.deadline_miss_count != 0)
    rememberFirst(ESP_ERR_TIMEOUT, first);
  stream_running.store(false);
  return first;
}

} // 名前空間 bringup::power
