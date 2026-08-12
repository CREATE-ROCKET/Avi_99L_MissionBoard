#include "bringup/motor_bringup.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include "bringup/power_bringup.hpp"
#include "bringup/safe_outputs.hpp"
#include "config/board_config.hpp"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace bringup {
namespace {

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kIn1Channel = LEDC_CHANNEL_0;
constexpr ledc_channel_t kIn2Channel = LEDC_CHANNEL_1;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kMaximumDutyCount = (1U << 10U) - 1U;
constexpr float kEncoderRadiansPerCount =
    6.2831853071795864769F / 16384.0F;
constexpr uint32_t kSamplePeriodUs =
    1'000'000U / board::kSensorRateHz;
static_assert(configTICK_RATE_HZ == board::kSensorRateHz);
constexpr uint32_t kOutputLockTimeoutMs = 5;
// TODO(HW_TEST): motor identification時のFIFO待機timeoutを実測で確定する
constexpr uint32_t kFifoWaitTimeoutMs = 2;

enum class DriveMode : uint8_t {
  coast = 0,
  in1 = 1,
  in2 = 2,
  brake = 3,
};

struct DriveCommand {
  float duty{0.0F};
  DriveMode mode{DriveMode::coast};
};

class BusyGuard {
public:
  explicit BusyGuard(std::atomic<bool> &busy) : busy_(busy) {
    bool expected = false;
    acquired_ = busy_.compare_exchange_strong(expected, true);
  }
  ~BusyGuard() {
    if (acquired_)
      busy_.store(false);
  }
  [[nodiscard]] bool acquired() const { return acquired_; }

private:
  std::atomic<bool> &busy_;
  bool acquired_{false};
};

class OutputLockGuard {
public:
  explicit OutputLockGuard(SemaphoreHandle_t lock) : lock_(lock) {
    if (lock_ == nullptr) {
      result_ = ESP_ERR_INVALID_STATE;
    } else if (xSemaphoreTake(lock_, pdMS_TO_TICKS(kOutputLockTimeoutMs)) ==
               pdTRUE) {
      result_ = ESP_OK;
    } else {
      result_ = ESP_ERR_TIMEOUT;
    }
  }
  ~OutputLockGuard() {
    if (result_ == ESP_OK)
      (void)xSemaphoreGive(lock_);
  }
  [[nodiscard]] bool acquired() const { return result_ == ESP_OK; }
  [[nodiscard]] esp_err_t result() const { return result_; }

private:
  SemaphoreHandle_t lock_{nullptr};
  esp_err_t result_{ESP_ERR_INVALID_STATE};
};

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

bool encoderFault(const AS5047D::Status &status) {
  return status.magnetic_too_low || status.magnetic_too_high ||
         status.cordic_overflow;
}

bool encoderError(const AS5047D::ErrorFlags &flags) {
  return flags.parity_error || flags.invalid_command || flags.framing_error;
}

uint16_t encoderStatusFlags(const AS5047D::Status &status, bool sample_valid,
                            bool status_valid) {
  uint16_t flags = status.magnetic_too_low ? 0x0001U : 0U;
  flags |= status.magnetic_too_high ? 0x0002U : 0U;
  flags |= status.cordic_overflow ? 0x0004U : 0U;
  flags |= status.offset_compensation_finished ? 0x0008U : 0U;
  flags |= status_valid ? 0x4000U : 0U;
  flags |= sample_valid ? 0x8000U : 0U;
  return flags;
}

uint8_t imuFlags(const ImuSample &sample) {
  uint8_t flags = sample.acceleration_valid ? 0x01U : 0U;
  flags |= sample.angular_velocity_valid ? 0x02U : 0U;
  flags |= sample.temperature_valid ? 0x04U : 0U;
  flags |= sample.accel_odr_changed ? 0x08U : 0U;
  flags |= sample.gyro_odr_changed ? 0x10U : 0U;
  return flags;
}

uint8_t fifoFlags(const ICM42688::FifoStatus &status) {
  uint8_t flags = status.threshold ? 0x01U : 0U;
  flags |= status.full ? 0x02U : 0U;
  flags |= status.faulted ? 0x04U : 0U;
  return flags;
}

constexpr uint16_t nextLfsr(uint16_t value) {
  const uint16_t bit = static_cast<uint16_t>(
      ((value >> 0U) ^ (value >> 2U) ^ (value >> 3U) ^ (value >> 5U)) & 1U);
  return static_cast<uint16_t>((value >> 1U) | (bit << 15U));
}

static_assert(nextLfsr(0xACE1U) == 0x5670U);

float prbsValue(uint32_t sample, float amplitude, uint16_t &state) {
  if (sample % 10U == 0U)
    state = nextLfsr(state);
  return (state & 1U) != 0U ? amplitude : -amplitude;
}

uint32_t durationMs(MotorBringup::TestKind kind) {
  switch (kind) {
  case MotorBringup::TestKind::polarity:
    return 6'000;
  case MotorBringup::TestKind::step:
    return 6'000;
  case MotorBringup::TestKind::prbs:
    return 22'000;
  case MotorBringup::TestKind::coast:
    return 5'500;
  case MotorBringup::TestKind::brake:
    return 5'000;
  case MotorBringup::TestKind::combined:
    return 41'000;
  }
  return 0;
}

DriveCommand profileCommand(MotorBringup::TestKind kind, uint32_t sample,
                            bool prbs_ten_percent, uint16_t &lfsr) {
  const uint32_t time_ms = sample;
  if (kind == MotorBringup::TestKind::polarity) {
    if (time_ms < 500U || time_ms >= 5'500U)
      return {};
    const uint32_t slot = (time_ms - 500U) / 500U;
    if ((time_ms - 500U) % 500U >= 250U)
      return {};
    constexpr std::array<float, 5> levels{0.02F, 0.04F, 0.06F, 0.08F,
                                          0.10F};
    const float magnitude = levels[slot % levels.size()];
    return slot < levels.size()
               ? DriveCommand{magnitude, DriveMode::in1}
               : DriveCommand{-magnitude, DriveMode::in2};
  }
  if (kind == MotorBringup::TestKind::step) {
    if (time_ms < 500U)
      return {};
    const uint32_t slot = (time_ms - 500U) / 800U;
    if (slot >= 6U || (time_ms - 500U) % 800U >= 300U)
      return {};
    constexpr std::array<float, 6> levels{0.05F, -0.05F, 0.10F,
                                          -0.10F, 0.15F, -0.15F};
    const float duty = levels[slot];
    return {duty, duty > 0.0F ? DriveMode::in1 : DriveMode::in2};
  }
  if (kind == MotorBringup::TestKind::prbs) {
    if (time_ms >= 500U && time_ms < 10'500U) {
      const float duty = prbsValue(sample, 0.05F, lfsr);
      return {duty, duty > 0.0F ? DriveMode::in1 : DriveMode::in2};
    }
    if (prbs_ten_percent && time_ms >= 11'500U && time_ms < 21'500U) {
      const float duty = prbsValue(sample, 0.10F, lfsr);
      return {duty, duty > 0.0F ? DriveMode::in1 : DriveMode::in2};
    }
    return {};
  }
  if (kind == MotorBringup::TestKind::coast) {
    if (time_ms >= 500U && time_ms < 1'500U)
      return {0.08F, DriveMode::in1};
    return {};
  }
  if (kind == MotorBringup::TestKind::brake) {
    if (time_ms >= 500U && time_ms < 1'500U)
      return {0.05F, DriveMode::in1};
    if (time_ms >= 1'500U && time_ms < 1'750U)
      return {0.0F, DriveMode::brake};
    return {};
  }
  if (time_ms >= 30'000U && time_ms < 40'000U) {
    const float duty = prbsValue(sample, 0.05F, lfsr);
    return {duty, duty > 0.0F ? DriveMode::in1 : DriveMode::in2};
  }
  return {};
}

int32_t wrappedDelta(uint16_t current, uint16_t previous) {
  int32_t delta = static_cast<int32_t>(current) - previous;
  if (delta > 8192)
    delta -= 16384;
  else if (delta < -8192)
    delta += 16384;
  return delta;
}

esp_err_t appendAndSendMotor(StreamProtocol &stream, uint64_t host_us,
                             uint32_t index, const DriveCommand &command,
                             const power::PowerSample &power_sample,
                             const EncoderSample &encoder_sample,
                             float unwrapped_rad, float speed_rad_s,
                             const AS5047D::Status &encoder_status,
                             bool encoder_status_valid,
                             const ImuSample &imu_sample,
                             const ICM42688::FifoStatus &fifo_status,
                             esp_err_t adc_result, esp_err_t encoder_result,
                             esp_err_t imu_result) {
  StreamPayload payload{};
  bool encoded = payload.u64(host_us) && payload.u32(index) &&
                 payload.f32(command.duty) &&
                 payload.u8(static_cast<uint8_t>(command.mode)) &&
                 payload.i32(power_sample.motor.raw) &&
                 payload.f32(power_sample.motor.source_voltage_v) &&
                 payload.u16(encoder_sample.angle_raw) &&
                 payload.f32(unwrapped_rad) && payload.f32(speed_rad_s) &&
                 payload.u16(encoderStatusFlags(encoder_status,
                                                encoder_sample.valid,
                                                encoder_status_valid)) &&
                 payload.u8(encoder_status.agc) &&
                 payload.u16(encoder_status.magnitude);
  for (const int16_t value : imu_sample.acceleration_raw)
    encoded = encoded && payload.i16(value);
  for (const int16_t value : imu_sample.angular_velocity_raw)
    encoded = encoded && payload.i16(value);
  encoded = encoded && payload.u16(imu_sample.timestamp_ticks) &&
            payload.u64(imu_sample.sensor_timestamp_us) &&
            payload.u8(imuFlags(imu_sample)) &&
            payload.u16(fifo_status.lost_packets) &&
            payload.u8(fifoFlags(fifo_status)) && payload.i32(adc_result) &&
            payload.i32(encoder_result) && payload.i32(imu_result);
  if (!encoded)
    return ESP_ERR_INVALID_SIZE;
  return stream.send(StreamRecordType::motor, payload);
}

} // 無名名前空間

esp_err_t MotorBringup::initialize() {
  if (initialized_.load())
    return ESP_OK;
  if (!safe_outputs::initialized())
    return ESP_ERR_INVALID_STATE;
  if (output_lock_ == nullptr) {
    output_lock_ = xSemaphoreCreateMutexStatic(&output_lock_storage_);
    if (output_lock_ == nullptr)
      return ESP_ERR_NO_MEM;
  }
  OutputLockGuard guard{output_lock_};
  if (!guard.acquired())
    return guard.result();
  if (initialized_.load())
    return ESP_OK;

  ledc_timer_config_t timer{};
  timer.speed_mode = kLedcMode;
  timer.duty_resolution = kDutyResolution;
  timer.timer_num = kLedcTimer;
  timer.freq_hz = board::kMotorPwmFrequencyHz;
  timer.clk_cfg = LEDC_USE_APB_CLK;
  esp_err_t result = ledc_timer_config(&timer);
  if (result != ESP_OK) {
    rememberFirst(coastUnlocked(true), result);
    return result;
  }
  timer_configured_.store(true);

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
    rememberFirst(coastUnlocked(true), result);
    return result;
  }
  in1_configured_.store(true);

  ledc_channel_config_t in2 = in1;
  in2.gpio_num = board::kMotorIn2;
  in2.channel = kIn2Channel;
  result = ledc_channel_config(&in2);
  if (result != ESP_OK) {
    rememberFirst(coastUnlocked(true), result);
    return result;
  }
  in2_configured_.store(true);
  initialized_.store(true);
  armed_.store(false);
  result = coastUnlocked();
  if (result != ESP_OK)
    initialized_.store(false);
  return result;
}

esp_err_t MotorBringup::end() {
  armed_.store(false);
  if (busy_.load()) {
    (void)emergencyCoast();
    return ESP_ERR_INVALID_STATE;
  }
  OutputLockGuard guard{output_lock_};
  if (!guard.acquired()) {
    const esp_err_t fallback = emergencyCoast();
    return fallback == ESP_OK ? guard.result() : fallback;
  }
  const esp_err_t result = coastUnlocked(true);
  initialized_.store(false);
  timer_configured_.store(false);
  in1_configured_.store(false);
  in2_configured_.store(false);
  return result;
}

esp_err_t MotorBringup::arm() {
  if (!initialized_.load() || busy_.load() || !safe_outputs::initialized())
    return ESP_ERR_INVALID_STATE;
  OutputLockGuard guard{output_lock_};
  if (!guard.acquired())
    return guard.result();
  const esp_err_t result = coastUnlocked();
  if (result == ESP_OK)
    armed_.store(true);
  return result;
}

esp_err_t MotorBringup::disarm() {
  armed_.store(false);
  return emergencyCoast();
}

esp_err_t MotorBringup::requestDisarm() {
  // workerがsensor処理中でも、arm解除と出力LOWをこのtaskから直ちに要求する。
  armed_.store(false);
  return emergencyCoast();
}

uint32_t MotorBringup::actualPwmFrequencyHz() const {
  return initialized_.load() ? ledc_get_freq(kLedcMode, kLedcTimer) : 0U;
}

esp_err_t MotorBringup::setDuty(float signed_duty) {
  if (!initialized_.load())
    return ESP_ERR_INVALID_STATE;
  if (!std::isfinite(signed_duty) ||
      std::fabs(signed_duty) > board::kMotorBringupMaxDuty) {
    armed_.store(false);
    const esp_err_t rollback = emergencyCoast();
    return rollback == ESP_OK ? ESP_ERR_INVALID_ARG : rollback;
  }
  OutputLockGuard guard{output_lock_};
  if (!guard.acquired()) {
    armed_.store(false);
    const esp_err_t rollback = emergencyCoast();
    return rollback == ESP_OK ? guard.result() : rollback;
  }
  if (!armed_.load()) {
    esp_err_t result = ESP_ERR_INVALID_STATE;
    rememberFirst(coastUnlocked(), result);
    return result;
  }
  const uint32_t duty = static_cast<uint32_t>(
      std::lround(std::fabs(signed_duty) * kMaximumDutyCount));
  const ledc_channel_t active = signed_duty >= 0.0F ? kIn1Channel : kIn2Channel;
  const ledc_channel_t inactive =
      signed_duty >= 0.0F ? kIn2Channel : kIn1Channel;
  esp_err_t result = ledc_set_duty(kLedcMode, inactive, 0);
  if (result == ESP_OK && armed_.load())
    result = ledc_update_duty(kLedcMode, inactive);
  if (result == ESP_OK && !armed_.load())
    result = ESP_ERR_INVALID_STATE;
  if (result == ESP_OK)
    result = ledc_set_duty(kLedcMode, active, duty);
  if (result == ESP_OK && !armed_.load())
    result = ESP_ERR_INVALID_STATE;
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, active);
  if (result == ESP_OK && !armed_.load())
    result = ESP_ERR_INVALID_STATE;
  if (result != ESP_OK)
    rememberFirst(coastUnlocked(), result);
  return result;
}

esp_err_t MotorBringup::coast() {
  OutputLockGuard guard{output_lock_};
  if (!guard.acquired()) {
    armed_.store(false);
    const esp_err_t rollback = emergencyCoast();
    return rollback == ESP_OK ? guard.result() : rollback;
  }
  return coastUnlocked();
}

esp_err_t MotorBringup::coastUnlocked(bool force_channels) {
  esp_err_t result = ESP_OK;
  if (timer_configured_.load() &&
      (force_channels || in1_configured_.load()))
    rememberFirst(ledc_stop(kLedcMode, kIn1Channel, 0), result);
  if (timer_configured_.load() &&
      (force_channels || in2_configured_.load()))
    rememberFirst(ledc_stop(kLedcMode, kIn2Channel, 0), result);
  rememberFirst(safe_outputs::motorCoast(), result);
  return result;
}

esp_err_t MotorBringup::emergencyCoast() {
  OutputLockGuard guard{output_lock_};
  if (guard.acquired())
    return coastUnlocked(true);

  // lock holderもarm解除を各LEDC操作間で確認する。timeout時は安全を優先し、
  // driver stopとGPIO LOWを直接再試行する。
  esp_err_t result = ESP_OK;
  if (timer_configured_.load()) {
    rememberFirst(ledc_stop(kLedcMode, kIn1Channel, 0), result);
    rememberFirst(ledc_stop(kLedcMode, kIn2Channel, 0), result);
  }
  rememberFirst(safe_outputs::motorCoast(), result);
  return result;
}

esp_err_t MotorBringup::brake() {
  if (!initialized_.load())
    return ESP_ERR_INVALID_STATE;
  OutputLockGuard guard{output_lock_};
  if (!guard.acquired()) {
    armed_.store(false);
    const esp_err_t rollback = emergencyCoast();
    return rollback == ESP_OK ? guard.result() : rollback;
  }
  if (!armed_.load()) {
    esp_err_t result = ESP_ERR_INVALID_STATE;
    rememberFirst(coastUnlocked(), result);
    return result;
  }
  esp_err_t result = ledc_set_duty(kLedcMode, kIn1Channel,
                                   kMaximumDutyCount);
  if (result == ESP_OK && armed_.load())
    result = ledc_update_duty(kLedcMode, kIn1Channel);
  if (result == ESP_OK && !armed_.load())
    result = ESP_ERR_INVALID_STATE;
  if (result == ESP_OK)
    result = ledc_set_duty(kLedcMode, kIn2Channel,
                           kMaximumDutyCount);
  if (result == ESP_OK && !armed_.load())
    result = ESP_ERR_INVALID_STATE;
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, kIn2Channel);
  if (result == ESP_OK && !armed_.load())
    result = ESP_ERR_INVALID_STATE;
  if (result != ESP_OK)
    rememberFirst(coastUnlocked(), result);
  return result;
}

esp_err_t MotorBringup::run(TestKind kind, MotorTestResult &result) {
  result = {};
  BusyGuard guard{busy_};
  if (!guard.acquired() || !initialized_.load() || !armed_.load())
    return ESP_ERR_INVALID_STATE;
  if (durationMs(kind) > board::kMotorBringupMaxDurationMs)
    return ESP_ERR_INVALID_ARG;

  esp_err_t first = power::initialize();
  if (first != ESP_OK)
    return first;
  first = spi_.begin();
  if (first != ESP_OK) {
    if (spi_.encoderBusInitialized() || spi_.imuBusInitialized())
      rememberFirst(spi_.end(), first);
    return first;
  }

  bool encoder_started = false;
  bool imu_started = false;
  bool pipeline_started = false;
  esp_err_t operation = encoder_.begin(spi_);
  if (operation == ESP_OK)
    encoder_started = true;
  rememberFirst(operation, first);
  if (first == ESP_OK) {
    operation = encoder_.getStatus(result.initial_encoder_status);
    rememberFirst(operation, first);
    if (operation == ESP_OK && encoderFault(result.initial_encoder_status))
      rememberFirst(ESP_ERR_INVALID_STATE, first);
  }
  if (first == ESP_OK) {
    operation = imu_.begin(spi_, true);
    if (operation == ESP_OK)
      imu_started = true;
    rememberFirst(operation, first);
  }
  if (first == ESP_OK) {
    operation = encoder_.startPipelinedRead();
    if (operation == ESP_OK)
      pipeline_started = true;
    rememberFirst(operation, first);
  }
  if (first == ESP_OK) {
    operation = stream_.initialize();
    rememberFirst(operation, first);
  }
  if (first == ESP_OK) {
    operation = stream_.beginCapture();
    rememberFirst(operation, first);
  }

  const uint32_t maximum_samples = durationMs(kind) * board::kSensorRateHz /
                                   1'000U;
  result.requested_samples = maximum_samples;
  uint16_t lfsr = 0xACE1U;
  bool have_previous = false;
  uint16_t previous_raw = 0;
  uint64_t previous_us = 0;
  float unwrapped_rad = 0.0F;
  float initial_unwrapped = 0.0F;
  DriveCommand previous_command{};
  bool have_previous_command = false;
  TickType_t wake = xTaskGetTickCount();
  const int64_t test_started_us = esp_timer_get_time();
  const int64_t hard_deadline_us =
      test_started_us + static_cast<int64_t>(durationMs(kind)) * 1'000 +
      kSamplePeriodUs;

  for (uint32_t index = 0; first == ESP_OK && index < maximum_samples;
       ++index) {
    if (!armed_.load()) {
      rememberFirst(coast(), first);
      if (first == ESP_OK)
        first = ESP_ERR_INVALID_STATE;
      break;
    }
    vTaskDelayUntil(&wake, 1);
    const int64_t scheduled_us =
        test_started_us + static_cast<int64_t>(index + 1U) * kSamplePeriodUs;
    const int64_t after_wake_us = esp_timer_get_time();
    if (after_wake_us > hard_deadline_us ||
        after_wake_us > scheduled_us + kSamplePeriodUs) {
      ++result.deadline_miss_count;
      rememberFirst(ESP_ERR_TIMEOUT, first);
      (void)coast();
      break;
    }
    if (!armed_.load()) {
      // disarmを検出したらsensor処理より先に両入力をLOWへ戻す。
      rememberFirst(coast(), first);
      if (first == ESP_OK)
        first = ESP_ERR_INVALID_STATE;
      break;
    }
    if (kind == TestKind::prbs && index == 11'500U) {
      result.prbs_ten_percent_executed =
          result.encoder_error_count == 0 && result.imu_error_count == 0 &&
          result.adc_error_count == 0 && result.stream_error_count == 0 &&
          result.maximum_abs_speed_rad_s <
              board::kMotorBringupMaxSpeedRadS * 0.5F;
      if (!result.prbs_ten_percent_executed) {
        result.requested_samples = index;
        break;
      }
    }

    const DriveCommand command =
        profileCommand(kind, index, result.prbs_ten_percent_executed, lfsr);
    const int64_t loop_started = esp_timer_get_time();
    operation = ESP_OK;
    if (!have_previous_command || command.mode != previous_command.mode ||
        command.duty != previous_command.duty) {
      if (command.mode == DriveMode::brake)
        operation = brake();
      else if (command.mode == DriveMode::coast)
        operation = coast();
      else
        operation = setDuty(command.duty);
      previous_command = command;
      have_previous_command = true;
    }
    rememberFirst(operation, first);
    if (operation != ESP_OK) {
      // 出力更新失敗時はsensor/log処理へ進まず、rollbackを重ねて終了する。
      (void)coast();
      break;
    }

    EncoderSample encoder_sample{};
    esp_err_t encoder_result = encoder_.readPipelined(encoder_sample);
    if (encoder_result != ESP_OK)
      ++result.encoder_error_count;

    ImuSample imu_sample{};
    std::size_t imu_count = 0;
    esp_err_t imu_result = imu_.waitFifo(kFifoWaitTimeoutMs);
    if (imu_result == ESP_OK)
      imu_result = imu_.readFifo(&imu_sample, 1, imu_count);
    if (imu_result == ESP_OK && imu_count != 1)
      imu_result = ESP_ERR_INVALID_RESPONSE;
    ICM42688::FifoStatus fifo_status{};
    const esp_err_t fifo_result = imu_.getFifoStatus(fifo_status);
    if (imu_result == ESP_OK)
      imu_result = fifo_result;
    if (imu_result != ESP_OK || !imu_sample.acceleration_valid ||
        !imu_sample.angular_velocity_valid || fifo_status.full ||
        fifo_status.faulted || fifo_status.lost_packets != 0) {
      ++result.imu_error_count;
      if (imu_result == ESP_OK)
        imu_result = ESP_ERR_INVALID_RESPONSE;
    }
    result.maximum_lost_packets =
        std::max(result.maximum_lost_packets, fifo_status.lost_packets);

    power::PowerSample power_sample{};
    const esp_err_t adc_result = power::read(power_sample);
    if (adc_result != ESP_OK)
      ++result.adc_error_count;

    float speed_rad_s = std::numeric_limits<float>::quiet_NaN();
    if (encoder_result == ESP_OK && encoder_sample.valid) {
      if (!have_previous) {
        unwrapped_rad = encoder_sample.angle_radians;
        initial_unwrapped = unwrapped_rad;
        previous_raw = encoder_sample.angle_raw;
        previous_us = encoder_sample.host_timestamp_us;
        have_previous = true;
      } else {
        const int32_t delta =
            wrappedDelta(encoder_sample.angle_raw, previous_raw);
        unwrapped_rad += static_cast<float>(delta) * kEncoderRadiansPerCount;
        const uint64_t elapsed_us =
            encoder_sample.host_timestamp_us - previous_us;
        if (elapsed_us != 0)
          speed_rad_s = static_cast<float>(delta) * kEncoderRadiansPerCount *
                        1'000'000.0F / static_cast<float>(elapsed_us);
        previous_raw = encoder_sample.angle_raw;
        previous_us = encoder_sample.host_timestamp_us;
      }
    }
    if (std::isfinite(speed_rad_s))
      result.maximum_abs_speed_rad_s =
          std::max(result.maximum_abs_speed_rad_s, std::fabs(speed_rad_s));

    // AS5047D status取得はpipelineを停止するため、開始時snapshotは最初の
    // recordだけをvalidにする。bit14がstatus snapshot validを表す。
    const bool encoder_status_valid = index == 0;
    const AS5047D::Status encoder_status =
        encoder_status_valid ? result.initial_encoder_status
                             : AS5047D::Status{};
    const esp_err_t send_result = appendAndSendMotor(
        stream_, static_cast<uint64_t>(loop_started), index, command,
        power_sample, encoder_sample, unwrapped_rad, speed_rad_s,
        encoder_status, encoder_status_valid, imu_sample, fifo_status,
        adc_result, encoder_result, imu_result);
    if (send_result != ESP_OK)
      ++result.stream_error_count;
    ++result.sample_count;

    const int64_t elapsed = esp_timer_get_time() - loop_started;
    if (elapsed > 0) {
      result.max_loop_latency_us =
          std::max(result.max_loop_latency_us,
                   static_cast<uint32_t>(elapsed));
      if (elapsed > kSamplePeriodUs)
        ++result.deadline_miss_count;
    }
    if (result.deadline_miss_count != 0) {
      rememberFirst(ESP_ERR_TIMEOUT, first);
      (void)coast();
      break;
    }
    if (encoder_result != ESP_OK || imu_result != ESP_OK ||
        adc_result != ESP_OK || send_result != ESP_OK ||
        result.maximum_abs_speed_rad_s > board::kMotorBringupMaxSpeedRadS) {
      rememberFirst(encoder_result, first);
      rememberFirst(imu_result, first);
      rememberFirst(adc_result, first);
      rememberFirst(send_result, first);
      if (first == ESP_OK)
        first = ESP_ERR_INVALID_STATE;
      (void)coast();
      break;
    }
  }

  rememberFirst(coast(), first);
  if (stream_.active())
    rememberFirst(stream_.finish(), first);
  result.dropped_frames = stream_.droppedFrames();
  result.output_errors = stream_.outputErrors();
  result.encoder_delta_rad = have_previous ? unwrapped_rad - initial_unwrapped
                                           : 0.0F;

  if (pipeline_started) {
    rememberFirst(encoder_.stopPipelinedRead(), first);
    pipeline_started = false;
  }
  if (encoder_started) {
    rememberFirst(encoder_.getStatus(result.final_encoder_status), first);
    rememberFirst(
        encoder_.readAndClearErrorFlags(result.final_encoder_errors), first);
    if (encoderFault(result.final_encoder_status) ||
        encoderError(result.final_encoder_errors))
      rememberFirst(ESP_ERR_INVALID_RESPONSE, first);
    rememberFirst(encoder_.end(), first);
  }
  if (imu_started)
    rememberFirst(imu_.end(), first);
  rememberFirst(spi_.end(), first);
  (void)coast();

  if (first != ESP_OK) {
    armed_.store(false);
    const esp_err_t disarm_result = emergencyCoast();
    if (disarm_result != ESP_OK)
      std::printf("motor failure cleanup disarm: %s\n",
                  esp_err_to_name(disarm_result));
  }

  std::printf(
      "motor summary: samples=%lu/%lu encoder_error=%lu imu_error=%lu "
      "adc_error=%lu stream_error=%lu dropped=%lu max_loop_us=%lu "
      "deadline_miss=%lu output_error=%lu lost=%u "
      "max_speed_rad_s=%.3f encoder_delta_rad=%.6f prbs_10pct=%s "
      "encoder_status=%d/%d/%d encoder_flags=%d/%d/%d result=%s\n",
      static_cast<unsigned long>(result.sample_count),
      static_cast<unsigned long>(result.requested_samples),
      static_cast<unsigned long>(result.encoder_error_count),
      static_cast<unsigned long>(result.imu_error_count),
      static_cast<unsigned long>(result.adc_error_count),
      static_cast<unsigned long>(result.stream_error_count),
      static_cast<unsigned long>(result.dropped_frames),
      static_cast<unsigned long>(result.max_loop_latency_us),
      static_cast<unsigned long>(result.deadline_miss_count),
      static_cast<unsigned long>(result.output_errors),
      result.maximum_lost_packets,
      result.maximum_abs_speed_rad_s, result.encoder_delta_rad,
      result.prbs_ten_percent_executed ? "yes" : "no",
      result.final_encoder_status.magnetic_too_low,
      result.final_encoder_status.magnetic_too_high,
      result.final_encoder_status.cordic_overflow,
      result.final_encoder_errors.parity_error,
      result.final_encoder_errors.invalid_command,
      result.final_encoder_errors.framing_error,
      esp_err_to_name(first));
  return first;
}

esp_err_t MotorBringup::polarity(MotorTestResult &result) {
  return run(TestKind::polarity, result);
}

esp_err_t MotorBringup::step(MotorTestResult &result) {
  return run(TestKind::step, result);
}

esp_err_t MotorBringup::prbs(MotorTestResult &result) {
  return run(TestKind::prbs, result);
}

esp_err_t MotorBringup::coastTest(MotorTestResult &result) {
  return run(TestKind::coast, result);
}

esp_err_t MotorBringup::brakeTest(MotorTestResult &result) {
  return run(TestKind::brake, result);
}

esp_err_t MotorBringup::combinedMotorImuTest(MotorTestResult &result) {
  return run(TestKind::combined, result);
}

} // 名前空間 bringup
