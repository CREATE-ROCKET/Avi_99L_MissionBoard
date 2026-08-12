#include "bringup/imu_bringup.hpp"

#include <algorithm>
#include <climits>
#include <limits>

#include "avi_esp_libs/timeout.h"
#include "config/board_config.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace bringup {
namespace {

constexpr float kAccelCountsPerG = 2048.0F;
constexpr float kGyroCountsPerDps = 16.4F;
constexpr uint8_t kExpectedWhoAmI = 0x47;
// TODO(HW_TEST): 1 kHz FIFO待機timeoutを実機latency測定で確定する
constexpr uint32_t kFifoWaitTimeoutMs = 5;
// TODO(HW_TEST): 指定sample数取得時の終了猶予を実機で確定する
constexpr uint64_t kCaptureCompletionMarginUs = 100'000;

class ExclusiveGuard {
public:
  explicit ExclusiveGuard(std::atomic<bool> &busy) : busy_(busy) {
    bool expected = false;
    acquired_ = busy_.compare_exchange_strong(expected, true);
  }
  ~ExclusiveGuard() {
    if (acquired_)
      busy_.store(false);
  }
  [[nodiscard]] bool acquired() const { return acquired_; }

private:
  std::atomic<bool> &busy_;
  bool acquired_{false};
};

uint64_t nowUs() {
  const int64_t value = esp_timer_get_time();
  return value < 0 ? 0 : static_cast<uint64_t>(value);
}

uint32_t elapsedUs(uint64_t started_at) {
  const uint64_t now = nowUs();
  const uint64_t elapsed = now >= started_at ? now - started_at : 0;
  return static_cast<uint32_t>(
      std::min<uint64_t>(elapsed, std::numeric_limits<uint32_t>::max()));
}

void rememberFirst(esp_err_t operation, esp_err_t &first_error) {
  if (first_error == ESP_OK && operation != ESP_OK)
    first_error = operation;
}

bool rawVectorValid(const std::array<int16_t, 3> &values) {
  return values[0] != INT16_MIN && values[1] != INT16_MIN &&
         values[2] != INT16_MIN;
}

uint64_t accumulateTimestamp(uint16_t ticks, uint64_t &microseconds,
                             uint8_t &remainder) {
  // ICM42688 v1.6 12.7の32/30補正をdriverと同じ順序で適用する。
  const uint32_t numerator = static_cast<uint32_t>(ticks) * 32U + remainder;
  microseconds += numerator / 30U;
  remainder = static_cast<uint8_t>(numerator % 30U);
  return microseconds;
}

uint8_t sampleFlags(const ImuSample &sample,
                    const ICM42688::FifoStatus &status) {
  uint8_t flags = sample.acceleration_valid ? 0x01U : 0U;
  flags |= sample.angular_velocity_valid ? 0x02U : 0U;
  flags |= sample.temperature_valid ? 0x04U : 0U;
  flags |= sample.accel_odr_changed ? 0x08U : 0U;
  flags |= sample.gyro_odr_changed ? 0x10U : 0U;
  flags |= status.threshold ? 0x20U : 0U;
  flags |= status.full ? 0x40U : 0U;
  flags |= status.faulted ? 0x80U : 0U;
  return flags;
}

bool appendSample(StreamPayload &payload, const ImuSample &sample,
                  const ICM42688::FifoStatus &status) {
  if (!payload.u64(sample.host_timestamp_us) ||
      !payload.u64(sample.sensor_timestamp_us) ||
      !payload.u16(sample.timestamp_ticks))
    return false;
  for (const int16_t value : sample.acceleration_raw) {
    if (!payload.i16(value))
      return false;
  }
  for (const int16_t value : sample.angular_velocity_raw) {
    if (!payload.i16(value))
      return false;
  }
  if (!payload.i16(sample.temperature_raw))
    return false;
  for (const float value : sample.acceleration_g) {
    if (!payload.f32(value))
      return false;
  }
  for (const float value : sample.angular_velocity_dps) {
    if (!payload.f32(value))
      return false;
  }
  return payload.f32(sample.temperature_celsius) &&
         payload.u32(sample.read_latency_us) &&
         payload.u16(status.records_available) &&
         payload.u16(status.lost_packets) &&
         payload.u8(sampleFlags(sample, status));
}

void includeFifoStatus(const ICM42688::FifoStatus &status,
                       ImuStreamResult &result) {
  if (status.full)
    ++result.fifo_full_count;
  if (status.faulted)
    ++result.fifo_fault_count;
  result.maximum_lost_packets =
      std::max(result.maximum_lost_packets, status.lost_packets);
}

} // 無名名前空間

bool ImuSelfTestResult::passed() const {
  return begin_result == ESP_OK && who_am_i_result == ESP_OK &&
         who_am_i == kExpectedWhoAmI && self_test_result == ESP_OK &&
         detail.passed && detail.restored && end_result == ESP_OK;
}

bool ImuStreamResult::passed() const {
  return requested_samples != 0 && sample_count == requested_samples &&
         driver_error_count == 0 && stream_error_count == 0 &&
         dropped_frames == 0 && output_errors == 0 &&
         invalid_acceleration_count == 0 &&
         invalid_angular_velocity_count == 0 &&
         invalid_temperature_count == 0 && accel_odr_change_count == 0 &&
         gyro_odr_change_count == 0 && timestamp_nonmonotonic_count == 0 &&
         fifo_full_count == 0 && fifo_fault_count == 0 &&
         maximum_lost_packets == 0 && begin_result == ESP_OK &&
         stream_begin_result == ESP_OK && final_status_result == ESP_OK &&
         stream_finish_result == ESP_OK && end_result == ESP_OK &&
         !final_fifo_status.full && !final_fifo_status.faulted &&
         final_fifo_status.lost_packets == 0;
}

esp_err_t ImuBringup::beginImpl(SpiBringup &spi, bool fifo_enabled) {
  SPICREATE *const bus = spi.imuBus();
  if (bus == nullptr)
    return ESP_ERR_INVALID_STATE;
  ICM42688::Config config{};
  config.frequency_hz = board::kImuSpiFrequencyHz;
  config.accel_range = ICM42688::AccelRange::g16;
  config.gyro_range = ICM42688::GyroRange::dps2000;
  config.accel_odr = ICM42688::AccelOdr::hz1000;
  config.gyro_odr = ICM42688::GyroOdr::hz1000;
  config.filter = ICM42688::Filter::odr_div4;
  config.int_gpio = board::kImuInterrupt;
  config.fifo.enabled = fifo_enabled;
  config.fifo.watermark_records = 1;
  const esp_err_t result = imu_.begin(*bus, board::kImuCs, config);
  if (result == ESP_OK) {
    fifo_enabled_ = fifo_enabled;
    fifo_timestamp_us_ = 0;
    fifo_timestamp_remainder_ = 0;
  }
  return result;
}

esp_err_t ImuBringup::readImpl(ImuSample &sample) {
  if (fifo_enabled_)
    return ESP_ERR_INVALID_STATE;
  ICM42688::RawData raw{};
  const uint64_t started_at = nowUs();
  const esp_err_t result = imu_.readRaw(raw);
  const uint32_t latency = elapsedUs(started_at);
  if (result != ESP_OK)
    return result;

  ImuSample next{};
  next.host_timestamp_us = nowUs();
  next.acceleration_raw = raw.acceleration;
  next.angular_velocity_raw = raw.angular_velocity;
  next.temperature_raw = raw.temperature;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    next.acceleration_g[axis] = raw.acceleration[axis] / kAccelCountsPerG;
    next.angular_velocity_dps[axis] =
        raw.angular_velocity[axis] / kGyroCountsPerDps;
  }
  next.temperature_celsius = raw.temperature / 132.48F + 25.0F;
  next.read_latency_us = latency;
  next.acceleration_valid = rawVectorValid(raw.acceleration);
  next.angular_velocity_valid = rawVectorValid(raw.angular_velocity);
  next.temperature_valid = raw.temperature != INT16_MIN;
  sample = next;
  return ESP_OK;
}

esp_err_t ImuBringup::readFifoImpl(ImuSample *samples, std::size_t capacity,
                                   std::size_t &count) {
  if (!imu_.initialized() || !fifo_enabled_)
    return ESP_ERR_INVALID_STATE;
  if ((samples == nullptr && capacity != 0) ||
      capacity > kMaximumFifoBatch)
    return ESP_ERR_INVALID_ARG;
  std::array<ICM42688::FifoRawData, kMaximumFifoBatch> raw{};
  std::size_t raw_count = 0;
  const uint64_t started_at = nowUs();
  const esp_err_t result = imu_.readFifoRaw(raw.data(), capacity, raw_count);
  const uint32_t latency = elapsedUs(started_at);
  if (result != ESP_OK)
    return result;

  const uint64_t host_timestamp = nowUs();
  for (std::size_t index = 0; index < raw_count; ++index) {
    ImuSample next{};
    next.host_timestamp_us = host_timestamp;
    next.sensor_timestamp_us =
        accumulateTimestamp(raw[index].timestamp_ticks, fifo_timestamp_us_,
                            fifo_timestamp_remainder_);
    next.timestamp_ticks = raw[index].timestamp_ticks;
    next.acceleration_raw = raw[index].acceleration;
    next.angular_velocity_raw = raw[index].angular_velocity;
    next.temperature_raw = raw[index].temperature;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      next.acceleration_g[axis] =
          raw[index].acceleration[axis] / kAccelCountsPerG;
      next.angular_velocity_dps[axis] =
          raw[index].angular_velocity[axis] / kGyroCountsPerDps;
    }
    next.temperature_celsius = raw[index].temperature / 2.07F + 25.0F;
    next.read_latency_us = latency;
    next.acceleration_valid = raw[index].acceleration_valid;
    next.angular_velocity_valid = raw[index].angular_velocity_valid;
    next.temperature_valid = raw[index].temperature_valid;
    next.accel_odr_changed = raw[index].accel_odr_changed;
    next.gyro_odr_changed = raw[index].gyro_odr_changed;
    samples[index] = next;
  }
  count = raw_count;
  return ESP_OK;
}

esp_err_t ImuBringup::endImpl() {
  const esp_err_t result = imu_.end();
  if (!imu_.initialized()) {
    fifo_enabled_ = false;
    fifo_timestamp_us_ = 0;
    fifo_timestamp_remainder_ = 0;
  }
  return result;
}

esp_err_t ImuBringup::begin(SpiBringup &spi, bool fifo_enabled) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? beginImpl(spi, fifo_enabled)
                          : ESP_ERR_INVALID_STATE;
}

esp_err_t ImuBringup::read(ImuSample &sample) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? readImpl(sample) : ESP_ERR_INVALID_STATE;
}

esp_err_t ImuBringup::waitFifo(uint32_t timeout_ms) {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  return imu_.waitFifo(timeout_ms == 0
                           ? avi::Timeout::noWait()
                           : avi::Timeout::milliseconds(timeout_ms));
}

esp_err_t ImuBringup::getFifoStatus(ICM42688::FifoStatus &status) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? imu_.getFifoStatus(status)
                          : ESP_ERR_INVALID_STATE;
}

esp_err_t ImuBringup::readFifo(ImuSample *samples, std::size_t capacity,
                               std::size_t &count) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? readFifoImpl(samples, capacity, count)
                          : ESP_ERR_INVALID_STATE;
}

esp_err_t ImuBringup::end() {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? endImpl() : ESP_ERR_INVALID_STATE;
}

esp_err_t ImuBringup::selfTest(SpiBringup &spi,
                               ImuSelfTestResult &result) {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired() || imu_.initialized())
    return ESP_ERR_INVALID_STATE;

  result = {};
  result.begin_result = beginImpl(spi, false);
  if (result.begin_result != ESP_OK)
    return result.begin_result;

  result.who_am_i_result = imu_.whoAmI(result.who_am_i);
  result.self_test_result =
      imu_.selfTest(result.detail, avi::Timeout::milliseconds(2'500));
  result.end_result = endImpl();

  esp_err_t first_error = ESP_OK;
  rememberFirst(result.who_am_i_result, first_error);
  rememberFirst(result.self_test_result, first_error);
  rememberFirst(result.end_result, first_error);
  if (first_error == ESP_OK && !result.passed())
    first_error = ESP_FAIL;
  return first_error;
}

esp_err_t ImuBringup::captureImpl(SpiBringup &spi, uint32_t seconds,
                                  StreamProtocol &protocol,
                                  ImuStreamResult &result) {
  if (imu_.initialized() || protocol.active())
    return ESP_ERR_INVALID_STATE;
  if (seconds == 0)
    return ESP_ERR_INVALID_ARG;
  const uint64_t requested =
      static_cast<uint64_t>(seconds) * board::kSensorRateHz;
  if (requested > std::numeric_limits<uint32_t>::max())
    return ESP_ERR_INVALID_ARG;

  result = {};
  result.requested_samples = static_cast<uint32_t>(requested);
  esp_err_t first_error = ESP_OK;
  result.begin_result = beginImpl(spi, true);
  rememberFirst(result.begin_result, first_error);
  if (result.begin_result == ESP_OK) {
    const esp_err_t initialize_result = protocol.initialize();
    rememberFirst(initialize_result, first_error);
    if (initialize_result == ESP_OK) {
      result.stream_begin_result = protocol.beginCapture();
      rememberFirst(result.stream_begin_result, first_error);
    }
  }

  const uint64_t capture_deadline =
      nowUs() + static_cast<uint64_t>(seconds) * 1'000'000U +
      kCaptureCompletionMarginUs;
  uint64_t previous_timestamp = 0;
  bool have_previous_timestamp = false;
  std::array<ImuSample, kMaximumFifoBatch> samples{};
  while (result.stream_begin_result == ESP_OK &&
         result.sample_count < result.requested_samples &&
         nowUs() < capture_deadline) {
    ICM42688::FifoStatus status{};
    esp_err_t operation = imu_.getFifoStatus(status);
    if (operation != ESP_OK) {
      ++result.driver_error_count;
      rememberFirst(operation, first_error);
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    includeFifoStatus(status, result);
    if (status.faulted)
      break;

    if (status.records_available == 0) {
      operation = imu_.waitFifo(
          avi::Timeout::milliseconds(kFifoWaitTimeoutMs));
      if (operation != ESP_OK) {
        ++result.driver_error_count;
        rememberFirst(operation, first_error);
        continue;
      }
      status = {};
      operation = imu_.getFifoStatus(status);
      if (operation != ESP_OK) {
        ++result.driver_error_count;
        rememberFirst(operation, first_error);
        continue;
      }
      includeFifoStatus(status, result);
      if (status.faulted)
        break;
    }

    const std::size_t remaining =
        result.requested_samples - result.sample_count;
    const std::size_t capacity =
        std::min<std::size_t>(kMaximumFifoBatch, remaining);
    std::size_t count = 0;
    operation = readFifoImpl(samples.data(), capacity, count);
    if (operation != ESP_OK) {
      ++result.driver_error_count;
      rememberFirst(operation, first_error);
      continue;
    }
    if (count == 0)
      continue;
    ++result.batch_count;

    for (std::size_t index = 0; index < count; ++index) {
      const ImuSample &sample = samples[index];
      ++result.sample_count;
      result.max_read_latency_us =
          std::max(result.max_read_latency_us, sample.read_latency_us);
      if (!sample.acceleration_valid)
        ++result.invalid_acceleration_count;
      if (!sample.angular_velocity_valid)
        ++result.invalid_angular_velocity_count;
      if (!sample.temperature_valid)
        ++result.invalid_temperature_count;
      if (sample.accel_odr_changed)
        ++result.accel_odr_change_count;
      if (sample.gyro_odr_changed)
        ++result.gyro_odr_change_count;
      if (have_previous_timestamp &&
          sample.sensor_timestamp_us <= previous_timestamp)
        ++result.timestamp_nonmonotonic_count;
      previous_timestamp = sample.sensor_timestamp_us;
      have_previous_timestamp = true;

      StreamPayload payload{};
      if (!appendSample(payload, sample, status)) {
        ++result.stream_error_count;
        rememberFirst(ESP_ERR_INVALID_SIZE, first_error);
        continue;
      }
      const esp_err_t stream_result =
          protocol.send(StreamRecordType::imu, payload);
      if (stream_result != ESP_OK) {
        ++result.stream_error_count;
        rememberFirst(stream_result, first_error);
      }
    }
  }

  if (imu_.initialized()) {
    result.final_status_result =
        imu_.getFifoStatus(result.final_fifo_status);
    rememberFirst(result.final_status_result, first_error);
    if (result.final_status_result == ESP_OK)
      includeFifoStatus(result.final_fifo_status, result);
  }
  if (imu_.initialized()) {
    result.end_result = endImpl();
    rememberFirst(result.end_result, first_error);
  }
  if (protocol.active()) {
    result.stream_finish_result = protocol.finish();
    rememberFirst(result.stream_finish_result, first_error);
  }
  result.dropped_frames = protocol.droppedFrames();
  result.output_errors = protocol.outputErrors();
  if (first_error == ESP_OK && !result.passed())
    first_error = ESP_FAIL;
  return first_error;
}

esp_err_t ImuBringup::stream(SpiBringup &spi, uint32_t seconds,
                             StreamProtocol &protocol,
                             ImuStreamResult &result) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? captureImpl(spi, seconds, protocol, result)
                          : ESP_ERR_INVALID_STATE;
}

esp_err_t ImuBringup::staticCapture(SpiBringup &spi, uint32_t seconds,
                                    StreamProtocol &protocol,
                                    ImuStreamResult &result) {
  ExclusiveGuard guard{busy_};
  // staticでもrawを同じ形式で保存し、統計値はhost側で再計算可能にする。
  return guard.acquired() ? captureImpl(spi, seconds, protocol, result)
                          : ESP_ERR_INVALID_STATE;
}

} // 名前空間 bringup
