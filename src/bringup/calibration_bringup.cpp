#include "bringup/calibration_bringup.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "I2CCREATE.h"
#include "SSCDRRN005PD2A5.h"
#include "avi_esp_libs/timeout.h"
#include "config/board_config.hpp"
#include "esp_timer.h"

namespace bringup {
namespace {

constexpr std::size_t kBatchCapacity = ImuBringup::kMaximumFifoBatch;
// TODO(HW_TEST): calibration中のFIFO待機timeoutを実測で確定する
constexpr uint32_t kFifoWaitTimeoutMs = 5;
constexpr uint32_t kMaximumRepeatCount = 100;

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

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

I2CCREATE::Config i2cConfig() {
  I2CCREATE::Config config{};
  config.port = board::kAirDataI2cPort;
  config.sda = board::kAirDataSda;
  config.scl = board::kAirDataScl;
  config.frequency_hz = board::kAirDataI2cFrequencyHz;
  config.enable_internal_pullups = false;
  config.lock_timeout = avi::Timeout::noWait();
  config.operation_timeout =
      avi::Timeout::milliseconds(board::kAirDataOperationTimeoutMs);
  return config;
}

bool finiteVector(const std::array<float, 3> &value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

uint8_t validFlags(const CalibrationAttempt &attempt) {
  uint8_t flags = attempt.gyro_valid ? 0x01U : 0U;
  flags |= attempt.gravity_valid ? 0x02U : 0U;
  flags |= attempt.launcher_tilt_valid ? 0x04U : 0U;
  flags |= attempt.ssc_valid ? 0x08U : 0U;
  return flags;
}

esp_err_t sendAttempt(StreamProtocol &stream,
                      const CalibrationAttempt &attempt) {
  StreamPayload payload{};
  bool encoded = payload.u64(attempt.completed_at_us) &&
                 payload.u32(attempt.id) && payload.u8(validFlags(attempt)) &&
                 payload.u32(attempt.imu_samples);
  for (const float value : attempt.gyro_bias_dps)
    encoded = encoded && payload.f32(value);
  for (const float value : attempt.gravity_sensor_g)
    encoded = encoded && payload.f32(value);
  encoded = encoded && payload.f32(attempt.acceleration_norm_g) &&
            payload.f32(attempt.launcher_tilt_deg) &&
            payload.f32(attempt.ssc_zero_pa) &&
            payload.u32(attempt.error_count);
  return encoded ? stream.send(StreamRecordType::calibration, payload)
                 : ESP_ERR_INVALID_SIZE;
}

esp_err_t collectAttempt(SpiBringup &spi, ImuBringup &imu,
                         SSCDRRN005PD2A5 *ssc,
                         CalibrationAttempt &attempt) {
  const uint32_t requested =
      board::kCalibrationDurationMs * board::kSensorRateHz / 1'000U;
  std::array<double, 3> gyro_sum{};
  std::array<double, 3> acceleration_sum{};
  double ssc_sum = 0.0;
  esp_err_t first = imu.begin(spi, true);
  attempt.imu_status = first;
  if (first != ESP_OK) {
    ++attempt.error_count;
    return first;
  }

  const int64_t deadline =
      esp_timer_get_time() + board::kCalibrationDurationMs * 1'000LL +
      200'000LL;
  std::array<ImuSample, kBatchCapacity> samples{};
  while (attempt.imu_samples < requested && esp_timer_get_time() < deadline) {
    esp_err_t operation = imu.waitFifo(kFifoWaitTimeoutMs);
    if (operation != ESP_OK) {
      ++attempt.error_count;
      rememberFirst(operation, first);
      continue;
    }

    ICM42688::FifoStatus fifo{};
    operation = imu.getFifoStatus(fifo);
    if (operation != ESP_OK) {
      ++attempt.error_count;
      rememberFirst(operation, first);
      continue;
    }
    attempt.maximum_lost_packets =
        std::max(attempt.maximum_lost_packets, fifo.lost_packets);
    if (fifo.full || fifo.faulted || fifo.lost_packets != 0) {
      ++attempt.error_count;
      rememberFirst(ESP_ERR_INVALID_RESPONSE, first);
      break;
    }

    const std::size_t capacity = std::min<std::size_t>(
        samples.size(), requested - attempt.imu_samples);
    std::size_t count = 0;
    operation = imu.readFifo(samples.data(), capacity, count);
    if (operation != ESP_OK) {
      ++attempt.error_count;
      rememberFirst(operation, first);
      continue;
    }
    for (std::size_t sample_index = 0; sample_index < count;
         ++sample_index) {
      const ImuSample &sample = samples[sample_index];
      if (!sample.acceleration_valid || !sample.angular_velocity_valid ||
          sample.accel_odr_changed || sample.gyro_odr_changed) {
        ++attempt.error_count;
        rememberFirst(ESP_ERR_INVALID_RESPONSE, first);
        continue;
      }
      for (std::size_t axis = 0; axis < 3; ++axis) {
        acceleration_sum[axis] += sample.acceleration_g[axis];
        gyro_sum[axis] += sample.angular_velocity_dps[axis];
      }
      ++attempt.imu_samples;

      if (ssc != nullptr && attempt.imu_samples % 10U == 0U) {
        SSCDRRN005PD2A5::Data data{};
        attempt.ssc_status = ssc->read(data);
        if (attempt.ssc_status == ESP_OK &&
            std::isfinite(data.differential_pressure_pa)) {
          ssc_sum += data.differential_pressure_pa;
          ++attempt.ssc_samples;
        } else {
          ++attempt.error_count;
        }
      }
    }
  }

  if (attempt.imu_samples == requested && first == ESP_OK) {
    const double inverse = 1.0 / static_cast<double>(attempt.imu_samples);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      attempt.gyro_bias_dps[axis] =
          static_cast<float>(gyro_sum[axis] * inverse);
      attempt.gravity_sensor_g[axis] =
          static_cast<float>(acceleration_sum[axis] * inverse);
    }
    attempt.acceleration_norm_g = std::sqrt(
        attempt.gravity_sensor_g[0] * attempt.gravity_sensor_g[0] +
        attempt.gravity_sensor_g[1] * attempt.gravity_sensor_g[1] +
        attempt.gravity_sensor_g[2] * attempt.gravity_sensor_g[2]);
    // X/Y方向は未確定なので、body_z=-sensor_zから得られる傾きの大きさだけを出す。
    const float radial = std::hypot(attempt.gravity_sensor_g[0],
                                    attempt.gravity_sensor_g[1]);
    attempt.launcher_tilt_deg =
        std::atan2(radial, std::fabs(attempt.gravity_sensor_g[2])) *
        57.29577951308232F;
    attempt.gyro_valid = finiteVector(attempt.gyro_bias_dps);
    attempt.gravity_valid = finiteVector(attempt.gravity_sensor_g) &&
                            std::isfinite(attempt.acceleration_norm_g);
    attempt.launcher_tilt_valid =
        attempt.gravity_valid && std::isfinite(attempt.launcher_tilt_deg);
  } else if (first == ESP_OK) {
    first = ESP_ERR_TIMEOUT;
    ++attempt.error_count;
  }
  attempt.imu_status = first;

  if (attempt.ssc_samples != 0) {
    attempt.ssc_zero_pa =
        static_cast<float>(ssc_sum / attempt.ssc_samples);
    attempt.ssc_valid = std::isfinite(attempt.ssc_zero_pa) &&
                        attempt.ssc_status == ESP_OK;
  }

  const esp_err_t end_result = imu.end();
  if (end_result != ESP_OK) {
    ++attempt.error_count;
    attempt.gyro_valid = false;
    attempt.gravity_valid = false;
    attempt.launcher_tilt_valid = false;
    attempt.imu_status = end_result;
  }
  rememberFirst(end_result, first);
  return first;
}

} // 無名名前空間

esp_err_t CalibrationBringup::calibrate(CalibrationAttempt &result) {
  const uint32_t previous_id = has_latest_ ? latest_.id : 0U;
  CalibrationRepeatResult repeat_result{};
  const esp_err_t status = repeat(1, repeat_result);
  result = has_latest_ && latest_.id != previous_id ? latest_
                                                    : CalibrationAttempt{};
  return status;
}

esp_err_t CalibrationBringup::repeat(uint32_t count,
                                     CalibrationRepeatResult &result) {
  result = {};
  BusyGuard guard{busy_};
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  if (count == 0 || count > kMaximumRepeatCount ||
      next_id_ + count - 1U > std::numeric_limits<uint32_t>::max())
    return ESP_ERR_INVALID_ARG;
  if (stream_.active() || spi_.encoderBusInitialized() ||
      spi_.imuBusInitialized())
    return ESP_ERR_INVALID_STATE;
  result.requested_attempts = count;

  esp_err_t first = spi_.begin();
  const bool spi_started = first == ESP_OK;

  I2CCREATE i2c{};
  const esp_err_t i2c_begin = i2c.begin(i2cConfig());
  if (i2c_begin != ESP_OK && i2c_begin != ESP_ERR_NOT_FOUND)
    rememberFirst(i2c_begin, first);
  SSCDRRN005PD2A5 ssc{};
  esp_err_t ssc_begin = ESP_ERR_NOT_FOUND;
  if (i2c_begin == ESP_OK)
    ssc_begin = ssc.begin(i2c);
  if (ssc_begin != ESP_OK && ssc_begin != ESP_ERR_NOT_FOUND)
    rememberFirst(ssc_begin, first);

  const esp_err_t stream_initialize = stream_.initialize();
  rememberFirst(stream_initialize, first);
  if (stream_initialize == ESP_OK)
    rememberFirst(stream_.beginCapture(), first);

  for (uint32_t index = 0; index < count; ++index) {
    CalibrationAttempt attempt{};
    attempt.id = static_cast<uint32_t>(next_id_++);
    attempt.ssc_status = ssc_begin;
    esp_err_t attempt_status = first;
    if (spi_started) {
      attempt_status =
          collectAttempt(spi_, imu_, ssc_begin == ESP_OK ? &ssc : nullptr,
                         attempt);
    } else {
      attempt.imu_status = first;
      ++attempt.error_count;
    }
    attempt.completed_at_us = static_cast<uint64_t>(esp_timer_get_time());
    latest_ = attempt;
    has_latest_ = true;
    ++result.completed_attempts;
    if (attempt.gyro_valid && attempt.gravity_valid)
      ++result.successful_imu_attempts;
    if (attempt.ssc_valid)
      ++result.successful_ssc_attempts;
    rememberFirst(attempt_status, first);

    if (stream_.active()) {
      const esp_err_t send = sendAttempt(stream_, attempt);
      if (send != ESP_OK)
        ++result.stream_error_count;
      rememberFirst(send, first);
    }
  }

  if (stream_.active())
    rememberFirst(stream_.finish(), first);
  result.dropped_frames = stream_.droppedFrames();
  result.output_errors = stream_.outputErrors();
  if (ssc.initialized())
    rememberFirst(ssc.end(), first);
  if (i2c.initialized())
    rememberFirst(i2c.end(), first);
  if (spi_started || spi_.encoderBusInitialized() ||
      spi_.imuBusInitialized())
    rememberFirst(spi_.end(), first);

  std::printf(
      "calibration summary: attempts=%lu/%lu imu_success=%lu ssc_success=%lu "
      "stream_error=%lu dropped=%lu output_error=%lu latest_id=%lu result=%s\n",
      static_cast<unsigned long>(result.completed_attempts),
      static_cast<unsigned long>(result.requested_attempts),
      static_cast<unsigned long>(result.successful_imu_attempts),
      static_cast<unsigned long>(result.successful_ssc_attempts),
      static_cast<unsigned long>(result.stream_error_count),
      static_cast<unsigned long>(result.dropped_frames),
      static_cast<unsigned long>(result.output_errors),
      static_cast<unsigned long>(latest_.id), esp_err_to_name(first));
  return first;
}

} // 名前空間 bringup
