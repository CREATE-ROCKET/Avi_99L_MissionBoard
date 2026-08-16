#include "sensors/display_attitude_runtime.hpp"

#include <array>
#include <atomic>
#include <cmath>

#include "bringup/imu_bringup.hpp"
#include "protocol/quantization.hpp"
#include "sensors/display_attitude_estimator.hpp"

namespace sensors::display_attitude_runtime {
namespace {

constexpr double kDegToRad = 0.017453292519943295;
constexpr double kLauncherTiltDeg = 20.0;
constexpr double kLauncherTrueAzimuthDeg = 280.66;
constexpr uint32_t kMinimumCalibrationSamples = 1'500;
constexpr uint64_t kStaleUs = 3'000;

std::atomic<uint8_t> calibration_transaction{0};
std::atomic<uint8_t> calibration_state{0}; // 0=idle, 1=active, 2=finish-success, 3=finish-failed
std::atomic<uint8_t> magnitude_raw{122};
std::atomic<uint16_t> direction_raw{511};
std::atomic<uint16_t> roll_raw{
    static_cast<uint16_t>(protocol::quantization::RollError::not_initialized)};
std::atomic<uint64_t> last_host_timestamp_us{0};

DisplayAttitudeEstimator estimator;
uint32_t gyro_samples = 0;
uint32_t accel_samples = 0;
std::array<double, 3> gyro_sum{};
std::array<double, 3> accel_sum{};
uint64_t last_sensor_timestamp_us = 0;

void publishInvalid(uint8_t tilt_raw,
                    protocol::quantization::RollError roll_error) {
  magnitude_raw.store(tilt_raw, std::memory_order_release);
  direction_raw.store(511, std::memory_order_release);
  roll_raw.store(static_cast<uint16_t>(roll_error), std::memory_order_release);
}

void resetCalibrationAccumulators() {
  gyro_samples = 0;
  accel_samples = 0;
  gyro_sum = {};
  accel_sum = {};
}

bool finite3(const std::array<double, 3> &value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

void publishEstimatorState() {
  const auto &state = estimator.state();
  if (!state.valid) {
    switch (state.invalid_reason) {
    case DisplayAttitudeInvalidReason::not_initialized:
      publishInvalid(122, protocol::quantization::RollError::not_initialized);
      break;
    case DisplayAttitudeInvalidReason::reset_invalidated:
      publishInvalid(125, protocol::quantization::RollError::reset_invalidated);
      break;
    case DisplayAttitudeInvalidReason::sample_invalid:
      publishInvalid(124, protocol::quantization::RollError::sample_invalid);
      break;
    case DisplayAttitudeInvalidReason::timestamp_invalid:
      publishInvalid(124, protocol::quantization::RollError::timestamp_invalid);
      break;
    case DisplayAttitudeInvalidReason::none:
      publishInvalid(121, protocol::quantization::RollError::unavailable);
      break;
    default:
      publishInvalid(124, protocol::quantization::RollError::estimator_invalid);
      break;
    }
    return;
  }

  const uint8_t magnitude = protocol::quantization::encodeTiltMagnitude(
      state.tilt_deg, 126);
  magnitude_raw.store(magnitude, std::memory_order_release);
  direction_raw.store(
      magnitude != 0 && magnitude <= 120 && state.direction_valid
          ? protocol::quantization::encodeTiltDirection(state.direction_deg)
          : 511,
      std::memory_order_release);
  roll_raw.store(
      protocol::quantization::encodeRoll(
          state.roll_deg, protocol::quantization::RollError::out_of_range),
      std::memory_order_release);
}

void finishCalibration(const bringup::ImuSample &sample) {
  if (gyro_samples < kMinimumCalibrationSamples ||
      accel_samples < kMinimumCalibrationSamples) {
    estimator.invalidateForDataLoss();
    publishInvalid(124, protocol::quantization::RollError::estimator_invalid);
    resetCalibrationAccumulators();
    return;
  }

  std::array<double, 3> bias{};
  std::array<double, 3> gravity{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    bias[axis] = gyro_sum[axis] / static_cast<double>(gyro_samples);
    gravity[axis] = accel_sum[axis] / static_cast<double>(accel_samples);
  }
  const double gravity_norm = std::sqrt(gravity[0] * gravity[0] +
                                        gravity[1] * gravity[1] +
                                        gravity[2] * gravity[2]);
  const bool bias_valid = finite3(bias) &&
                          std::abs(bias[0]) <= 5.0 * kDegToRad &&
                          std::abs(bias[1]) <= 5.0 * kDegToRad &&
                          std::abs(bias[2]) <= 5.0 * kDegToRad;
  const bool gravity_valid = finite3(gravity) && std::isfinite(gravity_norm) &&
                             gravity_norm >= 0.8 && gravity_norm <= 1.2;
  const uint64_t timestamp = sample.sensor_timestamp_us != 0
                                 ? sample.sensor_timestamp_us
                                 : sample.host_timestamp_us;
  if (!bias_valid || !gravity_valid || timestamp == 0 ||
      !estimator.initialize(timestamp, bias, gravity, kLauncherTiltDeg,
                            kLauncherTrueAzimuthDeg)) {
    publishInvalid(124, protocol::quantization::RollError::estimator_invalid);
  } else {
    last_sensor_timestamp_us = timestamp;
    publishEstimatorState();
  }
  resetCalibrationAccumulators();
}

} // namespace

void observe(const bringup::ImuSample &sample) {
  if (sample.host_timestamp_us != 0)
    last_host_timestamp_us.store(sample.host_timestamp_us,
                                 std::memory_order_release);

  const uint8_t state = calibration_state.load(std::memory_order_acquire);
  if (state == 3) {
    resetCalibrationAccumulators();
    calibration_state.store(0, std::memory_order_release);
    estimator.invalidateForDataLoss();
    publishInvalid(124, protocol::quantization::RollError::estimator_invalid);
  }

  const bool gyro_valid = sample.angular_velocity_valid &&
                          !sample.gyro_odr_changed &&
                          std::isfinite(sample.angular_velocity_dps[0]) &&
                          std::isfinite(sample.angular_velocity_dps[1]) &&
                          std::isfinite(sample.angular_velocity_dps[2]);
  const bool accel_valid = sample.acceleration_valid &&
                           !sample.accel_odr_changed &&
                           std::isfinite(sample.acceleration_g[0]) &&
                           std::isfinite(sample.acceleration_g[1]) &&
                           std::isfinite(sample.acceleration_g[2]);

  if (state == 1 || state == 2) {
    if (gyro_valid) {
      for (std::size_t axis = 0; axis < 3; ++axis)
        gyro_sum[axis] +=
            static_cast<double>(sample.angular_velocity_dps[axis]) * kDegToRad;
      ++gyro_samples;
    }
    if (accel_valid) {
      for (std::size_t axis = 0; axis < 3; ++axis)
        accel_sum[axis] += static_cast<double>(sample.acceleration_g[axis]);
      ++accel_samples;
    }
    if (state == 2) {
      finishCalibration(sample);
      calibration_state.store(0, std::memory_order_release);
    }
    return;
  }

  if (!estimator.state().valid)
    return;
  const uint64_t timestamp = sample.sensor_timestamp_us != 0
                                 ? sample.sensor_timestamp_us
                                 : sample.host_timestamp_us;
  const std::array<double, 3> angular_velocity{
      static_cast<double>(sample.angular_velocity_dps[0]) * kDegToRad,
      static_cast<double>(sample.angular_velocity_dps[1]) * kDegToRad,
      static_cast<double>(sample.angular_velocity_dps[2]) * kDegToRad};
  if (!gyro_valid || timestamp == 0 || timestamp <= last_sensor_timestamp_us ||
      !estimator.update(timestamp, angular_velocity, gyro_valid)) {
    estimator.invalidateForDataLoss();
    publishInvalid(124, protocol::quantization::RollError::estimator_invalid);
    return;
  }
  last_sensor_timestamp_us = timestamp;
  publishEstimatorState();
}

void calibrationAccepted(uint8_t transaction_id) {
  if (transaction_id == 0)
    return;
  calibration_transaction.store(transaction_id, std::memory_order_release);
  resetCalibrationAccumulators();
  calibration_state.store(1, std::memory_order_release);
  publishInvalid(122, protocol::quantization::RollError::not_initialized);
}

void calibrationFinished(uint8_t transaction_id, bool success) {
  if (transaction_id == 0 ||
      calibration_transaction.load(std::memory_order_acquire) != transaction_id)
    return;
  calibration_state.store(success ? 2 : 3, std::memory_order_release);
}

void invalidateForDataLoss() {
  estimator.invalidateForDataLoss();
  publishInvalid(124, protocol::quantization::RollError::estimator_invalid);
}

void invalidateForReset() {
  estimator.invalidateForReset();
  publishInvalid(125, protocol::quantization::RollError::reset_invalidated);
}

WireTelemetry wireTelemetry(uint64_t now_us) {
  const uint64_t last = last_host_timestamp_us.load(std::memory_order_acquire);
  const uint8_t magnitude = magnitude_raw.load(std::memory_order_acquire);
  if (magnitude <= 120 &&
      (last == 0 || now_us < last || now_us - last > kStaleUs))
    return {123, 511,
            static_cast<uint16_t>(protocol::quantization::RollError::stale)};
  return {magnitude, direction_raw.load(std::memory_order_acquire),
          roll_raw.load(std::memory_order_acquire)};
}

} // namespace sensors::display_attitude_runtime
