#include "sensors/display_attitude_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace sensors {
namespace {

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<std::array<double, 3>, 3>;
struct Quaternion { double w{1.0}; double x{}; double y{}; double z{}; };

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kVectorEpsilon = 1.0e-9;

bool finite(const Vector3 &value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

double dot(const Vector3 &left, const Vector3 &right) {
  return left[0] * right[0] + left[1] * right[1] +
         left[2] * right[2];
}

Vector3 subtract(const Vector3 &left, const Vector3 &right) {
  return {left[0] - right[0], left[1] - right[1], left[2] - right[2]};
}

Vector3 scale(const Vector3 &value, double factor) {
  return {value[0] * factor, value[1] * factor, value[2] * factor};
}

Vector3 cross(const Vector3 &left, const Vector3 &right) {
  return {left[1] * right[2] - left[2] * right[1],
          left[2] * right[0] - left[0] * right[2],
          left[0] * right[1] - left[1] * right[0]};
}

double norm(const Vector3 &value) { return std::sqrt(dot(value, value)); }

bool normalize(Vector3 &value) {
  const double length = norm(value);
  if (!std::isfinite(length) || length <= kVectorEpsilon)
    return false;
  value = scale(value, 1.0 / length);
  return finite(value);
}

Quaternion multiply(const Quaternion &left, const Quaternion &right) {
  return {
      left.w * right.w - left.x * right.x - left.y * right.y -
          left.z * right.z,
      left.w * right.x + left.x * right.w + left.y * right.z -
          left.z * right.y,
      left.w * right.y - left.x * right.z + left.y * right.w +
          left.z * right.x,
      left.w * right.z + left.x * right.y - left.y * right.x +
          left.z * right.w,
  };
}

bool normalize(Quaternion &value) {
  const double length = std::sqrt(value.w * value.w + value.x * value.x +
                                  value.y * value.y + value.z * value.z);
  if (!std::isfinite(length) || length <= kVectorEpsilon)
    return false;
  value.w /= length;
  value.x /= length;
  value.y /= length;
  value.z /= length;
  return std::isfinite(value.w) && std::isfinite(value.x) &&
         std::isfinite(value.y) && std::isfinite(value.z);
}

Vector3 rotate(const Quaternion &q, const Vector3 &value) {
  const Quaternion vector{0.0, value[0], value[1], value[2]};
  const Quaternion conjugate{q.w, -q.x, -q.y, -q.z};
  const Quaternion rotated = multiply(multiply(q, vector), conjugate);
  return {rotated.x, rotated.y, rotated.z};
}

Quaternion fromMatrix(const Matrix3 &m) {
  Quaternion q{};
  const double trace = m[0][0] + m[1][1] + m[2][2];
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (m[2][1] - m[1][2]) / s;
    q.y = (m[0][2] - m[2][0]) / s;
    q.z = (m[1][0] - m[0][1]) / s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q.w = (m[2][1] - m[1][2]) / s;
    q.x = 0.25 * s;
    q.y = (m[0][1] + m[1][0]) / s;
    q.z = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    q.w = (m[0][2] - m[2][0]) / s;
    q.x = (m[0][1] + m[1][0]) / s;
    q.y = 0.25 * s;
    q.z = (m[1][2] + m[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    q.w = (m[1][0] - m[0][1]) / s;
    q.x = (m[0][2] + m[2][0]) / s;
    q.y = (m[1][2] + m[2][1]) / s;
    q.z = 0.25 * s;
  }
  (void)normalize(q);
  return q;
}

Matrix3 mapBasis(const Vector3 &sensor_horizontal,
                 const Vector3 &sensor_lateral, const Vector3 &sensor_up,
                 const Vector3 &world_horizontal,
                 const Vector3 &world_lateral, const Vector3 &world_up) {
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row][column] =
          world_horizontal[row] * sensor_horizontal[column] +
          world_lateral[row] * sensor_lateral[column] +
          world_up[row] * sensor_up[column];
    }
  }
  return result;
}

} // namespace

bool DisplayAttitudeEstimator::initialize(
    uint64_t timestamp_us, const std::array<double, 3> &gyro_bias_rad_s,
    const std::array<double, 3> &mean_acceleration_g,
    double launcher_tilt_deg, double launcher_true_azimuth_deg) {
  if (timestamp_us == 0 || !finite(gyro_bias_rad_s) ||
      !finite(mean_acceleration_g) || !std::isfinite(launcher_tilt_deg) ||
      !std::isfinite(launcher_true_azimuth_deg) || launcher_tilt_deg <= 0.0 ||
      launcher_tilt_deg >= 90.0) {
    invalidate(DisplayAttitudeInvalidReason::calibration_invalid);
    return false;
  }

  Vector3 sensor_up = mean_acceleration_g;
  if (!normalize(sensor_up)) {
    invalidate(DisplayAttitudeInvalidReason::calibration_invalid);
    return false;
  }

  // 既存roll実装はICM Z軸を機体longitudinal軸としている。
  // 実装向きの+/-符号はlauncher上の既知tiltに近い側を選び、
  // 基板silkscreen等の未確認情報をコードへ埋め込まない。
  const Vector3 positive_longitudinal{0.0, 0.0, 1.0};
  const Vector3 negative_longitudinal{0.0, 0.0, -1.0};
  const auto candidate_tilt_deg = [&](const Vector3 &candidate) {
    return std::acos(std::clamp(dot(candidate, sensor_up), -1.0, 1.0)) *
           kRadToDeg;
  };
  longitudinal_sensor_ =
      std::abs(candidate_tilt_deg(positive_longitudinal) -
               launcher_tilt_deg) <=
              std::abs(candidate_tilt_deg(negative_longitudinal) -
                       launcher_tilt_deg)
          ? positive_longitudinal
          : negative_longitudinal;
  Vector3 sensor_horizontal = subtract(
      longitudinal_sensor_, scale(sensor_up, dot(longitudinal_sensor_, sensor_up)));
  if (!normalize(sensor_horizontal)) {
    invalidate(DisplayAttitudeInvalidReason::calibration_invalid);
    return false;
  }
  Vector3 sensor_lateral = cross(sensor_up, sensor_horizontal);
  if (!normalize(sensor_lateral)) {
    invalidate(DisplayAttitudeInvalidReason::calibration_invalid);
    return false;
  }

  const double tilt_rad = launcher_tilt_deg * kDegToRad;
  const double azimuth_rad = launcher_true_azimuth_deg * kDegToRad;
  Vector3 world_longitudinal{
      std::sin(tilt_rad) * std::sin(azimuth_rad),
      std::sin(tilt_rad) * std::cos(azimuth_rad), std::cos(tilt_rad)};
  Vector3 world_up{0.0, 0.0, 1.0};
  Vector3 world_horizontal = subtract(
      world_longitudinal, scale(world_up, dot(world_longitudinal, world_up)));
  if (!normalize(world_horizontal)) {
    invalidate(DisplayAttitudeInvalidReason::calibration_invalid);
    return false;
  }
  Vector3 world_lateral = cross(world_up, world_horizontal);
  if (!normalize(world_lateral)) {
    invalidate(DisplayAttitudeInvalidReason::calibration_invalid);
    return false;
  }

  const Quaternion initial = fromMatrix(mapBasis(
      sensor_horizontal, sensor_lateral, sensor_up, world_horizontal,
      world_lateral, world_up));
  world_from_sensor_ = {initial.w, initial.x, initial.y, initial.z};
  gyro_bias_rad_s_ = gyro_bias_rad_s;
  previous_timestamp_us_ = timestamp_us;
  initialized_ = true;
  if (!publish(timestamp_us)) {
    invalidate(DisplayAttitudeInvalidReason::numeric_error);
    return false;
  }
  return true;
}

bool DisplayAttitudeEstimator::update(
    uint64_t timestamp_us,
    const std::array<double, 3> &angular_velocity_rad_s,
    bool sample_valid) {
  if (!initialized_) {
    invalidate(DisplayAttitudeInvalidReason::not_initialized);
    return false;
  }
  if (!sample_valid || !finite(angular_velocity_rad_s)) {
    initialized_ = false;
    invalidate(DisplayAttitudeInvalidReason::sample_invalid);
    return false;
  }
  if (timestamp_us <= previous_timestamp_us_) {
    initialized_ = false;
    invalidate(DisplayAttitudeInvalidReason::timestamp_invalid);
    return false;
  }

  Vector3 corrected{
      angular_velocity_rad_s[0] - gyro_bias_rad_s_[0],
      angular_velocity_rad_s[1] - gyro_bias_rad_s_[1],
      angular_velocity_rad_s[2] - gyro_bias_rad_s_[2],
  };
  if (!finite(corrected)) {
    initialized_ = false;
    invalidate(DisplayAttitudeInvalidReason::numeric_error);
    return false;
  }

  const double dt = static_cast<double>(timestamp_us - previous_timestamp_us_) / 1.0e6;
  const double rate = norm(corrected);
  Quaternion delta{};
  if (rate > kVectorEpsilon) {
    const double half_angle = 0.5 * rate * dt;
    const double scale_value = std::sin(half_angle) / rate;
    delta = {std::cos(half_angle), corrected[0] * scale_value,
             corrected[1] * scale_value, corrected[2] * scale_value};
  }
  Quaternion current{world_from_sensor_[0], world_from_sensor_[1], world_from_sensor_[2], world_from_sensor_[3]};
  current = multiply(current, delta);
  if (!normalize(current)) {
    initialized_ = false;
    invalidate(DisplayAttitudeInvalidReason::numeric_error);
    return false;
  }
  world_from_sensor_ = {current.w, current.x, current.y, current.z};
  previous_timestamp_us_ = timestamp_us;
  if (!publish(timestamp_us)) {
    initialized_ = false;
    invalidate(DisplayAttitudeInvalidReason::numeric_error);
    return false;
  }
  return true;
}

void DisplayAttitudeEstimator::invalidateForDataLoss() {
  initialized_ = false;
  invalidate(DisplayAttitudeInvalidReason::sample_invalid);
}

void DisplayAttitudeEstimator::invalidateForReset() {
  initialized_ = false;
  invalidate(DisplayAttitudeInvalidReason::reset_invalidated);
}

bool DisplayAttitudeEstimator::publish(uint64_t timestamp_us) {
  const Quaternion current{world_from_sensor_[0], world_from_sensor_[1], world_from_sensor_[2], world_from_sensor_[3]};
  Vector3 longitudinal_world = rotate(current, longitudinal_sensor_);
  if (!normalize(longitudinal_world))
    return false;
  const double up = std::clamp(longitudinal_world[2], -1.0, 1.0);
  const double tilt_deg = std::acos(up) * kRadToDeg;
  const double horizontal =
      std::hypot(longitudinal_world[0], longitudinal_world[1]);
  double direction_deg = 0.0;
  const bool direction_valid = horizontal > kVectorEpsilon;
  if (direction_valid) {
    direction_deg =
        std::atan2(longitudinal_world[0], longitudinal_world[1]) * kRadToDeg;
    if (direction_deg < 0.0)
      direction_deg += 360.0;
  }
  if (!std::isfinite(tilt_deg) || !std::isfinite(direction_deg))
    return false;
  state_ = {tilt_deg, direction_deg, timestamp_us,
            DisplayAttitudeInvalidReason::none, direction_valid, true};
  return true;
}

void DisplayAttitudeEstimator::invalidate(DisplayAttitudeInvalidReason reason) {
  state_.valid = false;
  state_.direction_valid = false;
  state_.invalid_reason = reason;
}

} // namespace sensors
