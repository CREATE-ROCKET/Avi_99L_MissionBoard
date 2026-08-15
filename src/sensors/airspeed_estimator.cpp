#include "sensors/airspeed_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace sensors {
namespace {

constexpr double kSpecificGasConstantAir = 287.05;
constexpr double kHeatCapacityRatioAir = 1.4;
constexpr double kKelvinOffset = 273.15;

} // 無名名前空間

AirspeedComputation
computeSaintVenantAirspeed(double static_pressure_pa,
                          double differential_pressure_pa,
                          double temperature_celsius,
                          double pitot_pressure_correction_coefficient) {
  if (!std::isfinite(static_pressure_pa) ||
      !std::isfinite(differential_pressure_pa) ||
      !std::isfinite(temperature_celsius) ||
      !std::isfinite(pitot_pressure_correction_coefficient) ||
      pitot_pressure_correction_coefficient <= 0.0)
    return {0.0, AirspeedModelError::non_finite_input, false};
  if (static_pressure_pa <= 0.0)
    return {0.0, AirspeedModelError::static_pressure_invalid, false};
  const double temperature_kelvin = temperature_celsius + kKelvinOffset;
  if (temperature_kelvin <= 0.0)
    return {0.0, AirspeedModelError::temperature_invalid, false};
  if (differential_pressure_pa < 0.0)
    return {0.0, AirspeedModelError::negative_differential_pressure, false};
  if (differential_pressure_pa == 0.0)
    return {0.0, AirspeedModelError::none, true};

  // The sourced K is a pressure correction coefficient: K^2*q_c/P_s is
  // inside the Saint-Venant pressure ratio.  Applying K after sqrt is a
  // different nonlinear model and breaks Mission/Spica parity.
  const double corrected_pressure_ratio =
      pitot_pressure_correction_coefficient *
      pitot_pressure_correction_coefficient * differential_pressure_pa /
      static_pressure_pa;
  if (!std::isfinite(corrected_pressure_ratio) ||
      corrected_pressure_ratio < 0.0)
    return {0.0, AirspeedModelError::numeric_error, false};
  const double exponent =
      (kHeatCapacityRatioAir - 1.0) / kHeatCapacityRatioAir;
  const double expansion =
      std::expm1(exponent * std::log1p(corrected_pressure_ratio));
  const double radicand =
      (2.0 * kHeatCapacityRatioAir / (kHeatCapacityRatioAir - 1.0)) *
      kSpecificGasConstantAir * temperature_kelvin * expansion;
  if (!std::isfinite(radicand) || radicand < 0.0)
    return {0.0, AirspeedModelError::numeric_error, false};
  const double airspeed = std::sqrt(radicand);
  if (!std::isfinite(airspeed))
    return {0.0, AirspeedModelError::numeric_error, false};
  return {airspeed, AirspeedModelError::none, true};
}

DifferentialPressureConditioner::DifferentialPressureConditioner(
    uint16_t zero_calibration_samples, uint8_t moving_average_samples,
    double negative_tolerance_pa)
    : required_zero_samples_(zero_calibration_samples),
      moving_average_samples_(static_cast<uint8_t>(std::min<std::size_t>(
          moving_average_samples, kMaximumMovingAverageSamples))),
      negative_tolerance_pa_(negative_tolerance_pa) {}

void DifferentialPressureConditioner::reset() {
  zero_sum_pa_ = 0.0;
  zero_offset_pa_ = 0.0;
  zero_count_ = 0;
  zero_valid_ = false;
  window_.fill(0.0);
  window_head_ = 0;
  window_count_ = 0;
  window_sum_pa_ = 0.0;
}

bool DifferentialPressureConditioner::updateZero(
    double differential_pressure_pa, bool calibration_allowed) {
  if (zero_valid_ || !calibration_allowed ||
      !std::isfinite(differential_pressure_pa) ||
      required_zero_samples_ == 0)
    return zero_valid_;
  zero_sum_pa_ += differential_pressure_pa;
  ++zero_count_;
  if (zero_count_ < required_zero_samples_)
    return false;
  zero_offset_pa_ = zero_sum_pa_ / static_cast<double>(zero_count_);
  zero_valid_ = std::isfinite(zero_offset_pa_);
  window_.fill(0.0);
  window_head_ = 0;
  window_count_ = 0;
  window_sum_pa_ = 0.0;
  return zero_valid_;
}

ConditionedDifferentialPressure
DifferentialPressureConditioner::update(double differential_pressure_pa) {
  ConditionedDifferentialPressure result{};
  result.zero_valid = zero_valid_;
  if (!zero_valid_ || !std::isfinite(differential_pressure_pa) ||
      moving_average_samples_ == 0 ||
      !std::isfinite(negative_tolerance_pa_) ||
      negative_tolerance_pa_ < 0.0)
    return result;

  double corrected = differential_pressure_pa - zero_offset_pa_;
  if (corrected < -negative_tolerance_pa_) {
    result.negative_beyond_tolerance = true;
    return result;
  }
  corrected = std::max(0.0, corrected);
  if (window_count_ == moving_average_samples_) {
    window_sum_pa_ -= window_[window_head_];
  } else {
    ++window_count_;
  }
  window_[window_head_] = corrected;
  window_sum_pa_ += corrected;
  window_head_ = (window_head_ + 1U) % moving_average_samples_;
  result.pressure_pa = window_sum_pa_ / static_cast<double>(window_count_);
  result.valid = std::isfinite(result.pressure_pa);
  return result;
}

} // 名前空間 sensors
