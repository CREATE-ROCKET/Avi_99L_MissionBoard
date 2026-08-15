#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sensors {

enum class AirspeedModelError : uint8_t {
  none,
  non_finite_input,
  static_pressure_invalid,
  temperature_invalid,
  negative_differential_pressure,
  numeric_error,
};

struct AirspeedComputation {
  double airspeed_mps{};
  AirspeedModelError error{AirspeedModelError::non_finite_input};
  bool valid{};
};

[[nodiscard]] AirspeedComputation
computeSaintVenantAirspeed(double static_pressure_pa,
                          double differential_pressure_pa,
                          double temperature_celsius,
                          double pitot_coefficient);

struct ConditionedDifferentialPressure {
  double pressure_pa{};
  bool zero_valid{};
  bool valid{};
  bool negative_beyond_tolerance{};
};

class DifferentialPressureConditioner {
public:
  static constexpr std::size_t kMaximumMovingAverageSamples = 32;

  DifferentialPressureConditioner(uint16_t zero_calibration_samples,
                                  uint8_t moving_average_samples,
                                  double negative_tolerance_pa);

  void reset();
  [[nodiscard]] bool updateZero(double differential_pressure_pa,
                                bool calibration_allowed);
  [[nodiscard]] ConditionedDifferentialPressure
  update(double differential_pressure_pa);
  [[nodiscard]] bool zeroValid() const { return zero_valid_; }
  [[nodiscard]] double zeroOffsetPa() const { return zero_offset_pa_; }
  [[nodiscard]] uint16_t zeroSampleCount() const { return zero_count_; }

private:
  uint16_t required_zero_samples_{};
  uint8_t moving_average_samples_{};
  double negative_tolerance_pa_{};
  double zero_sum_pa_{};
  double zero_offset_pa_{};
  uint16_t zero_count_{};
  bool zero_valid_{};
  std::array<double, kMaximumMovingAverageSamples> window_{};
  std::size_t window_head_{};
  std::size_t window_count_{};
  double window_sum_pa_{};
};

} // 名前空間 sensors
