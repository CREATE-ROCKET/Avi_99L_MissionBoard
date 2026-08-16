#pragma once

#include <array>
#include <cstdint>

namespace sensors {

enum class DisplayAttitudeInvalidReason : uint8_t {
  none,
  not_initialized,
  calibration_invalid,
  sample_invalid,
  timestamp_invalid,
  numeric_error,
  reset_invalidated,
};

struct DisplayAttitudeState {
  double tilt_deg{};
  double direction_deg{};
  uint64_t timestamp_us{};
  DisplayAttitudeInvalidReason invalid_reason{
      DisplayAttitudeInvalidReason::not_initialized};
  bool direction_valid{};
  bool valid{};
};

class DisplayAttitudeEstimator {
public:
  [[nodiscard]] bool initialize(
      uint64_t timestamp_us,
      const std::array<double, 3> &gyro_bias_rad_s,
      const std::array<double, 3> &mean_acceleration_g,
      double launcher_tilt_deg, double launcher_true_azimuth_deg);
  [[nodiscard]] bool update(
      uint64_t timestamp_us,
      const std::array<double, 3> &angular_velocity_rad_s,
      bool sample_valid);
  void invalidateForDataLoss();
  void invalidateForReset();
  [[nodiscard]] const DisplayAttitudeState &state() const { return state_; }

private:
  [[nodiscard]] bool publish(uint64_t timestamp_us);
  void invalidate(DisplayAttitudeInvalidReason reason);

  std::array<double, 4> world_from_sensor_{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> gyro_bias_rad_s_{};
  std::array<double, 3> longitudinal_sensor_{0.0, 0.0, 1.0};
  DisplayAttitudeState state_{};
  uint64_t previous_timestamp_us_{};
  bool initialized_{};
};

} // namespace sensors
