#pragma once

#include <cstdint>

namespace mission {

enum class PreflightReadinessBit : uint8_t {
  fin_zero_configured = 0,
  parachute_open_configured = 1,
  parachute_close_configured = 2,
  motor_profile_valid = 3,
  gyro_bias_valid = 4,
  gravity_reference_valid = 5,
  ssc_zero_valid = 6,
};

[[nodiscard]] constexpr uint8_t preflightReadinessBit(
    PreflightReadinessBit bit) {
  return static_cast<uint8_t>(1U << static_cast<uint8_t>(bit));
}

struct PreflightReadinessSnapshot {
  uint32_t generation{};
  uint64_t captured_at_us{};
  bool fin_zero_configured{};
  bool parachute_open_configured{};
  bool parachute_close_configured{};
  bool motor_profile_valid{};
  bool gyro_bias_valid{};
  bool gravity_reference_valid{};
  bool ssc_zero_valid{};
  bool resources_preallocated{};

  [[nodiscard]] constexpr uint8_t missingMask() const {
    uint8_t mask = 0;
    if (!fin_zero_configured)
      mask |= preflightReadinessBit(
          PreflightReadinessBit::fin_zero_configured);
    if (!parachute_open_configured)
      mask |= preflightReadinessBit(
          PreflightReadinessBit::parachute_open_configured);
    if (!parachute_close_configured)
      mask |= preflightReadinessBit(
          PreflightReadinessBit::parachute_close_configured);
    if (!motor_profile_valid)
      mask |= preflightReadinessBit(
          PreflightReadinessBit::motor_profile_valid);
    if (!gyro_bias_valid)
      mask |= preflightReadinessBit(
          PreflightReadinessBit::gyro_bias_valid);
    if (!gravity_reference_valid)
      mask |= preflightReadinessBit(
          PreflightReadinessBit::gravity_reference_valid);
    if (!ssc_zero_valid)
      mask |= preflightReadinessBit(
          PreflightReadinessBit::ssc_zero_valid);
    return mask;
  }

  [[nodiscard]] constexpr bool ready() const {
    return resources_preallocated && missingMask() == 0;
  }
};

} // 名前空間 mission
