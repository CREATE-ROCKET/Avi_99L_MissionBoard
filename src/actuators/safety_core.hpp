#pragma once

#include <cstdint>

#include "actuators/parachute_configuration.hpp"

namespace actuators {

struct PowerState {
  bool auxiliary_5v{};
  bool parachute_power{};
  bool cutoff_latched{};
};

class PowerArbiter {
public:
  [[nodiscard]] bool requestAuxiliary5v(bool enabled);
  [[nodiscard]] bool requestParachutePower(bool enabled);
  void latchDeploymentCutoff();
  [[nodiscard]] const PowerState &state() const { return state_; }

private:
  PowerState state_{};
};

enum class ParachuteAction : uint8_t {
  none,
  command_open,
  retry_open,
  hold_open,
  stop_retrying,
};

enum class ParachuteOpenState : uint8_t {
  idle,
  opening,
  retrying,
  open_confirmed,
  retry_exhausted,
  powered_off,
};

struct ParachuteTick {
  uint64_t now_us{};
  bool position_valid{};
  uint16_t position_count{};
  bool target_reached{};
};

struct ParachuteStatus {
  ParachuteOpenState state{ParachuteOpenState::idle};
  uint32_t retry_count{};
  bool servo_open_confirmed{};
  bool open_attempt_finished{};
};

class ParachuteController {
public:
  [[nodiscard]] ParachuteAction startOpen(uint64_t now_us,
                                         uint16_t initial_position_count);
  [[nodiscard]] ParachuteAction tick(const ParachuteTick &input);
  void notifyPowerCutoff();
  [[nodiscard]] const ParachuteStatus &status() const { return status_; }

private:
  // TODO(HW_TEST): STS3215 speed/torque、2 deg/0.5 s、5 s deadlineを確定する。
  static constexpr uint64_t kProgressWindowUs = 500'000;
  static constexpr uint64_t kGlobalDeadlineUs = 5'000'000;
  static constexpr int16_t kMinimumProgressCount = 23;

  ParachuteStatus status_{};
  uint64_t started_at_us_{};
  uint64_t window_started_at_us_{};
  uint16_t window_position_count_{};
};

} // 名前空間 actuators
