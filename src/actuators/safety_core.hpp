#pragma once

#include <cstdint>

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
  cut_power,
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
  double position_deg{};
  bool target_reached{};
};

struct ParachuteStatus {
  ParachuteOpenState state{ParachuteOpenState::idle};
  uint32_t retry_count{};
  bool servo_open_confirmed{};
  bool power_cutoff_requested{};
};

class ParachuteController {
public:
  [[nodiscard]] ParachuteAction startOpen(uint64_t now_us,
                                         double initial_position_deg);
  [[nodiscard]] ParachuteAction tick(const ParachuteTick &input);
  void notifyPowerCutoff();
  [[nodiscard]] const ParachuteStatus &status() const { return status_; }

private:
  // TODO(HW_TEST): STS3215 speed/torque、2 deg/0.5 s、5 s deadlineを確定する。
  static constexpr uint64_t kProgressWindowUs = 500'000;
  static constexpr uint64_t kGlobalDeadlineUs = 5'000'000;
  static constexpr double kMinimumProgressDeg = 2.0;

  ParachuteStatus status_{};
  uint64_t started_at_us_{};
  uint64_t window_started_at_us_{};
  double window_position_deg_{};
};

} // 名前空間 actuators
