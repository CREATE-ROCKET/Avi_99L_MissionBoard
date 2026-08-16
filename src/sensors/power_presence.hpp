#pragma once

#include <cstdint>

namespace sensors {

struct PowerPresenceConfig {
  bool configured{};
  double present_on_voltage_v{};
  double present_off_voltage_v{};
  uint16_t assert_samples{};
  uint16_t clear_samples{};
  uint64_t stale_timeout_us{};
};

class PowerPresenceDetector {
public:
  explicit PowerPresenceDetector(PowerPresenceConfig config = {})
      : config_(config) {}

  void reset();
  void update(uint64_t timestamp_us, double voltage_v, bool sample_valid);
  [[nodiscard]] bool present(uint64_t now_us) const;
  [[nodiscard]] bool configured() const { return config_.configured; }

private:
  PowerPresenceConfig config_{};
  uint64_t last_valid_timestamp_us_{};
  uint16_t assert_count_{};
  uint16_t clear_count_{};
  bool present_{};
};

} // namespace sensors
