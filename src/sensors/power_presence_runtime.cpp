#include "sensors/power_presence_runtime.hpp"

#include "sensors/power_presence.hpp"

namespace sensors::power_presence_runtime {
namespace {

// TODO(HW_TEST): 実機ADC captureからON/OFF閾値、hysteresis、debounce、
// stale timeoutを確定する。未確定中はFlight Status bit5/6を0とする。
constexpr PowerPresenceConfig kLogicConfig{};
constexpr PowerPresenceConfig kMotorConfig{};
PowerPresenceDetector logic_detector{kLogicConfig};
PowerPresenceDetector motor_detector{kMotorConfig};

void observeOne(PowerPresenceDetector &detector, uint8_t raw,
                uint64_t timestamp_us) {
  const bool valid = raw <= 240;
  detector.update(timestamp_us, valid ? static_cast<double>(raw) * 0.05 : 0.0,
                  valid);
}

} // namespace

void observeRaw(uint8_t logic_voltage_raw, uint8_t motor_voltage_raw,
                uint64_t timestamp_us) {
  observeOne(logic_detector, logic_voltage_raw, timestamp_us);
  observeOne(motor_detector, motor_voltage_raw, timestamp_us);
}

bool logicPresent(uint64_t now_us) { return logic_detector.present(now_us); }
bool motorPresent(uint64_t now_us) { return motor_detector.present(now_us); }

} // namespace sensors::power_presence_runtime
