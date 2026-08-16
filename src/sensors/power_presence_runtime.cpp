#include "sensors/power_presence_runtime.hpp"

#include "sensors/power_presence.hpp"

namespace sensors::power_presence_runtime {
namespace {

// TODO(HW_TEST): 本番値は実機ADC captureで更新する。
// 現段階ではundervoltage判定には使わず、「railが実質0 Vではない」ことだけを
// telemetryへ反映するための保守的なpresence閾値とする。
constexpr PowerPresenceConfig kLogicConfig{
    true, 1.0, 0.5, 3, 2, 500'000};
constexpr PowerPresenceConfig kMotorConfig{
    true, 1.0, 0.5, 3, 2, 500'000};
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
