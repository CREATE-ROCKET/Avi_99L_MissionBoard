#include "control/motor_bus_voltage.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

namespace control::motor_bus_voltage {
namespace {

#if defined(ESP_PLATFORM)
constexpr Status kResetStatus = Status::invalid;
#else
// native/host testでは既存TorqueMapper APIのcaller supplied Vbusを使用できる。
constexpr Status kResetStatus = Status::uninitialized;
#endif

std::atomic<uint8_t> status_raw{static_cast<uint8_t>(kResetStatus)};
std::atomic<uint32_t> voltage_millivolts{};

} // namespace

void publish(double voltage_v) {
  if (!std::isfinite(voltage_v) || voltage_v <= 0.0) {
    invalidate();
    return;
  }
  const double millivolts = voltage_v * 1000.0;
  if (!std::isfinite(millivolts) || millivolts < 1.0 ||
      millivolts > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
    invalidate();
    return;
  }
  voltage_millivolts.store(static_cast<uint32_t>(millivolts + 0.5),
                           std::memory_order_relaxed);
  status_raw.store(static_cast<uint8_t>(Status::live),
                   std::memory_order_release);
}

void invalidate() {
  status_raw.store(static_cast<uint8_t>(Status::invalid),
                   std::memory_order_release);
}

void reset() {
  voltage_millivolts.store(0, std::memory_order_relaxed);
  status_raw.store(static_cast<uint8_t>(kResetStatus),
                   std::memory_order_release);
}

Snapshot snapshot() {
  const auto status =
      static_cast<Status>(status_raw.load(std::memory_order_acquire));
  if (status != Status::live)
    return {status, 0.0};
  const uint32_t millivolts =
      voltage_millivolts.load(std::memory_order_relaxed);
  if (millivolts == 0)
    return {Status::invalid, 0.0};
  return {Status::live, static_cast<double>(millivolts) * 0.001};
}

} // namespace control::motor_bus_voltage
