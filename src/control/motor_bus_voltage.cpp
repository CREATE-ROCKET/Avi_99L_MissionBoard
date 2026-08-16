#include "control/motor_bus_voltage.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace control::motor_bus_voltage {
namespace {

std::atomic<uint8_t> status_raw{
    static_cast<uint8_t>(Status::uninitialized)};
std::atomic<uint64_t> voltage_bits{};

uint64_t encode(double value) {
  uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double decode(uint64_t bits) {
  double value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

} // namespace

void publish(double voltage_v) {
  if (!std::isfinite(voltage_v) || voltage_v <= 0.0) {
    invalidate();
    return;
  }
  voltage_bits.store(encode(voltage_v), std::memory_order_relaxed);
  status_raw.store(static_cast<uint8_t>(Status::live),
                   std::memory_order_release);
}

void invalidate() {
  status_raw.store(static_cast<uint8_t>(Status::invalid),
                   std::memory_order_release);
}

void reset() {
  voltage_bits.store(0, std::memory_order_relaxed);
  status_raw.store(static_cast<uint8_t>(Status::uninitialized),
                   std::memory_order_release);
}

Snapshot snapshot() {
  const auto status = static_cast<Status>(
      status_raw.load(std::memory_order_acquire));
  if (status != Status::live)
    return {status, 0.0};
  const double voltage_v =
      decode(voltage_bits.load(std::memory_order_relaxed));
  if (!std::isfinite(voltage_v) || voltage_v <= 0.0)
    return {Status::invalid, 0.0};
  return {Status::live, voltage_v};
}

} // namespace control::motor_bus_voltage
