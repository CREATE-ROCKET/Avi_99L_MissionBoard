#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace runtime {

enum class HardwareOwner : uint8_t {
  deployment_power,
  parachute_uart,
  mission_spi_and_motor,
  air_data_i2c,
  can_controller,
  command_dispatch,
  housekeeping_adc,
  internal_flash,
  mission_sd,
};

struct TaskDescriptor {
  const char *name;
  HardwareOwner owner;
  uint8_t priority;
  uint32_t period_us;
  bool may_block;
};

inline constexpr std::array<TaskDescriptor, 9> kTaskArchitecture{{
    {"SafetyTask", HardwareOwner::deployment_power, 22, 1'000, false},
    {"ParachuteTask", HardwareOwner::parachute_uart, 21, 10'000, true},
    {"MissionRealtimeTask", HardwareOwner::mission_spi_and_motor, 20, 1'000,
     false},
    {"AirDataTask", HardwareOwner::air_data_i2c, 18, 2'500, true},
    {"CanTask", HardwareOwner::can_controller, 16, 1'000, true},
    {"CommandWorkerTask", HardwareOwner::command_dispatch, 14, 10'000, true},
    {"HousekeepingTask", HardwareOwner::housekeeping_adc, 10, 100'000, true},
    {"InternalFlashTask", HardwareOwner::internal_flash, 8, 20'000, true},
    {"SdLogTask", HardwareOwner::mission_sd, 7, 20'000, true},
}};

template <typename T, std::size_t Capacity> class SpscBoundedQueue {
public:
  static_assert(Capacity > 0, "queue capacity must be positive");

  [[nodiscard]] bool push(const T &value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % storage_.size();
    if (next == tail_.load(std::memory_order_acquire)) {
      overflow_count_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    storage_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  [[nodiscard]] bool pop(T &value) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire))
      return false;
    value = storage_[tail];
    tail_.store((tail + 1) % storage_.size(), std::memory_order_release);
    return true;
  }

  [[nodiscard]] uint32_t overflowCount() const {
    return overflow_count_.load(std::memory_order_relaxed);
  }

private:
  // 1 slotを空けてfull/emptyを区別する。
  std::array<T, Capacity + 1> storage_{};
  std::atomic<std::size_t> head_{};
  std::atomic<std::size_t> tail_{};
  std::atomic<uint32_t> overflow_count_{};
};

} // 名前空間 runtime
