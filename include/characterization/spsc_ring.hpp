#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace avi::characterization {

// producerとconsumerを各1 taskに限定した固定長queue。
// producerは完成済みrecordだけをpublishし、consumerはlive stateを参照しない。
template <typename T, std::size_t Capacity> class SpscRing {
  static_assert(Capacity >= 2U);
  static_assert(std::is_trivially_copyable_v<T>);

public:
  [[nodiscard]] bool push(const T &value) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = increment(head);
    if (next == tail_.load(std::memory_order_acquire))
      return false;
    storage_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  [[nodiscard]] bool pop(T &value) noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire))
      return false;
    value = storage_[tail];
    tail_.store(increment(tail), std::memory_order_release);
    return true;
  }

  [[nodiscard]] bool empty() const noexcept {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

  // producer/consumer停止中にだけ呼び、次runへ古い値を持ち越さない。
  void reset() noexcept {
    head_.store(0U, std::memory_order_relaxed);
    tail_.store(0U, std::memory_order_relaxed);
  }

private:
  static constexpr std::size_t increment(std::size_t value) noexcept {
    return (value + 1U) % Capacity;
  }

  std::array<T, Capacity> storage_{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
};

} // 名前空間 avi::characterization
