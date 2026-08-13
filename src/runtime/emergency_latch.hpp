#pragma once

#include <atomic>
#include <cstdint>

namespace runtime {

class EmergencyLatch {
public:
  bool signal(uint8_t transaction_id) {
    return (pending_.exchange(static_cast<uint16_t>(transaction_id) | 0x100U,
                              std::memory_order_acq_rel) &
            0x100U) != 0;
  }

  [[nodiscard]] bool take(uint8_t &transaction_id) {
    const uint16_t pending = pending_.exchange(0, std::memory_order_acq_rel);
    if ((pending & 0x100U) == 0)
      return false;
    transaction_id = static_cast<uint8_t>(pending);
    return true;
  }

  [[nodiscard]] bool pending() const {
    return (pending_.load(std::memory_order_acquire) & 0x100U) != 0;
  }

private:
  std::atomic<uint16_t> pending_{};
};

} // 名前空間 runtime
