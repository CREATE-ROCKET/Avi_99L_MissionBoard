#pragma once

#include <cstdint>

namespace avi::characterization {

enum class ShutdownStep : std::uint8_t {
  Coast = 0,
  Disarm = 1,
  SamplingStop = 2,
  RealtimeQueueDrain = 3,
  WriterQueueDrain = 4,
  Sync = 5,
  Footer = 6,
  Close = 7,
  PersistM0 = 8,
  PowerCycleRequired = 9,
};

class ShutdownSequence {
public:
  [[nodiscard]] bool mark(ShutdownStep step) noexcept {
    if (static_cast<std::uint8_t>(step) != next_)
      return false;
    mask_ |= 1U << next_;
    ++next_;
    return true;
  }

  [[nodiscard]] std::uint32_t mask() const noexcept { return mask_; }
  [[nodiscard]] std::uint8_t completedSteps() const noexcept {
    return next_;
  }

private:
  std::uint32_t mask_{0U};
  std::uint8_t next_{0U};
};

} // 名前空間 avi::characterization
