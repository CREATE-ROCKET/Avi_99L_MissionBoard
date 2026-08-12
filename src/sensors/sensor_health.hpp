#pragma once

#include <cstdint>

namespace sensors {

enum class AvailabilityReason : uint8_t {
  available,
  not_initialized,
  powered_off,
  unhealthy,
  no_valid_sample,
  stale,
  calibration_missing,
};

struct FreshnessThresholds {
  // TODO(SIMULATION): Spica Phase 8と実機timestamp logで最終決定する。
  static constexpr uint64_t imu_us = 3'000;
  static constexpr uint64_t fin_us = 3'000;
  static constexpr uint64_t ssc_us = 20'000;
  static constexpr uint64_t lps_us = 120'000;
};

template <typename T> class FreshSample {
public:
  void publish(const T &value, uint64_t timestamp_us) {
    value_ = value;
    timestamp_us_ = timestamp_us;
    have_value_ = true;
  }
  void setInitialized(bool initialized) { initialized_ = initialized; }
  void setPowered(bool powered) { powered_ = powered; }
  void setHealthy(bool healthy) { healthy_ = healthy; }

  [[nodiscard]] AvailabilityReason availability(uint64_t now_us,
                                                uint64_t freshness_us) const {
    if (!initialized_)
      return AvailabilityReason::not_initialized;
    if (!powered_)
      return AvailabilityReason::powered_off;
    if (!healthy_)
      return AvailabilityReason::unhealthy;
    if (!have_value_)
      return AvailabilityReason::no_valid_sample;
    if (now_us < timestamp_us_ || now_us - timestamp_us_ > freshness_us)
      return AvailabilityReason::stale;
    return AvailabilityReason::available;
  }
  [[nodiscard]] const T &value() const { return value_; }
  [[nodiscard]] uint64_t timestampUs() const { return timestamp_us_; }
  [[nodiscard]] bool hasValue() const { return have_value_; }

private:
  T value_{};
  uint64_t timestamp_us_{};
  bool initialized_{};
  bool powered_{true};
  bool healthy_{};
  bool have_value_{};
};

class AirspeedGate {
public:
  [[nodiscard]] bool update(bool available, double airspeed_mps);
  void reset();
  [[nodiscard]] bool aboveThreshold() const { return above_threshold_; }

private:
  // TODO(SIMULATION): entry 50 ms / stop 20 msをSpica Phase 8で決定する。
  static constexpr uint16_t kEntrySamples = 50;
  static constexpr uint16_t kStopSamples = 20;
  uint16_t entry_count_{};
  uint16_t stop_count_{};
  bool above_threshold_{};
};

} // 名前空間 sensors
