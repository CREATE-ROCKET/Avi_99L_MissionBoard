#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sensors {

struct GyroSample {
  uint64_t timestamp_us{};
  double roll_rate_rad_s{};
  uint32_t timestamp_epoch{};
  uint16_t lost_packets{};
  bool valid{};
  bool saturated{};
  bool fifo_full{};
  bool format_fault{};
};

class GyroHistoryRing {
public:
  static constexpr std::size_t kCapacity = 1'200;

  void push(const GyroSample &sample);
  void clear();
  [[nodiscard]] std::size_t size() const { return count_; }
  [[nodiscard]] const GyroSample &at(std::size_t chronological_index) const;

private:
  std::array<GyroSample, kCapacity> samples_{};
  std::size_t head_{};
  std::size_t count_{};
};

enum class AttitudeInvalidReason : uint8_t {
  none,
  history_unavailable,
  sample_invalid,
  saturated,
  fifo_format_fault,
  excess_data_loss,
  timestamp_invalid,
  timestamp_epoch_changed,
  numeric_error,
  reset_invalidated,
};

struct AttitudeState {
  double roll_rad{};
  double roll_rate_rad_s{};
  uint64_t timestamp_us{};
  uint32_t interpolated_sample_count{};
  uint32_t data_loss_event_count{};
  uint32_t fifo_full_event_count{};
  AttitudeInvalidReason invalid_reason{AttitudeInvalidReason::history_unavailable};
  bool valid{};
};

class AttitudeEstimator {
public:
  [[nodiscard]] bool beginFlight(const GyroHistoryRing &history,
                                 uint64_t liftoff_time_us,
                                 double gyro_bias_rad_s);
  [[nodiscard]] bool update(const GyroSample &sample);
  void invalidateForReset();
  [[nodiscard]] const AttitudeState &state() const { return state_; }

private:
  [[nodiscard]] bool accept(const GyroSample &sample);
  void invalidate(AttitudeInvalidReason reason);

  AttitudeState state_{};
  double bias_rad_s_{};
  double previous_rate_rad_s_{};
  uint64_t previous_timestamp_us_{};
  uint32_t timestamp_epoch_{};
  bool seeded_{};
};

} // 名前空間 sensors
