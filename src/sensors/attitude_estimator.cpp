#include "sensors/attitude_estimator.hpp"

#include <cmath>

namespace sensors {
namespace {

constexpr uint64_t kNominalSampleUs = 1'000;
constexpr uint64_t kTimestampToleranceUs = 250;

bool nearPeriod(uint64_t delta_us, uint64_t periods) {
  const uint64_t expected = periods * kNominalSampleUs;
  return delta_us >= expected - kTimestampToleranceUs &&
         delta_us <= expected + kTimestampToleranceUs;
}

} // 無名名前空間

void GyroHistoryRing::push(const GyroSample &sample) {
  samples_[head_] = sample;
  head_ = (head_ + 1) % samples_.size();
  if (count_ < samples_.size())
    ++count_;
}

void GyroHistoryRing::clear() { *this = {}; }

const GyroSample &
GyroHistoryRing::at(std::size_t chronological_index) const {
  const std::size_t oldest =
      count_ == samples_.size() ? head_ : (head_ + samples_.size() - count_) %
                                             samples_.size();
  return samples_[(oldest + chronological_index) % samples_.size()];
}

bool AttitudeEstimator::beginFlight(const GyroHistoryRing &history,
                                    uint64_t liftoff_time_us,
                                    double gyro_bias_rad_s) {
  *this = {};
  if (!std::isfinite(gyro_bias_rad_s)) {
    invalidate(AttitudeInvalidReason::numeric_error);
    return false;
  }
  bias_rad_s_ = gyro_bias_rad_s;

  const GyroSample *seed = nullptr;
  for (std::size_t index = 0; index < history.size(); ++index) {
    const auto &sample = history.at(index);
    if (sample.timestamp_us <= liftoff_time_us && sample.valid &&
        !sample.saturated && !sample.format_fault)
      seed = &sample;
  }
  if (seed == nullptr) {
    invalidate(AttitudeInvalidReason::history_unavailable);
    return false;
  }

  state_ = {};
  state_.valid = true;
  state_.invalid_reason = AttitudeInvalidReason::none;
  state_.timestamp_us = liftoff_time_us;
  previous_timestamp_us_ = liftoff_time_us;
  previous_rate_rad_s_ = seed->roll_rate_rad_s - bias_rad_s_;
  timestamp_epoch_ = seed->timestamp_epoch;
  seeded_ = true;
  for (std::size_t index = 0; index < history.size(); ++index) {
    const auto &sample = history.at(index);
    if (sample.timestamp_us > liftoff_time_us && !accept(sample))
      return false;
  }
  return state_.valid;
}

bool AttitudeEstimator::update(const GyroSample &sample) {
  if (!state_.valid || !seeded_)
    return false;
  return accept(sample);
}

void AttitudeEstimator::invalidateForReset() {
  invalidate(AttitudeInvalidReason::reset_invalidated);
}

bool AttitudeEstimator::accept(const GyroSample &sample) {
  if (sample.fifo_full)
    ++state_.fifo_full_event_count;
  if (sample.format_fault) {
    invalidate(AttitudeInvalidReason::fifo_format_fault);
    return false;
  }
  if (!sample.valid || !std::isfinite(sample.roll_rate_rad_s)) {
    invalidate(AttitudeInvalidReason::sample_invalid);
    return false;
  }
  if (sample.saturated) {
    invalidate(AttitudeInvalidReason::saturated);
    return false;
  }
  if (sample.timestamp_epoch != timestamp_epoch_) {
    invalidate(AttitudeInvalidReason::timestamp_epoch_changed);
    return false;
  }
  if (sample.timestamp_us <= previous_timestamp_us_) {
    invalidate(AttitudeInvalidReason::timestamp_invalid);
    return false;
  }

  const uint64_t delta_us = sample.timestamp_us - previous_timestamp_us_;
  const double rate = sample.roll_rate_rad_s - bias_rad_s_;
  double increment{};
  if (nearPeriod(delta_us, 1) && sample.lost_packets == 0) {
    increment = 0.5 * (previous_rate_rad_s_ + rate) *
                static_cast<double>(delta_us) * 1.0e-6;
  } else if (nearPeriod(delta_us, 2) && sample.lost_packets <= 1) {
    // TODO(SIMULATION): 1 sample欠落許容を実機noise込みで再評価する。
    increment = (previous_rate_rad_s_ + rate) * 0.001;
    ++state_.interpolated_sample_count;
    ++state_.data_loss_event_count;
  } else {
    if (sample.lost_packets != 0)
      ++state_.data_loss_event_count;
    invalidate(sample.lost_packets > 1
                   ? AttitudeInvalidReason::excess_data_loss
                   : AttitudeInvalidReason::timestamp_invalid);
    return false;
  }
  if (!std::isfinite(increment)) {
    invalidate(AttitudeInvalidReason::numeric_error);
    return false;
  }
  state_.roll_rad += increment;
  state_.roll_rate_rad_s = rate;
  state_.timestamp_us = sample.timestamp_us;
  previous_timestamp_us_ = sample.timestamp_us;
  previous_rate_rad_s_ = rate;
  return true;
}

void AttitudeEstimator::invalidate(AttitudeInvalidReason reason) {
  state_.valid = false;
  state_.invalid_reason = reason;
}

} // 名前空間 sensors
