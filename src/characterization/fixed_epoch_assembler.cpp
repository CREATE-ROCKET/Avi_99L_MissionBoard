#include "characterization/fixed_epoch_assembler.hpp"

#include <algorithm>
#include <limits>

namespace avi::characterization {
namespace {

bool checkedAdd(std::int64_t lhs, std::int64_t rhs,
                std::int64_t &sum) noexcept {
  if ((rhs > 0 && lhs > std::numeric_limits<std::int64_t>::max() - rhs) ||
      (rhs < 0 && lhs < std::numeric_limits<std::int64_t>::min() - rhs))
    return false;
  sum = lhs + rhs;
  return true;
}

bool checkedQ16(std::int64_t counts, std::int64_t &q16) noexcept {
  constexpr std::int64_t kScale = 65'536;
  if (counts > std::numeric_limits<std::int64_t>::max() / kScale ||
      counts < std::numeric_limits<std::int64_t>::min() / kScale)
    return false;
  q16 = counts * kScale;
  return true;
}

} // 無名名前空間

FixedEpochAssembler::FixedEpochAssembler(
    EncoderRate rate, std::uint32_t deadline_budget_us) noexcept
    : rate_(rate), samples_per_epoch_(expectedSamplesPerEpoch(rate)),
      deadline_budget_us_(deadline_budget_us) {}

void FixedEpochAssembler::reset(
    std::uint64_t epoch_zero_timestamp_us) noexcept {
  epoch_zero_timestamp_us_ = epoch_zero_timestamp_us;
  initialized_ = true;
  has_released_epoch_ = false;
  last_released_epoch_ = 0;
  has_unwrapped_reference_ = false;
  last_released_raw_ = 0;
  last_unwrapped_count_ = 0;
  unreported_late_samples_ = 0U;
  buckets_ = {};
  statistics_ = {};
}

void FixedEpochAssembler::reset(std::uint64_t epoch_zero_timestamp_us,
                                std::uint16_t initial_raw) noexcept {
  reset(epoch_zero_timestamp_us);
  last_released_raw_ = initial_raw;
  last_unwrapped_count_ = last_released_raw_;
  has_unwrapped_reference_ = true;
}

void FixedEpochAssembler::reset(
    EncoderRate rate, std::uint64_t epoch_zero_timestamp_us) noexcept {
  rate_ = rate;
  samples_per_epoch_ = expectedSamplesPerEpoch(rate);
  reset(epoch_zero_timestamp_us);
}

std::uint8_t FixedEpochAssembler::slotForCaptureOffset(
    std::uint64_t offset_us) const noexcept {
  const std::uint64_t scaled =
      offset_us * static_cast<std::uint64_t>(samples_per_epoch_);
  const std::uint64_t slot = scaled / kEpochDurationUs;
  return static_cast<std::uint8_t>(
      std::min<std::uint64_t>(slot, samples_per_epoch_ - 1U));
}

std::uint64_t
FixedEpochAssembler::slotCenterOffsetUs(std::uint8_t slot) const noexcept {
  const std::uint64_t numerator =
      (2U * static_cast<std::uint64_t>(slot) + 1U) * kEpochDurationUs;
  const std::uint64_t denominator =
      2U * static_cast<std::uint64_t>(samples_per_epoch_);
  return numerator / denominator;
}

std::uint64_t FixedEpochAssembler::absoluteDifference(std::uint64_t lhs,
                                                      std::uint64_t rhs)
    noexcept {
  return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

std::int32_t FixedEpochAssembler::wrappedDelta(std::uint16_t current,
                                               std::uint16_t previous)
    noexcept {
  std::int32_t delta = static_cast<std::int32_t>(current) -
                       static_cast<std::int32_t>(previous);
  if (delta > 8192)
    delta -= 16384;
  else if (delta < -8192)
    delta += 16384;
  return delta;
}

AddSampleResult FixedEpochAssembler::add(
    const EncoderRawSample &sample) noexcept {
  if (!initialized_ || samples_per_epoch_ == 0U ||
      sample.capture_timestamp_us < epoch_zero_timestamp_us_) {
    ++statistics_.pre_epoch_samples;
    return AddSampleResult::pre_epoch;
  }

  const std::uint64_t relative =
      sample.capture_timestamp_us - epoch_zero_timestamp_us_;
  const std::uint64_t epoch_index = relative / kEpochDurationUs;
  if (has_released_epoch_ && epoch_index <= last_released_epoch_) {
    ++statistics_.late_samples;
    ++unreported_late_samples_;
    return AddSampleResult::already_released;
  }

  Bucket &bucket = buckets_[epoch_index % kBucketCount];
  if (bucket.initialized && bucket.epoch_index != epoch_index) {
    ++statistics_.bucket_collisions;
    return AddSampleResult::bucket_collision;
  }
  if (!bucket.initialized) {
    bucket = {};
    bucket.initialized = true;
    bucket.epoch_index = epoch_index;
  }

  const std::uint64_t offset_us = relative % kEpochDurationUs;
  const std::uint8_t slot = slotForCaptureOffset(offset_us);
  ++bucket.observed_count;

  if (!bucket.present[slot]) {
    bucket.samples[slot] = sample;
    bucket.present[slot] = true;
    return AddSampleResult::accepted;
  }

  ++bucket.repeated_count;
  const std::uint64_t center = slotCenterOffsetUs(slot);
  const std::uint64_t current_offset =
      bucket.samples[slot].capture_timestamp_us -
      (epoch_zero_timestamp_us_ + epoch_index * kEpochDurationUs);
  const std::uint64_t old_distance = absoluteDifference(current_offset, center);
  const std::uint64_t new_distance = absoluteDifference(offset_us, center);
  if (new_distance < old_distance ||
      (new_distance == old_distance &&
       sample.capture_timestamp_us <
           bucket.samples[slot].capture_timestamp_us)) {
    bucket.samples[slot] = sample;
  }
  return AddSampleResult::accepted;
}

bool FixedEpochAssembler::release(std::uint64_t epoch_index,
                                  std::uint64_t release_timestamp_us,
                                  EncoderEpochBlock &block) noexcept {
  if (!initialized_ || samples_per_epoch_ == 0U)
    return false;
  if ((!has_released_epoch_ && epoch_index != 0U) ||
      (has_released_epoch_ && epoch_index != last_released_epoch_ + 1U))
    return false;

  const std::uint64_t remaining_time =
      std::numeric_limits<std::uint64_t>::max() - epoch_zero_timestamp_us_;
  if (epoch_index >= remaining_time / kEpochDurationUs)
    return false;
  const std::uint64_t expected_epoch_end =
      epoch_zero_timestamp_us_ + (epoch_index + 1U) * kEpochDurationUs;
  // 境界前にepochをsealすると、同じepochへ属する遅延sampleを捨てるため拒否する。
  if (release_timestamp_us < expected_epoch_end)
    return false;

  block = {};
  block.epoch_index = epoch_index;
  block.epoch_start_timestamp_us =
      epoch_zero_timestamp_us_ + epoch_index * kEpochDurationUs;
  block.epoch_end_timestamp_us =
      block.epoch_start_timestamp_us + kEpochDurationUs;
  block.release_timestamp_us = release_timestamp_us;
  block.expected_sample_count = samples_per_epoch_;
  if (unreported_late_samples_ != 0U) {
    block.flags = static_cast<std::uint16_t>(block.flags | EpochLate);
    unreported_late_samples_ = 0U;
  }

  const std::int64_t signed_lateness =
      release_timestamp_us >= block.epoch_end_timestamp_us
          ? static_cast<std::int64_t>(release_timestamp_us -
                                      block.epoch_end_timestamp_us)
          : -static_cast<std::int64_t>(block.epoch_end_timestamp_us -
                                       release_timestamp_us);
  block.consumer_lateness_us = static_cast<std::int32_t>(std::clamp<
      std::int64_t>(signed_lateness, std::numeric_limits<std::int32_t>::min(),
                   std::numeric_limits<std::int32_t>::max()));

  Bucket &bucket = buckets_[epoch_index % kBucketCount];
  if (bucket.initialized && bucket.epoch_index == epoch_index) {
    block.observed_sample_count = bucket.observed_count;
    block.actual_sample_count = bucket.observed_count;
    block.repeated_sample_count = bucket.repeated_count;
    for (std::uint8_t slot = 0; slot < samples_per_epoch_; ++slot) {
      if (!bucket.present[slot]) {
        ++block.skipped_sample_count;
        continue;
      }
      block.sample_present[slot] = true;
      block.samples[slot] = bucket.samples[slot];
      block.samples[slot].slot = slot;
      ++block.selected_sample_count;
      if (bucket.samples[slot].valid &&
          bucket.samples[slot].read_result_code == 0) {
        ++block.valid_sample_count;
      } else {
        ++block.invalid_sample_count;
      }
    }
    bucket = {};
  } else {
    block.skipped_sample_count = samples_per_epoch_;
  }

  if (block.repeated_sample_count != 0U)
    block.flags = static_cast<std::uint16_t>(block.flags |
                                            epoch_repeated_sample);
  if (block.skipped_sample_count != 0U)
    block.flags =
        static_cast<std::uint16_t>(block.flags | epoch_skipped_sample);
  if (block.invalid_sample_count != 0U)
    block.flags =
        static_cast<std::uint16_t>(block.flags | epoch_invalid_sample);

  const bool complete = block.observed_sample_count == samples_per_epoch_ &&
                        block.selected_sample_count == samples_per_epoch_ &&
                        block.valid_sample_count == samples_per_epoch_ &&
                        block.repeated_sample_count == 0U;
  if (!complete)
    block.flags = static_cast<std::uint16_t>(block.flags | epoch_incomplete);
  if (!complete && epoch_index == 0U)
    block.flags = static_cast<std::uint16_t>(block.flags | epoch_startup);

  if (release_timestamp_us >
      block.epoch_end_timestamp_us + deadline_budget_us_) {
    block.flags = static_cast<std::uint16_t>(
        block.flags | epoch_consumer_deadline_miss);
    ++statistics_.consumer_deadline_misses;
  }

  std::int64_t sum_q16 = 0;
  std::uint8_t aggregate_count = 0;
  for (std::uint8_t slot = 0; slot < samples_per_epoch_; ++slot) {
    if (!block.sample_present[slot] || !block.samples[slot].valid ||
        block.samples[slot].read_result_code != 0)
      continue;
    const std::uint16_t raw = block.samples[slot].angle_raw;
    if (!has_unwrapped_reference_) {
      last_unwrapped_count_ = static_cast<std::int64_t>(raw);
      has_unwrapped_reference_ = true;
    } else {
      std::int64_t next_unwrapped = 0;
      if (!checkedAdd(last_unwrapped_count_,
                      wrappedDelta(raw, last_released_raw_),
                      next_unwrapped))
        return false;
      last_unwrapped_count_ = next_unwrapped;
    }
    last_released_raw_ = raw;
    std::int64_t sample_q16 = 0;
    std::int64_t next_sum = 0;
    if (!checkedQ16(last_unwrapped_count_, sample_q16) ||
        !checkedAdd(sum_q16, sample_q16, next_sum))
      return false;
    sum_q16 = next_sum;
    ++aggregate_count;
  }
  if (aggregate_count != 0U) {
    block.mean_unwrapped_counts_q16 =
        sum_q16 / static_cast<std::int64_t>(aggregate_count);
  }
  if (complete)
    block.flags =
        static_cast<std::uint16_t>(block.flags | epoch_aggregate_valid);

  ++statistics_.released_epochs;
  statistics_.repeated_samples += block.repeated_sample_count;
  statistics_.skipped_samples += block.skipped_sample_count;
  statistics_.invalid_samples += block.invalid_sample_count;
  if (!complete)
    ++statistics_.incomplete_epochs;
  if (!complete && epoch_index == 0U)
    ++statistics_.startup_incomplete_epochs;
  else if (!complete)
    ++statistics_.steady_state_incomplete_epochs;

  has_released_epoch_ = true;
  last_released_epoch_ = epoch_index;
  return true;
}

EncoderEpochBlock
FixedEpochAssembler::release(std::uint64_t epoch_index,
                             std::uint64_t release_timestamp_us) noexcept {
  EncoderEpochBlock block{};
  if (!release(epoch_index, release_timestamp_us, block))
    block.flags = EpochIncomplete;
  return block;
}

} // 名前空間 avi::characterization
