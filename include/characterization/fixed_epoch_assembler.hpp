#pragma once

#include "characterization/characterization_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace avi::characterization {

class FixedEpochAssembler {
public:
  FixedEpochAssembler() noexcept = default;
  explicit FixedEpochAssembler(EncoderRate rate,
                               std::uint32_t deadline_budget_us =
                                   kConsumerDeadlineBudgetUs) noexcept;

  void reset(std::uint64_t epoch_zero_timestamp_us) noexcept;
  void reset(std::uint64_t epoch_zero_timestamp_us,
             std::uint16_t initial_raw) noexcept;
  void reset(EncoderRate rate,
             std::uint64_t epoch_zero_timestamp_us) noexcept;

  [[nodiscard]] AddSampleResult add(const EncoderRawSample &sample) noexcept;

  [[nodiscard]] bool release(std::uint64_t epoch_index,
                             std::uint64_t release_timestamp_us,
                             EncoderEpochBlock &block) noexcept;
  [[nodiscard]] EncoderEpochBlock
  release(std::uint64_t epoch_index,
          std::uint64_t release_timestamp_us) noexcept;

  [[nodiscard]] EncoderRate rate() const noexcept { return rate_; }
  [[nodiscard]] std::uint64_t epochZeroTimestampUs() const noexcept {
    return epoch_zero_timestamp_us_;
  }
  [[nodiscard]] const EpochStatistics &statistics() const noexcept {
    return statistics_;
  }

private:
  static constexpr std::size_t kBucketCount = 8U;

  struct Bucket {
    bool initialized{false};
    std::uint64_t epoch_index{0};
    std::uint8_t observed_count{0};
    std::uint8_t repeated_count{0};
    std::array<EncoderRawSample, kMaximumEncoderSamplesPerEpoch> samples{};
    std::array<bool, kMaximumEncoderSamplesPerEpoch> present{};
  };

  [[nodiscard]] std::uint8_t slotForCaptureOffset(std::uint64_t offset_us) const
      noexcept;
  [[nodiscard]] std::uint64_t slotCenterOffsetUs(std::uint8_t slot) const
      noexcept;
  [[nodiscard]] static std::uint64_t absoluteDifference(std::uint64_t lhs,
                                                        std::uint64_t rhs)
      noexcept;
  [[nodiscard]] static std::int32_t wrappedDelta(std::uint16_t current,
                                                 std::uint16_t previous)
      noexcept;

  EncoderRate rate_{EncoderRate::hz_1000};
  std::uint8_t samples_per_epoch_{1};
  std::uint32_t deadline_budget_us_{100};
  std::uint64_t epoch_zero_timestamp_us_{0};
  bool initialized_{false};
  bool has_released_epoch_{false};
  std::uint64_t last_released_epoch_{0};
  bool has_unwrapped_reference_{false};
  std::uint16_t last_released_raw_{0};
  std::int64_t last_unwrapped_count_{0};
  std::uint64_t unreported_late_samples_{0};
  std::array<Bucket, kBucketCount> buckets_{};
  EpochStatistics statistics_{};
};

} // 名前空間 avi::characterization
