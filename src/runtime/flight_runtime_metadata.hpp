#pragma once

#include <cstdint>
#include <limits>

namespace runtime::flight_runtime_metadata {

enum class FinZeroApproachDirection : uint8_t {
  unknown = 0,
  positive = 1,
  negative = 2,
};

enum class FinZeroCalibrationMethod : uint8_t {
  unknown = 0,
  current_position = 1,
  positive_single_approach = 2,
  negative_single_approach = 3,
  bidirectional_midpoint = 4,
};

enum class FinZeroGroundVerificationStatus : uint8_t {
  unverified = 0,
  passed = 1,
  failed = 2,
};

enum class EncoderPipelineState : uint8_t {
  uninitialized = 0,
  warming_up = 1,
  ready = 2,
  faulted = 3,
};

struct FinZeroMetadata {
  uint16_t encoder_zero_count{0xFFFFU};
  uint64_t configured_timestamp_us{};
  FinZeroApproachDirection approach_direction{FinZeroApproachDirection::unknown};
  FinZeroCalibrationMethod calibration_method{FinZeroCalibrationMethod::unknown};
  FinZeroGroundVerificationStatus ground_verification_status{
      FinZeroGroundVerificationStatus::unverified};
  float measured_bidirectional_span_rad{
      std::numeric_limits<float>::quiet_NaN()};
};

struct EncoderTimingMetadata {
  uint64_t capture_requested_timestamp_us{};
  uint64_t spi_transaction_start_us{};
  uint64_t spi_transaction_complete_us{};
  uint64_t consumer_timestamp_us{};
  uint64_t pipeline_ready_timestamp_us{};
  uint16_t consumer_deadline_miss_count{};
  uint16_t repeated_block_count{};
  uint16_t coalesced_notification_count{};
  uint16_t raw_capture_missed_tick_count{};
  uint16_t timestamp_offset_saturation_count{};
  EncoderPipelineState pipeline_state{EncoderPipelineState::uninitialized};
};

struct Snapshot {
  FinZeroMetadata fin_zero{};
  EncoderTimingMetadata encoder{};
};

void publishFinZero(const FinZeroMetadata &metadata);
void publishEncoderTiming(const EncoderTimingMetadata &metadata);
[[nodiscard]] Snapshot snapshot();

} // namespace runtime::flight_runtime_metadata
