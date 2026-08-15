#pragma once

#include "characterization/characterization_types.hpp"

#include <cstdint>

namespace avi::characterization {

enum class RecordValidationError : std::uint32_t {
  None = 0,
  InvalidStage = 1U << 0U,
  InvalidEncoderRate = 1U << 1U,
  InvalidEnum = 1U << 2U,
  RequestedCommandModeMismatch = 1U << 3U,
  AppliedCommandModeMismatch = 1U << 4U,
  InvalidCommandTimestamp = 1U << 5U,
  InvalidEpochTimestamp = 1U << 6U,
  WrongExpectedSampleCount = 1U << 7U,
  InconsistentCounts = 1U << 8U,
  InconsistentFlags = 1U << 9U,
  SampleOutsideEpoch = 1U << 10U,
  InvalidRawDiagnostic = 1U << 11U,
  InvalidGeneration = 1U << 12U,
  InvalidVbus = 1U << 13U,
};

constexpr RecordValidationError operator|(RecordValidationError left,
                                          RecordValidationError right)
    noexcept {
  return static_cast<RecordValidationError>(
      static_cast<std::uint32_t>(left) |
      static_cast<std::uint32_t>(right));
}

constexpr bool hasError(RecordValidationError error) noexcept {
  return error != RecordValidationError::None;
}

// characterization実機buildではrealtime callerのvalidateRecord()をdeferし、
// char_writerがencode直前にvalidateRecordStrict()を必ず実行する。
// native/offline buildではvalidateRecord()も従来どおりstrict validationを行う。
[[nodiscard]] RecordValidationError
validateRecord(const ImmutableLogRecord &record) noexcept;
[[nodiscard]] RecordValidationError
validateRecordStrict(const ImmutableLogRecord &record) noexcept;

} // 名前空間 avi::characterization
