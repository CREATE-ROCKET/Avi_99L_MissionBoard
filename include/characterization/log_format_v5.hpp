#pragma once

#include "characterization/characterization_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace avi::characterization::wire_v5 {

constexpr std::size_t kHeaderBytes = 256U;
constexpr std::size_t kRecordBytes = 320U;
constexpr std::size_t kFooterBytes = 192U;
constexpr std::array<std::uint8_t, 8> kHeaderMagic{
    '9', '9', 'L', 'M', 'C', 'V', '5', '\0'};
constexpr std::array<std::uint8_t, 4> kRecordMagic{'E', 'P', 'V', '5'};
constexpr std::array<std::uint8_t, 8> kFooterMagic{
    '9', '9', 'L', 'E', 'N', 'D', '5', '\0'};

using HeaderBytes = std::array<std::uint8_t, kHeaderBytes>;
using RecordBytes = std::array<std::uint8_t, kRecordBytes>;
using FooterBytes = std::array<std::uint8_t, kFooterBytes>;

enum class DecodeError : std::uint8_t {
  None = 0,
  Truncated,
  TrailingBytes,
  Magic,
  Schema,
  Size,
  Crc,
  Enum,
  Reserved,
  Sequence,
  Invariant,
};

struct CaptureSummary {
  DecodeError error{DecodeError::None};
  std::uint64_t records{0};
  std::uint64_t first_sequence{0};
  std::uint64_t last_sequence{0};
};

[[nodiscard]] std::uint32_t
crc32(const std::uint8_t *data, std::size_t size,
      std::uint32_t previous_crc = 0U) noexcept;

[[nodiscard]] bool encodeHeader(const LogHeaderV5 &header,
                                HeaderBytes &bytes) noexcept;
[[nodiscard]] bool encodeRecord(const ImmutableLogRecord &record,
                                RecordBytes &bytes) noexcept;
[[nodiscard]] bool encodeFooter(const LogFooterV5 &footer,
                                FooterBytes &bytes) noexcept;

[[nodiscard]] DecodeError decodeHeader(const std::uint8_t *data,
                                       std::size_t size,
                                       LogHeaderV5 &header) noexcept;
[[nodiscard]] DecodeError decodeRecord(const std::uint8_t *data,
                                       std::size_t size,
                                       ImmutableLogRecord &record) noexcept;
[[nodiscard]] DecodeError decodeFooter(const std::uint8_t *data,
                                       std::size_t size,
                                       LogFooterV5 &footer) noexcept;
[[nodiscard]] CaptureSummary validateCapture(const std::uint8_t *data,
                                             std::size_t size) noexcept;

static_assert(kHeaderBytes == 256U);
static_assert(kRecordBytes == 320U);
static_assert(kFooterBytes == 192U);

} // 名前空間 avi::characterization::wire_v5
