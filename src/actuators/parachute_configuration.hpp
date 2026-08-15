#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace actuators {

constexpr int kParachuteCountsPerRevolution = 4096;
constexpr int kParachuteHalfRevolutionCounts =
    kParachuteCountsPerRevolution / 2;
constexpr double kParachuteDegreesPerCount =
    360.0 / static_cast<double>(kParachuteCountsPerRevolution);

class AbsoluteParachuteAngle {
public:
  [[nodiscard]] static constexpr std::optional<AbsoluteParachuteAngle>
  fromCount(uint16_t count) {
    if (count >= kParachuteCountsPerRevolution)
      return std::nullopt;
    return AbsoluteParachuteAngle{count};
  }

  [[nodiscard]] constexpr uint16_t count() const { return count_; }

  friend constexpr bool operator==(AbsoluteParachuteAngle left,
                                   AbsoluteParachuteAngle right) {
    return left.count_ == right.count_;
  }

  friend constexpr bool operator!=(AbsoluteParachuteAngle left,
                                   AbsoluteParachuteAngle right) {
    return !(left == right);
  }

private:
  explicit constexpr AbsoluteParachuteAngle(uint16_t count) : count_(count) {}

  uint16_t count_{};
};

enum class ParachutePathError : uint8_t {
  none,
  exactly_half_turn,
};

struct SignedParachuteDisplacement {
  int16_t counts{};
  ParachutePathError error{ParachutePathError::none};

  [[nodiscard]] constexpr bool valid() const {
    return error == ParachutePathError::none;
  }

  [[nodiscard]] constexpr double degrees() const {
    return static_cast<double>(counts) * kParachuteDegreesPerCount;
  }
};

// 返値は -2048 < counts < 2048。ちょうど半回転は方向を選ばず拒否する。
[[nodiscard]] constexpr SignedParachuteDisplacement
shortestParachuteDisplacement(AbsoluteParachuteAngle current,
                              AbsoluteParachuteAngle target) {
  int delta = static_cast<int>(target.count()) -
              static_cast<int>(current.count());
  if (delta < 0)
    delta += kParachuteCountsPerRevolution;
  if (delta == kParachuteHalfRevolutionCounts)
    return {0, ParachutePathError::exactly_half_turn};
  if (delta > kParachuteHalfRevolutionCounts)
    delta -= kParachuteCountsPerRevolution;
  return {static_cast<int16_t>(delta), ParachutePathError::none};
}

enum class ParachuteEndpoint : uint8_t {
  open = 1,
  close = 2,
};

struct ParachuteConfiguration {
  std::optional<AbsoluteParachuteAngle> open{};
  std::optional<AbsoluteParachuteAngle> close{};

  [[nodiscard]] constexpr bool openConfigured() const {
    return open.has_value();
  }

  [[nodiscard]] constexpr bool closeConfigured() const {
    return close.has_value();
  }
};

struct FlightParachuteConfiguration {
  std::optional<AbsoluteParachuteAngle> open{};
  std::optional<AbsoluteParachuteAngle> close{};
};

enum class FlightParachutePreparationError : uint8_t {
  none,
  open_not_configured,
  close_not_configured,
  current_open_exactly_half_turn,
};

struct FlightParachutePreparationResult {
  FlightParachutePreparationError error{
      FlightParachutePreparationError::none};

  [[nodiscard]] constexpr bool ready() const {
    return error == FlightParachutePreparationError::none;
  }
};

// CommandReceive用設定と飛行用snapshotを同じowner内で管理する。
class ParachuteConfigurationState {
public:
  [[nodiscard]] const ParachuteConfiguration &active() const {
    return active_;
  }

  [[nodiscard]] ParachuteConfiguration
  candidateWith(ParachuteEndpoint endpoint,
                AbsoluteParachuteAngle angle) const {
    ParachuteConfiguration candidate = active_;
    if (endpoint == ParachuteEndpoint::open)
      candidate.open = angle;
    else
      candidate.close = angle;
    return candidate;
  }

  // NVS commitとreadbackが成功した後だけ呼ぶ。
  void activatePersistedCandidate(const ParachuteConfiguration &candidate) {
    active_ = candidate;
  }

  // 起動時load済みの有効endpointだけを渡す。破損endpointはnulloptとする。
  void replaceLoadedConfiguration(const ParachuteConfiguration &loaded) {
    active_ = loaded;
  }

  // RTC checkpointから復元した飛行用snapshotだけを戻す。
  void restoreFlightSnapshot(const FlightParachuteConfiguration &snapshot) {
    flight_snapshot_ = snapshot;
    flight_snapshot_valid_ = true;
  }

  [[nodiscard]] FlightParachutePreparationResult
  freezeNormalFlightSnapshot(AbsoluteParachuteAngle current) {
    if (!active_.open.has_value())
      return {FlightParachutePreparationError::open_not_configured};
    if (!active_.close.has_value())
      return {FlightParachutePreparationError::close_not_configured};
    if (!shortestParachuteDisplacement(current, *active_.open).valid())
      return {FlightParachutePreparationError::current_open_exactly_half_turn};

    // Open/Close相互のhalf-turnはStart拒否理由にしない。
    flight_snapshot_ = {active_.open, active_.close};
    flight_snapshot_valid_ = true;
    return {};
  }

  void freezeForcedFlightSnapshot() {
    // Forceでもmissing/corrupt endpointを生成・補正しない。
    flight_snapshot_ = {active_.open, active_.close};
    flight_snapshot_valid_ = true;
  }

  void discardFlightSnapshot() {
    flight_snapshot_ = {};
    flight_snapshot_valid_ = false;
  }

  [[nodiscard]] bool flightSnapshotValid() const {
    return flight_snapshot_valid_;
  }

  [[nodiscard]] const FlightParachuteConfiguration *flightSnapshot() const {
    return flight_snapshot_valid_ ? &flight_snapshot_ : nullptr;
  }

private:
  ParachuteConfiguration active_{};
  FlightParachuteConfiguration flight_snapshot_{};
  bool flight_snapshot_valid_{};
};

constexpr std::size_t kParachuteEndpointBlobSize = 16;
using ParachuteEndpointBlob =
    std::array<uint8_t, kParachuteEndpointBlobSize>;

enum class ParachuteBlobError : uint8_t {
  none,
  wrong_size,
  crc_mismatch,
  wrong_magic,
  wrong_schema,
  wrong_endpoint,
  wrong_payload_size,
  reserved_nonzero,
  angle_out_of_range,
};

struct DecodedParachuteEndpoint {
  std::optional<AbsoluteParachuteAngle> angle{};
  ParachuteBlobError error{ParachuteBlobError::none};

  [[nodiscard]] bool valid() const {
    return error == ParachuteBlobError::none && angle.has_value();
  }
};

[[nodiscard]] inline uint32_t parachuteCrc32(const uint8_t *data,
                                             std::size_t size) {
  uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U : crc >> 1U;
  }
  return ~crc;
}

inline void writeLe16(uint8_t *destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

inline void writeLe32(uint8_t *destination, uint32_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  destination[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  destination[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

[[nodiscard]] inline uint16_t readLe16(const uint8_t *source) {
  return static_cast<uint16_t>(source[0]) |
         static_cast<uint16_t>(source[1]) << 8U;
}

[[nodiscard]] inline uint32_t readLe32(const uint8_t *source) {
  return static_cast<uint32_t>(source[0]) |
         static_cast<uint32_t>(source[1]) << 8U |
         static_cast<uint32_t>(source[2]) << 16U |
         static_cast<uint32_t>(source[3]) << 24U;
}

// 永続形式v1。末尾4 byteは先頭12 byteのCRC32とする。
[[nodiscard]] inline ParachuteEndpointBlob
encodeParachuteEndpoint(ParachuteEndpoint endpoint,
                        AbsoluteParachuteAngle angle) {
  ParachuteEndpointBlob blob{};
  blob[0] = '9';
  blob[1] = '9';
  blob[2] = 'L';
  blob[3] = 'P';
  blob[4] = 1;
  blob[5] = static_cast<uint8_t>(endpoint);
  writeLe16(blob.data() + 6, 2);
  writeLe16(blob.data() + 8, angle.count());
  writeLe16(blob.data() + 10, 0);
  writeLe32(blob.data() + 12, parachuteCrc32(blob.data(), 12));
  return blob;
}

[[nodiscard]] inline DecodedParachuteEndpoint
decodeParachuteEndpoint(const uint8_t *data, std::size_t size,
                        ParachuteEndpoint expected_endpoint) {
  if (data == nullptr || size != kParachuteEndpointBlobSize)
    return {std::nullopt, ParachuteBlobError::wrong_size};
  if (readLe32(data + 12) != parachuteCrc32(data, 12))
    return {std::nullopt, ParachuteBlobError::crc_mismatch};
  if (data[0] != '9' || data[1] != '9' || data[2] != 'L' ||
      data[3] != 'P')
    return {std::nullopt, ParachuteBlobError::wrong_magic};
  if (data[4] != 1)
    return {std::nullopt, ParachuteBlobError::wrong_schema};
  if (data[5] != static_cast<uint8_t>(expected_endpoint))
    return {std::nullopt, ParachuteBlobError::wrong_endpoint};
  if (readLe16(data + 6) != 2)
    return {std::nullopt, ParachuteBlobError::wrong_payload_size};
  if (readLe16(data + 10) != 0)
    return {std::nullopt, ParachuteBlobError::reserved_nonzero};

  const auto angle = AbsoluteParachuteAngle::fromCount(readLe16(data + 8));
  if (!angle.has_value())
    return {std::nullopt, ParachuteBlobError::angle_out_of_range};
  return {angle, ParachuteBlobError::none};
}

} // 名前空間 actuators
