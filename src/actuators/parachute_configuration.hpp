#pragma once

#include <cstdint>
#include <optional>

namespace actuators {

constexpr int kParachuteCountsPerRevolution = 4096;
constexpr int kParachuteHalfRevolutionCounts =
    kParachuteCountsPerRevolution / 2;
constexpr double kParachuteDegreesPerCount =
    360.0 / static_cast<double>(kParachuteCountsPerRevolution);

// STS3215 step modeでは正方向を時計回りとして扱う。
// TODO(HW_TEST): 実機取付状態で符号を最終確認する。
constexpr float kParachuteOpenRelativeDegrees = -130.0F;
constexpr float kParachuteCloseRelativeDegrees = 130.0F;

// STS3215 position feedbackのbit15=sign、bit0..14=magnitudeを
// 連続したsigned countへ変換する。
[[nodiscard]] constexpr int32_t
decodeStsSignedMagnitudePositionCount(uint16_t raw_count) {
  const int32_t magnitude = static_cast<int32_t>(raw_count & 0x7FFFU);
  return (raw_count & 0x8000U) != 0U ? -magnitude : magnitude;
}

class AbsoluteParachuteAngle {
public:
  [[nodiscard]] static constexpr std::optional<AbsoluteParachuteAngle>
  fromCanonicalCount(uint16_t count) {
    if (count >= kParachuteCountsPerRevolution)
      return std::nullopt;
    return AbsoluteParachuteAngle{count};
  }

  [[nodiscard]] static constexpr std::optional<AbsoluteParachuteAngle>
  fromCount(uint16_t raw_count) {
    const int32_t signed_count =
        decodeStsSignedMagnitudePositionCount(raw_count);
    int32_t wrapped = signed_count % kParachuteCountsPerRevolution;
    if (wrapped < 0)
      wrapped += kParachuteCountsPerRevolution;
    return fromCanonicalCount(static_cast<uint16_t>(wrapped));
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

enum class ParachutePathError : uint8_t { none, exactly_half_turn };

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

// telemetry/test用のwrap helper。Open/Close制御そのものはこの絶対角計算を使わず、
// STS3215のrelative moveを1回だけ発行する。
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

} // 名前空間 actuators