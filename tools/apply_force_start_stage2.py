from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def write(path: str, content: str) -> None:
    (ROOT / path).write_text(content, encoding="utf-8")


def replace_once(path: str, old: str, new: str) -> None:
    p = ROOT / path
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old[:120]!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


write("src/actuators/parachute_configuration.hpp", r'''#pragma once

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

// freshな現在角からtargetへの最短変位を返す。exact half-turnでは方向を選ばない。
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

enum class ParachuteEndpoint : uint8_t { open = 1, close = 2 };

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

// 飛行中はactive NVS設定を再参照せず、このoptional snapshotだけを使用する。
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

class ParachuteConfigurationState {
public:
  [[nodiscard]] const ParachuteConfiguration &active() const { return active_; }

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

  // key missing/corruptはnulloptへ変換済みの値だけを受け取る。
  void replaceLoadedConfiguration(const ParachuteConfiguration &loaded) {
    active_ = loaded;
  }

  // RTC checkpointはOpen/Closeを独立optionalのまま復元する。
  void restoreFlightSnapshot(const FlightParachuteConfiguration &snapshot) {
    flight_snapshot_ = snapshot;
    flight_snapshot_valid_ = true;
  }

  // 通常Startは両endpointを要求し、fresh current->Openだけを事前検証する。
  [[nodiscard]] FlightParachutePreparationResult
  freezeFlightSnapshot(AbsoluteParachuteAngle current) {
    if (!active_.open.has_value())
      return {FlightParachutePreparationError::open_not_configured};
    if (!active_.close.has_value())
      return {FlightParachutePreparationError::close_not_configured};
    if (!shortestParachuteDisplacement(current, *active_.open).valid())
      return {FlightParachutePreparationError::current_open_exactly_half_turn};
    flight_snapshot_ = {active_.open, active_.close};
    flight_snapshot_valid_ = true;
    return {};
  }

  // ForceStartではendpointを生成せず、独立optionalをそのままfreezeする。
  void freezeFlightSnapshotForced() {
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
  if (data[0] != '9' || data[1] != '9' || data[2] != 'L' || data[3] != 'P')
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
''')

write("src/actuators/safety_core.hpp", r'''#pragma once

#include <cstdint>

#include "actuators/parachute_configuration.hpp"

namespace actuators {

struct PowerState {
  bool auxiliary_5v{};
  bool parachute_power{};
  bool cutoff_latched{};
};

class PowerArbiter {
public:
  [[nodiscard]] bool requestAuxiliary5v(bool enabled);
  [[nodiscard]] bool requestParachutePower(bool enabled);
  void latchDeploymentCutoff();
  [[nodiscard]] const PowerState &state() const { return state_; }

private:
  PowerState state_{};
};

enum class ParachuteAction : uint8_t {
  none,
  command_open,
  retry_open,
  hold_open,
  stop_retrying,
};

enum class ParachuteOpenState : uint8_t {
  idle,
  opening,
  retrying,
  open_confirmed,
  retry_exhausted,
  powered_off,
};

struct ParachuteTick {
  uint64_t now_us{};
  bool position_valid{};
  uint16_t position_count{};
  bool target_reached{};
};

struct ParachuteStatus {
  ParachuteOpenState state{ParachuteOpenState::idle};
  uint32_t retry_count{};
  bool servo_open_confirmed{};
  bool open_attempt_finished{};
};

class ParachuteController {
public:
  [[nodiscard]] ParachuteAction startOpen(uint64_t now_us,
                                         uint16_t initial_position_count);
  [[nodiscard]] ParachuteAction tick(const ParachuteTick &input);
  void notifyPowerCutoff();
  [[nodiscard]] const ParachuteStatus &status() const { return status_; }

private:
  // TODO(HW_TEST): STS3215 speed/torque、2 deg/0.5 s、5 s deadlineを確定する。
  static constexpr uint64_t kProgressWindowUs = 500'000;
  static constexpr uint64_t kGlobalDeadlineUs = 5'000'000;
  static constexpr int16_t kMinimumProgressCount = 23;

  ParachuteStatus status_{};
  uint64_t started_at_us_{};
  uint64_t window_started_at_us_{};
  uint16_t window_position_count_{};
};

} // 名前空間 actuators
''')

write("src/actuators/safety_core.cpp", r'''#include "actuators/safety_core.hpp"

namespace actuators {

bool PowerArbiter::requestAuxiliary5v(bool enabled) {
  if (enabled && state_.cutoff_latched)
    return false;
  state_.auxiliary_5v = enabled;
  return true;
}

bool PowerArbiter::requestParachutePower(bool enabled) {
  if (enabled && state_.cutoff_latched)
    return false;
  state_.parachute_power = enabled;
  return true;
}

void PowerArbiter::latchDeploymentCutoff() {
  state_.cutoff_latched = true;
  state_.auxiliary_5v = false;
  state_.parachute_power = false;
}

ParachuteAction
ParachuteController::startOpen(uint64_t now_us,
                              uint16_t initial_position_count) {
  if (!AbsoluteParachuteAngle::fromCount(initial_position_count).has_value() ||
      status_.state != ParachuteOpenState::idle)
    return ParachuteAction::none;
  status_ = {};
  status_.state = ParachuteOpenState::opening;
  started_at_us_ = now_us;
  window_started_at_us_ = now_us;
  window_position_count_ = initial_position_count;
  return ParachuteAction::command_open;
}

ParachuteAction ParachuteController::tick(const ParachuteTick &input) {
  if (status_.state != ParachuteOpenState::opening &&
      status_.state != ParachuteOpenState::retrying)
    return ParachuteAction::none;
  if (input.now_us < started_at_us_ ||
      input.now_us - started_at_us_ >= kGlobalDeadlineUs) {
    status_.state = ParachuteOpenState::retry_exhausted;
    status_.open_attempt_finished = true;
    return ParachuteAction::stop_retrying;
  }
  if (input.target_reached && input.position_valid) {
    status_.state = ParachuteOpenState::open_confirmed;
    status_.servo_open_confirmed = true;
    status_.open_attempt_finished = true;
    return ParachuteAction::hold_open;
  }
  if (input.now_us < window_started_at_us_ ||
      input.now_us - window_started_at_us_ < kProgressWindowUs)
    return ParachuteAction::none;

  bool progressed = false;
  const auto previous = AbsoluteParachuteAngle::fromCount(window_position_count_);
  const auto current = AbsoluteParachuteAngle::fromCount(input.position_count);
  if (input.position_valid && previous.has_value() && current.has_value()) {
    const auto displacement = shortestParachuteDisplacement(*previous, *current);
    progressed = displacement.valid() &&
                 (displacement.counts >= kMinimumProgressCount ||
                  displacement.counts <= -kMinimumProgressCount);
  }
  window_started_at_us_ = input.now_us;
  if (input.position_valid && current.has_value())
    window_position_count_ = input.position_count;
  if (progressed)
    return ParachuteAction::none;
  status_.state = ParachuteOpenState::retrying;
  ++status_.retry_count;
  return ParachuteAction::retry_open;
}

void ParachuteController::notifyPowerCutoff() {
  status_.state = ParachuteOpenState::powered_off;
}

} // 名前空間 actuators
''')

write("src/mission/mission_state.hpp", r'''#pragma once

#include <cstdint>
#include <limits>

#include "protocol/can_protocol.hpp"

namespace mission {

enum class FinDirective : uint8_t { free, brake, zero_hold, roll_control };
enum class ParaDirective : uint8_t { hold, open, powered_off };
enum class StartMode : uint8_t { normal, forced };

enum class PreflightReadinessBit : uint8_t {
  fin_zero_configured = 0,
  parachute_open_configured = 1,
  parachute_close_configured = 2,
  motor_profile_valid = 3,
  gyro_bias_valid = 4,
  gravity_reference_valid = 5,
  ssc_zero_valid = 6,
};

constexpr uint8_t kPreflightReadinessMask = 0x7FU;

struct PreflightReadinessSnapshot {
  uint32_t generation{};
  uint64_t captured_at_us{};
  bool fin_zero_configured{};
  bool parachute_open_configured{};
  bool parachute_close_configured{};
  bool motor_profile_valid{};
  bool gyro_bias_valid{};
  bool gravity_reference_valid{};
  bool ssc_zero_valid{};
  bool resources_preallocated{};

  [[nodiscard]] uint8_t readyMask() const;
  [[nodiscard]] uint8_t missingMask() const;
  [[nodiscard]] bool normalReady() const;
};

struct ControlAvailability {
  bool fin_control_available{};
  bool fin_zero_hold_valid{};
  bool attitude_valid{};
  bool airspeed_above_60{};
  bool lps_available{};
  bool ssc_available{};
  bool gyro_bias_valid{};
  bool ssc_zero_valid{};
  double roll_estimate_liftoff_relative_unwrapped_rad{
      std::numeric_limits<double>::quiet_NaN()};
  uint64_t roll_estimator_timestamp_us{};
  bool attitude_fresh{};
  [[nodiscard]] bool ready() const;
};

struct MissionTickInput {
  uint64_t monotonic_us{};
  bool liftoff_detected{};
  bool deployment_pressure_condition{};
  ControlAvailability control{};
  uint64_t control_tick{};
};

struct SafetyRequest {
  uint32_t flight_epoch{};
  bool deploy{};
  bool absolute_power_cutoff{};
};

struct MissionSnapshot {
  protocol::MissionState state{protocol::MissionState::command_receive};
  uint32_t flight_epoch{};
  bool liftoff_time_valid{};
  uint64_t liftoff_time_us{};
  uint64_t elapsed_us{};
  bool fin_control_disabled{};
  bool control_reentry_inhibited{};
  bool reset_invalidated{};
  bool deployment_started{};
  bool deployment_power_cutoff_latched{};
  bool forced_start{};
  uint32_t preflight_generation{};
  uint64_t preflight_captured_at_us{};
  uint8_t preflight_ready_mask{};
  uint8_t preflight_missing_mask{};
  double control_roll_reference_unwrapped_rad{};
  uint64_t control_roll_reference_capture_tick{};
  uint64_t control_roll_reference_estimator_timestamp_us{};
  uint8_t control_roll_reference_capture_event_sequence{};
  bool control_roll_reference_valid{};
  FinDirective fin{FinDirective::brake};
  ParaDirective parachute{ParaDirective::hold};
};

enum class TransitionResult : uint8_t {
  completed,
  invalid_state,
  not_configured,
  runtime_unavailable,
};

struct ResetCheckpoint {
  bool valid{};
  protocol::MissionState state{protocol::MissionState::command_receive};
  uint32_t flight_epoch{};
  bool elapsed_valid{};
  uint64_t elapsed_us{};
  bool deployment_started{};
  bool power_cutoff_latched{};
  bool forced_start{};
  uint32_t preflight_generation{};
  uint64_t preflight_captured_at_us{};
  uint8_t preflight_ready_mask{};
  uint8_t preflight_missing_mask{};
};

class MissionStateMachine {
public:
  [[nodiscard]] const MissionSnapshot &snapshot() const { return snapshot_; }
  [[nodiscard]] TransitionResult
  startSequence(uint64_t now_us, const PreflightReadinessSnapshot &readiness,
                StartMode mode);
  [[nodiscard]] TransitionResult cancelSequence();
  [[nodiscard]] TransitionResult disableFinControl();
  [[nodiscard]] TransitionResult liftoffDetectionEmergencyStop();
  [[nodiscard]] TransitionResult
  restoreAfterReset(uint64_t now_us, const ResetCheckpoint &checkpoint);
  void tick(const MissionTickInput &input,
            const SafetyRequest &safety = SafetyRequest{});

private:
  void enterDescent();
  void updateDirectives(uint64_t now_us);
  void invalidateLiftoff();
  void invalidateControlRollReference();
  void clearFlightAttemptMetadata();
  [[nodiscard]] bool captureControlRollReference(const MissionTickInput &input);

  MissionSnapshot snapshot_{};
  bool control_gate_evaluated_{};
  bool fin_control_available_{};
  uint64_t elapsed_offset_us_{};
  uint8_t control_roll_reference_capture_event_sequence_{};
};

} // 名前空間 mission
''')

write("src/mission/mission_state.cpp", r'''#include "mission/mission_state.hpp"

#include <cmath>

namespace mission {
namespace {
constexpr uint64_t kOneSecondUs = 1'000'000;
constexpr uint64_t kControlGateUs = 8 * kOneSecondUs;
constexpr uint64_t kPressureDeploymentGateUs = 10 * kOneSecondUs;
constexpr uint64_t kDeploymentDeadlineUs = 17 * kOneSecondUs;
constexpr uint64_t kPowerCutoffUs = 25 * kOneSecondUs;

uint32_t nextEpoch(uint32_t current) {
  ++current;
  return current == 0 ? 1 : current;
}

constexpr uint8_t bit(PreflightReadinessBit value) {
  return static_cast<uint8_t>(1U << static_cast<uint8_t>(value));
}
} // 無名名前空間

uint8_t PreflightReadinessSnapshot::readyMask() const {
  uint8_t result = 0;
  if (fin_zero_configured)
    result |= bit(PreflightReadinessBit::fin_zero_configured);
  if (parachute_open_configured)
    result |= bit(PreflightReadinessBit::parachute_open_configured);
  if (parachute_close_configured)
    result |= bit(PreflightReadinessBit::parachute_close_configured);
  if (motor_profile_valid)
    result |= bit(PreflightReadinessBit::motor_profile_valid);
  if (gyro_bias_valid)
    result |= bit(PreflightReadinessBit::gyro_bias_valid);
  if (gravity_reference_valid)
    result |= bit(PreflightReadinessBit::gravity_reference_valid);
  if (ssc_zero_valid)
    result |= bit(PreflightReadinessBit::ssc_zero_valid);
  return result;
}

uint8_t PreflightReadinessSnapshot::missingMask() const {
  return static_cast<uint8_t>((~readyMask()) & kPreflightReadinessMask);
}

bool PreflightReadinessSnapshot::normalReady() const {
  return resources_preallocated && missingMask() == 0;
}

bool ControlAvailability::ready() const {
  return fin_control_available && fin_zero_hold_valid && attitude_valid &&
         airspeed_above_60 && lps_available && ssc_available &&
         gyro_bias_valid && ssc_zero_valid && attitude_fresh &&
         std::isfinite(roll_estimate_liftoff_relative_unwrapped_rad) &&
         roll_estimator_timestamp_us != 0;
}

TransitionResult MissionStateMachine::startSequence(
    uint64_t, const PreflightReadinessSnapshot &readiness, StartMode mode) {
  if (snapshot_.state != protocol::MissionState::command_receive)
    return TransitionResult::invalid_state;
  if (!readiness.resources_preallocated)
    return TransitionResult::runtime_unavailable;
  if (mode == StartMode::normal && readiness.missingMask() != 0)
    return TransitionResult::not_configured;

  snapshot_.flight_epoch = nextEpoch(snapshot_.flight_epoch);
  snapshot_.state = protocol::MissionState::liftoff_detection;
  snapshot_.fin_control_disabled = false;
  snapshot_.control_reentry_inhibited = false;
  snapshot_.reset_invalidated = false;
  snapshot_.deployment_started = false;
  snapshot_.deployment_power_cutoff_latched = false;
  snapshot_.forced_start = mode == StartMode::forced;
  snapshot_.preflight_generation = readiness.generation;
  snapshot_.preflight_captured_at_us = readiness.captured_at_us;
  snapshot_.preflight_ready_mask = readiness.readyMask();
  snapshot_.preflight_missing_mask = readiness.missingMask();
  snapshot_.parachute = ParaDirective::hold;
  control_gate_evaluated_ = false;
  fin_control_available_ = readiness.fin_zero_configured;
  elapsed_offset_us_ = 0;
  invalidateControlRollReference();
  invalidateLiftoff();
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::cancelSequence() {
  if (snapshot_.state != protocol::MissionState::liftoff_detection)
    return TransitionResult::invalid_state;
  snapshot_.state = protocol::MissionState::command_receive;
  snapshot_.fin_control_disabled = false;
  snapshot_.control_reentry_inhibited = false;
  snapshot_.deployment_started = false;
  control_gate_evaluated_ = false;
  fin_control_available_ = false;
  elapsed_offset_us_ = 0;
  invalidateControlRollReference();
  invalidateLiftoff();
  clearFlightAttemptMetadata();
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::disableFinControl() {
  if (snapshot_.state != protocol::MissionState::liftoff_detection &&
      snapshot_.state != protocol::MissionState::engine_burn &&
      snapshot_.state != protocol::MissionState::control)
    return TransitionResult::invalid_state;
  snapshot_.fin_control_disabled = true;
  snapshot_.control_reentry_inhibited = true;
  invalidateControlRollReference();
  if (snapshot_.state == protocol::MissionState::control)
    snapshot_.state = protocol::MissionState::engine_burn;
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::liftoffDetectionEmergencyStop() {
  if (snapshot_.state != protocol::MissionState::engine_burn)
    return TransitionResult::invalid_state;
  snapshot_.flight_epoch = nextEpoch(snapshot_.flight_epoch);
  snapshot_.state = protocol::MissionState::liftoff_detection;
  snapshot_.control_reentry_inhibited = snapshot_.fin_control_disabled;
  snapshot_.deployment_started = false;
  control_gate_evaluated_ = false;
  elapsed_offset_us_ = 0;
  invalidateControlRollReference();
  invalidateLiftoff();
  // 同一flight attempt由来なのでforced_start/preflight snapshotは維持する。
  updateDirectives(0);
  return TransitionResult::completed;
}

TransitionResult MissionStateMachine::restoreAfterReset(
    uint64_t now_us, const ResetCheckpoint &checkpoint) {
  if (!checkpoint.valid || checkpoint.flight_epoch == 0)
    return TransitionResult::not_configured;
  if (checkpoint.state == protocol::MissionState::liftoff_detection &&
      checkpoint.elapsed_valid)
    return TransitionResult::not_configured;
  if (checkpoint.state != protocol::MissionState::liftoff_detection &&
      (!checkpoint.elapsed_valid ||
       (checkpoint.state != protocol::MissionState::engine_burn &&
        checkpoint.state != protocol::MissionState::control &&
        checkpoint.state != protocol::MissionState::descent)))
    return TransitionResult::not_configured;

  snapshot_ = {};
  snapshot_.flight_epoch = checkpoint.flight_epoch;
  snapshot_.forced_start = checkpoint.forced_start;
  snapshot_.preflight_generation = checkpoint.preflight_generation;
  snapshot_.preflight_captured_at_us = checkpoint.preflight_captured_at_us;
  snapshot_.preflight_ready_mask = checkpoint.preflight_ready_mask;
  snapshot_.preflight_missing_mask = checkpoint.preflight_missing_mask;
  snapshot_.control_reentry_inhibited = true;
  snapshot_.reset_invalidated = true;
  invalidateControlRollReference();

  if (checkpoint.state == protocol::MissionState::liftoff_detection) {
    snapshot_.state = protocol::MissionState::liftoff_detection;
    snapshot_.parachute = ParaDirective::hold;
    control_gate_evaluated_ = false;
    fin_control_available_ = false;
    elapsed_offset_us_ = 0;
    invalidateLiftoff();
    updateDirectives(now_us);
    return TransitionResult::completed;
  }

  snapshot_.state = checkpoint.deployment_started ||
                            checkpoint.state == protocol::MissionState::descent
                        ? protocol::MissionState::descent
                        : protocol::MissionState::engine_burn;
  snapshot_.liftoff_time_valid = true;
  snapshot_.liftoff_time_us = now_us;
  snapshot_.elapsed_us = checkpoint.elapsed_us;
  snapshot_.deployment_started = checkpoint.deployment_started;
  snapshot_.deployment_power_cutoff_latched = checkpoint.power_cutoff_latched;
  snapshot_.fin = FinDirective::brake;
  snapshot_.parachute = checkpoint.power_cutoff_latched
                            ? ParaDirective::powered_off
                            : (checkpoint.deployment_started
                                   ? ParaDirective::open
                                   : ParaDirective::hold);
  control_gate_evaluated_ = true;
  fin_control_available_ = false;
  elapsed_offset_us_ = checkpoint.elapsed_us;
  updateDirectives(now_us);
  return TransitionResult::completed;
}

void MissionStateMachine::tick(const MissionTickInput &input,
                               const SafetyRequest &safety) {
  const bool current_safety = safety.flight_epoch != 0 &&
                              safety.flight_epoch == snapshot_.flight_epoch;
  if (current_safety && snapshot_.liftoff_time_valid &&
      safety.absolute_power_cutoff)
    snapshot_.deployment_power_cutoff_latched = true;

  if (snapshot_.state == protocol::MissionState::liftoff_detection &&
      input.liftoff_detected) {
    snapshot_.liftoff_time_valid = true;
    snapshot_.liftoff_time_us = input.monotonic_us >= kOneSecondUs
                                    ? input.monotonic_us - kOneSecondUs
                                    : 0;
    snapshot_.state = protocol::MissionState::engine_burn;
    control_gate_evaluated_ = false;
    elapsed_offset_us_ = 0;
  }

  uint64_t elapsed_us{};
  if (snapshot_.liftoff_time_valid &&
      input.monotonic_us >= snapshot_.liftoff_time_us)
    elapsed_us = elapsed_offset_us_ + input.monotonic_us - snapshot_.liftoff_time_us;
  snapshot_.elapsed_us = elapsed_us;
  fin_control_available_ = input.control.fin_control_available;

  const bool flight_state = snapshot_.state == protocol::MissionState::engine_burn ||
                            snapshot_.state == protocol::MissionState::control;
  if (flight_state &&
      ((current_safety && safety.deploy) ||
       (input.deployment_pressure_condition && snapshot_.liftoff_time_valid &&
        elapsed_us >= kPressureDeploymentGateUs) ||
       (snapshot_.liftoff_time_valid && elapsed_us >= kDeploymentDeadlineUs)))
    enterDescent();

  if (snapshot_.state == protocol::MissionState::engine_burn &&
      snapshot_.liftoff_time_valid && elapsed_us >= kControlGateUs &&
      !control_gate_evaluated_) {
    control_gate_evaluated_ = true;
    if (!snapshot_.fin_control_disabled && !snapshot_.control_reentry_inhibited &&
        input.control.ready() && captureControlRollReference(input))
      snapshot_.state = protocol::MissionState::control;
    else
      snapshot_.control_reentry_inhibited = true;
  }

  if (snapshot_.state == protocol::MissionState::control &&
      !input.control.ready()) {
    snapshot_.state = protocol::MissionState::engine_burn;
    snapshot_.control_reentry_inhibited = true;
  }

  if (snapshot_.liftoff_time_valid && elapsed_us >= kPowerCutoffUs)
    snapshot_.deployment_power_cutoff_latched = true;
  updateDirectives(input.monotonic_us);
}

void MissionStateMachine::enterDescent() {
  snapshot_.state = protocol::MissionState::descent;
  snapshot_.deployment_started = true;
  snapshot_.parachute = ParaDirective::open;
}

void MissionStateMachine::updateDirectives(uint64_t now_us) {
  if (snapshot_.deployment_power_cutoff_latched)
    snapshot_.parachute = ParaDirective::powered_off;
  else if (snapshot_.deployment_started)
    snapshot_.parachute = ParaDirective::open;
  else
    snapshot_.parachute = ParaDirective::hold;

  if (snapshot_.reset_invalidated || snapshot_.fin_control_disabled) {
    snapshot_.fin = FinDirective::brake;
    return;
  }
  if (snapshot_.state == protocol::MissionState::control) {
    snapshot_.fin = FinDirective::roll_control;
    return;
  }
  if (snapshot_.state == protocol::MissionState::liftoff_detection ||
      snapshot_.state == protocol::MissionState::engine_burn ||
      snapshot_.state == protocol::MissionState::descent) {
    if (snapshot_.liftoff_time_valid &&
        now_us >= snapshot_.liftoff_time_us + kPowerCutoffUs)
      snapshot_.fin = FinDirective::brake;
    else if (fin_control_available_)
      snapshot_.fin = FinDirective::zero_hold;
    else
      snapshot_.fin = FinDirective::brake;
    return;
  }
  snapshot_.fin = FinDirective::brake;
}

void MissionStateMachine::invalidateLiftoff() {
  snapshot_.liftoff_time_valid = false;
  snapshot_.liftoff_time_us = 0;
  snapshot_.elapsed_us = 0;
  elapsed_offset_us_ = 0;
}

void MissionStateMachine::invalidateControlRollReference() {
  snapshot_.control_roll_reference_unwrapped_rad = 0.0;
  snapshot_.control_roll_reference_capture_tick = 0;
  snapshot_.control_roll_reference_estimator_timestamp_us = 0;
  snapshot_.control_roll_reference_capture_event_sequence =
      control_roll_reference_capture_event_sequence_;
  snapshot_.control_roll_reference_valid = false;
}

void MissionStateMachine::clearFlightAttemptMetadata() {
  snapshot_.forced_start = false;
  snapshot_.preflight_generation = 0;
  snapshot_.preflight_captured_at_us = 0;
  snapshot_.preflight_ready_mask = 0;
  snapshot_.preflight_missing_mask = 0;
}

bool MissionStateMachine::captureControlRollReference(
    const MissionTickInput &input) {
  if (snapshot_.control_roll_reference_valid)
    return true;
  if (input.control.roll_estimator_timestamp_us > input.monotonic_us ||
      input.monotonic_us - input.control.roll_estimator_timestamp_us > 3'000)
    return false;
  ++control_roll_reference_capture_event_sequence_;
  snapshot_.control_roll_reference_unwrapped_rad =
      input.control.roll_estimate_liftoff_relative_unwrapped_rad;
  snapshot_.control_roll_reference_capture_tick = input.control_tick;
  snapshot_.control_roll_reference_estimator_timestamp_us =
      input.control.roll_estimator_timestamp_us;
  snapshot_.control_roll_reference_capture_event_sequence =
      control_roll_reference_capture_event_sequence_;
  snapshot_.control_roll_reference_valid = true;
  return true;
}

} // 名前空間 mission
''')

write("src/mission/command_executor.hpp", r'''#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "protocol/can_protocol.hpp"

namespace mission {

enum class CommandCode : uint8_t {
  start_sequence = 0x01,
  cancel_sequence = 0x02,
  disable_fin_control = 0x03,
  force_start_sequence = 0x04,
  fin_free = 0x10,
  set_fin_zero = 0x11,
  start_fin_zero_hold = 0x12,
  fin_move_relative = 0x13,
  para_free = 0x20,
  para_hold = 0x21,
  para_move_relative = 0x22,
  set_para_open = 0x23,
  set_para_close = 0x24,
  para_open = 0x25,
  para_close = 0x26,
  run_preflight_calibration = 0x30,
  export_flash_log = 0x31,
  select_motor_profile = 0x32,
  actuator_emergency_result = 0xF0,
  liftoff_emergency_result = 0xF1,
};

enum class CommandDomain : uint8_t {
  sequence,
  fin,
  parachute,
  calibration,
  storage,
  motor_profile,
  count,
};

struct CommandContext {
  protocol::MissionState state{protocol::MissionState::command_receive};
  bool resources_preallocated{};
  bool persistence_load_complete{};
  bool persistence_runtime_available{};
  bool fin_available{true};
  bool parachute_available{true};
  bool motor_test_busy{};
  bool calibration_supported{};
  bool storage_export_supported{};
  bool motor_profile_selection_supported{};
  bool fin_safe_commands_supported{};
};

struct CommandDecision {
  protocol::CommandResult result{};
  bool execute{};
  bool replay{};
  CommandDomain domain{CommandDomain::sequence};
};

struct EmergencyDecision {
  protocol::CommandResult result{};
  std::array<protocol::CommandResult, 2> interrupted{};
  std::size_t interrupted_count{};
  bool execute{};
};

class CommandExecutor {
public:
  static constexpr std::size_t kResultCacheSize = 16;
  [[nodiscard]] CommandDecision
  begin(const protocol::GenericCommandRequest &request,
        const CommandContext &context);
  [[nodiscard]] protocol::CommandResult
  finish(uint8_t transaction_id, protocol::CommandPhase phase,
         protocol::CommandReason reason = protocol::CommandReason::none,
         uint32_t detail = 0);
  [[nodiscard]] EmergencyDecision
  actuatorEmergency(uint8_t transaction_id, protocol::MissionState state);
  [[nodiscard]] protocol::CommandResult
  liftoffEmergencyResult(uint8_t transaction_id, bool accepted);
  [[nodiscard]] bool busy(CommandDomain domain) const;
  [[nodiscard]] std::size_t cachedCount() const;

private:
  struct Entry {
    bool valid{};
    bool pending{};
    uint32_t age{};
    protocol::GenericCommandRequest request{};
    protocol::CommandResult result{};
    CommandDomain domain{CommandDomain::sequence};
  };
  [[nodiscard]] Entry *find(uint8_t transaction_id);
  [[nodiscard]] const Entry *find(uint8_t transaction_id) const;
  [[nodiscard]] Entry *allocate();
  void remember(const protocol::GenericCommandRequest &request,
                const protocol::CommandResult &result, bool pending,
                CommandDomain domain);
  std::array<Entry, kResultCacheSize> entries_{};
  uint32_t age_{};
};

} // 名前空間 mission
''')

write("src/mission/command_executor.cpp", r'''#include "mission/command_executor.hpp"

#include <algorithm>
#include <cstdlib>

namespace mission {
namespace {
using protocol::CommandPhase;
using protocol::CommandReason;
using protocol::CommandResult;
using protocol::GenericCommandRequest;
using protocol::MissionState;

bool sameRequest(const GenericCommandRequest &left,
                 const GenericCommandRequest &right) {
  return left.transaction_id == right.transaction_id &&
         left.command == right.command && left.arguments == right.arguments;
}

bool allZero(const GenericCommandRequest &request, std::size_t first) {
  return std::all_of(request.arguments.begin() + first, request.arguments.end(),
                     [](uint8_t value) { return value == 0; });
}

bool commandKnown(CommandCode code) {
  const uint8_t raw = static_cast<uint8_t>(code);
  return (raw >= 0x01 && raw <= 0x04) ||
         (raw >= 0x10 && raw <= 0x13) ||
         (raw >= 0x20 && raw <= 0x26) ||
         (raw >= 0x30 && raw <= 0x32);
}

CommandDomain domainFor(CommandCode code) {
  switch (code) {
  case CommandCode::start_sequence:
  case CommandCode::cancel_sequence:
  case CommandCode::disable_fin_control:
  case CommandCode::force_start_sequence:
    return CommandDomain::sequence;
  case CommandCode::fin_free:
  case CommandCode::set_fin_zero:
  case CommandCode::start_fin_zero_hold:
  case CommandCode::fin_move_relative:
    return CommandDomain::fin;
  case CommandCode::para_free:
  case CommandCode::para_hold:
  case CommandCode::para_move_relative:
  case CommandCode::set_para_open:
  case CommandCode::set_para_close:
  case CommandCode::para_open:
  case CommandCode::para_close:
    return CommandDomain::parachute;
  case CommandCode::run_preflight_calibration:
    return CommandDomain::calibration;
  case CommandCode::export_flash_log:
    return CommandDomain::storage;
  case CommandCode::select_motor_profile:
    return CommandDomain::motor_profile;
  default:
    return CommandDomain::sequence;
  }
}

bool argumentsValid(const GenericCommandRequest &request, CommandCode code) {
  switch (code) {
  case CommandCode::fin_move_relative:
    return allZero(request, 2);
  case CommandCode::para_move_relative: {
    if (!allZero(request, 2))
      return false;
    const auto raw = static_cast<uint16_t>(request.arguments[0]) |
                     static_cast<uint16_t>(request.arguments[1]) << 8U;
    return std::abs(static_cast<int32_t>(static_cast<int16_t>(raw))) < 1800;
  }
  case CommandCode::select_motor_profile:
    return request.arguments[0] != 0 && allZero(request, 1);
  default:
    return allZero(request, 0);
  }
}

bool stateValid(CommandCode code, MissionState state) {
  switch (code) {
  case CommandCode::start_sequence:
  case CommandCode::force_start_sequence:
  case CommandCode::fin_free:
  case CommandCode::set_fin_zero:
  case CommandCode::start_fin_zero_hold:
  case CommandCode::fin_move_relative:
  case CommandCode::para_free:
  case CommandCode::para_hold:
  case CommandCode::para_move_relative:
  case CommandCode::set_para_open:
  case CommandCode::set_para_close:
  case CommandCode::para_open:
  case CommandCode::para_close:
  case CommandCode::run_preflight_calibration:
  case CommandCode::export_flash_log:
  case CommandCode::select_motor_profile:
    return state == MissionState::command_receive;
  case CommandCode::cancel_sequence:
    return state == MissionState::liftoff_detection;
  case CommandCode::disable_fin_control:
    return state == MissionState::liftoff_detection ||
           state == MissionState::engine_burn || state == MissionState::control;
  default:
    return false;
  }
}

bool isStart(CommandCode code) {
  return code == CommandCode::start_sequence ||
         code == CommandCode::force_start_sequence;
}

CommandResult resultFor(const GenericCommandRequest &request,
                        CommandPhase phase, CommandReason reason,
                        uint32_t detail = 0) {
  return {request.transaction_id, request.command, phase, reason, detail};
}
} // 無名名前空間

CommandDecision CommandExecutor::begin(const GenericCommandRequest &request,
                                       const CommandContext &context) {
  if (request.transaction_id == 0)
    return {resultFor(request, CommandPhase::rejected,
                      CommandReason::invalid_argument),
            false, false, CommandDomain::sequence};

  if (const Entry *const existing = find(request.transaction_id)) {
    if (sameRequest(existing->request, request))
      return {existing->result, false, true, existing->domain};
    return {resultFor(request, CommandPhase::rejected,
                      CommandReason::protocol_error),
            false, false, existing->domain};
  }

  const auto code = static_cast<CommandCode>(request.command);
  const CommandDomain domain = domainFor(code);
  CommandReason rejection = CommandReason::none;
  if (!commandKnown(code))
    rejection = CommandReason::not_supported;
  else if (!argumentsValid(request, code))
    rejection = CommandReason::invalid_argument;
  else if (!stateValid(code, context.state))
    rejection = CommandReason::invalid_state;
  else if (isStart(code) &&
           (busy(CommandDomain::parachute) || busy(CommandDomain::fin) ||
            busy(CommandDomain::calibration) || context.motor_test_busy))
    rejection = CommandReason::busy;
  else if (isStart(code) && !context.resources_preallocated)
    rejection = CommandReason::internal_error;
  else if (isStart(code) && !context.persistence_load_complete)
    rejection = CommandReason::busy;
  else if (isStart(code) && !context.persistence_runtime_available)
    rejection = CommandReason::persistence_error;
  else if (domain == CommandDomain::parachute && busy(CommandDomain::sequence))
    rejection = CommandReason::busy;
  else if ((domain == CommandDomain::fin && !context.fin_available) ||
           (domain == CommandDomain::parachute && !context.parachute_available))
    rejection = CommandReason::device_unavailable;
  else if (domain == CommandDomain::fin &&
           !context.fin_safe_commands_supported)
    rejection = CommandReason::not_supported;
  else if ((domain == CommandDomain::calibration &&
            !context.calibration_supported) ||
           (domain == CommandDomain::storage &&
            !context.storage_export_supported) ||
           (domain == CommandDomain::motor_profile &&
            !context.motor_profile_selection_supported))
    rejection = CommandReason::not_supported;
  else if (busy(domain))
    rejection = CommandReason::busy;
  else if (isStart(code) && cachedCount() != 0 &&
           std::any_of(entries_.begin(), entries_.end(),
                       [](const Entry &entry) { return entry.pending; }))
    rejection = CommandReason::busy;
  else if (domain == CommandDomain::calibration &&
           (busy(CommandDomain::fin) || busy(CommandDomain::parachute) ||
            context.motor_test_busy))
    rejection = CommandReason::busy;
  else if ((domain == CommandDomain::fin || domain == CommandDomain::parachute) &&
           busy(CommandDomain::calibration))
    rejection = CommandReason::busy;

  if (rejection != CommandReason::none) {
    const auto result = resultFor(request, CommandPhase::rejected, rejection);
    remember(request, result, false, domain);
    return {result, false, false, domain};
  }
  const auto result = resultFor(request, CommandPhase::accepted, CommandReason::none);
  remember(request, result, true, domain);
  return {result, true, false, domain};
}

CommandResult CommandExecutor::finish(uint8_t transaction_id,
                                      CommandPhase phase,
                                      CommandReason reason, uint32_t detail) {
  Entry *const entry = find(transaction_id);
  if (entry == nullptr || !entry->pending ||
      (phase != CommandPhase::completed && phase != CommandPhase::failed))
    return {transaction_id, 0, CommandPhase::failed,
            CommandReason::internal_error, detail};
  entry->pending = false;
  entry->age = ++age_;
  entry->result.phase = phase;
  entry->result.reason = reason;
  entry->result.detail = detail;
  return entry->result;
}

EmergencyDecision CommandExecutor::actuatorEmergency(uint8_t transaction_id,
                                                      MissionState state) {
  EmergencyDecision decision{};
  const bool accepted = transaction_id != 0 && state == MissionState::command_receive;
  decision.result = {transaction_id,
                     static_cast<uint8_t>(CommandCode::actuator_emergency_result),
                     accepted ? CommandPhase::completed : CommandPhase::rejected,
                     accepted ? CommandReason::none
                              : (transaction_id == 0 ? CommandReason::invalid_argument
                                                     : CommandReason::invalid_state),
                     0};
  decision.execute = accepted;
  if (!decision.execute)
    return decision;
  for (auto &entry : entries_) {
    if (!entry.pending ||
        (entry.domain != CommandDomain::fin &&
         entry.domain != CommandDomain::parachute))
      continue;
    entry.pending = false;
    entry.age = ++age_;
    entry.result.phase = CommandPhase::failed;
    entry.result.reason = CommandReason::interrupted_by_emergency;
    if (decision.interrupted_count < decision.interrupted.size())
      decision.interrupted[decision.interrupted_count++] = entry.result;
  }
  return decision;
}

CommandResult CommandExecutor::liftoffEmergencyResult(uint8_t transaction_id,
                                                      bool accepted) {
  const bool valid = transaction_id != 0 && accepted;
  return {transaction_id,
          static_cast<uint8_t>(CommandCode::liftoff_emergency_result),
          valid ? CommandPhase::completed : CommandPhase::rejected,
          valid ? CommandReason::none
                : (transaction_id == 0 ? CommandReason::invalid_argument
                                       : CommandReason::invalid_state),
          0};
}

bool CommandExecutor::busy(CommandDomain domain) const {
  return std::any_of(entries_.begin(), entries_.end(), [domain](const Entry &entry) {
    return entry.valid && entry.pending && entry.domain == domain;
  });
}

std::size_t CommandExecutor::cachedCount() const {
  return static_cast<std::size_t>(std::count_if(
      entries_.begin(), entries_.end(), [](const Entry &entry) { return entry.valid; }));
}

CommandExecutor::Entry *CommandExecutor::find(uint8_t transaction_id) {
  const auto iterator = std::find_if(entries_.begin(), entries_.end(),
                                     [transaction_id](const Entry &entry) {
    return entry.valid && entry.request.transaction_id == transaction_id;
  });
  return iterator == entries_.end() ? nullptr : &*iterator;
}
const CommandExecutor::Entry *CommandExecutor::find(uint8_t transaction_id) const {
  const auto iterator = std::find_if(entries_.begin(), entries_.end(),
                                     [transaction_id](const Entry &entry) {
    return entry.valid && entry.request.transaction_id == transaction_id;
  });
  return iterator == entries_.end() ? nullptr : &*iterator;
}
CommandExecutor::Entry *CommandExecutor::allocate() {
  const auto unused = std::find_if(entries_.begin(), entries_.end(),
                                   [](const Entry &entry) { return !entry.valid; });
  if (unused != entries_.end())
    return &*unused;
  auto oldest = entries_.end();
  for (auto iterator = entries_.begin(); iterator != entries_.end(); ++iterator) {
    if (!iterator->pending &&
        (oldest == entries_.end() || iterator->age < oldest->age))
      oldest = iterator;
  }
  return oldest == entries_.end() ? nullptr : &*oldest;
}
void CommandExecutor::remember(const GenericCommandRequest &request,
                               const CommandResult &result, bool pending,
                               CommandDomain domain) {
  Entry *const entry = allocate();
  if (entry == nullptr)
    return;
  *entry = {true, pending, ++age_, request, result, domain};
}

} // 名前空間 mission
''')

write("src/runtime/recovery_boot.cpp", r'''#include "runtime/recovery_boot.hpp"

#include "esp_attr.h"
#include "esp_rtc_time.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "mission/recovery.hpp"

namespace runtime::recovery_boot {
namespace {
RTC_DATA_ATTR mission::RecoveryMarker recovery_marker{};

constexpr uint32_t kFlightCheckpointMagic = 0x39394C43;
constexpr uint16_t kFlightCheckpointVersion = 2;
struct FlightCheckpointRecord {
  uint32_t magic{};
  uint16_t version{};
  uint16_t checksum{};
  uint32_t flight_epoch{};
  uint64_t elapsed_us{};
  uint64_t rtc_time_us{};
  uint64_t preflight_captured_at_us{};
  uint32_t preflight_generation{};
  uint8_t state{};
  uint8_t preflight_ready_mask{};
  uint8_t preflight_missing_mask{};
  bool elapsed_valid{};
  bool deployment_started{};
  bool power_cutoff_latched{};
  bool forced_start{};
};
RTC_DATA_ATTR FlightCheckpointRecord flight_checkpoint{};

constexpr uint32_t kParachuteCheckpointMagic = 0x39395043;
constexpr uint8_t kParachuteCheckpointVersion = 2;
constexpr uint8_t kOpenValid = 1U << 0U;
constexpr uint8_t kCloseValid = 1U << 1U;
struct ParachuteCheckpointRecord {
  uint32_t magic{};
  uint8_t version{};
  uint8_t valid_mask{};
  uint16_t open_count{};
  uint16_t close_count{};
  uint16_t checksum{};
};
RTC_DATA_ATTR ParachuteCheckpointRecord parachute_checkpoint{};

uint16_t parachuteCheckpointChecksum(const ParachuteCheckpointRecord &record) {
  uint32_t value = record.magic ^
                   (static_cast<uint32_t>(record.version) << 24U) ^
                   (static_cast<uint32_t>(record.valid_mask) << 16U) ^
                   (static_cast<uint32_t>(record.open_count) << 16U) ^
                   record.close_count;
  value ^= value >> 16U;
  return static_cast<uint16_t>(value);
}

uint16_t checkpointChecksum(const FlightCheckpointRecord &record) {
  uint64_t value = record.magic ^
                   (static_cast<uint32_t>(record.version) << 16U) ^
                   record.flight_epoch ^ record.elapsed_us ^
                   (record.elapsed_us >> 32U) ^ record.rtc_time_us ^
                   (record.rtc_time_us >> 32U) ^ record.preflight_captured_at_us ^
                   (record.preflight_captured_at_us >> 32U) ^
                   record.preflight_generation ^
                   (static_cast<uint32_t>(record.state) << 24U) ^
                   (static_cast<uint32_t>(record.preflight_ready_mask) << 8U) ^
                   record.preflight_missing_mask ^
                   (record.elapsed_valid ? 0xA55AU : 0U) ^
                   (record.deployment_started ? 0x5AA5U : 0U) ^
                   (record.power_cutoff_latched ? 0x9696U : 0U) ^
                   (record.forced_start ? 0xC33CU : 0U);
  value ^= value >> 32U;
  value ^= value >> 16U;
  return static_cast<uint16_t>(value);
}

bool validFlightCheckpoint(const FlightCheckpointRecord &record) {
  const auto state = static_cast<protocol::MissionState>(record.state);
  const bool state_valid =
      state == protocol::MissionState::liftoff_detection ||
      state == protocol::MissionState::engine_burn ||
      state == protocol::MissionState::control ||
      state == protocol::MissionState::descent;
  const bool elapsed_contract =
      state == protocol::MissionState::liftoff_detection
          ? !record.elapsed_valid
          : record.elapsed_valid;
  const uint8_t masks = static_cast<uint8_t>(
      (record.preflight_ready_mask | record.preflight_missing_mask) &
      mission::kPreflightReadinessMask);
  const bool masks_disjoint =
      (record.preflight_ready_mask & record.preflight_missing_mask) == 0;
  return record.magic == kFlightCheckpointMagic &&
         record.version == kFlightCheckpointVersion && record.flight_epoch != 0 &&
         state_valid && elapsed_contract &&
         masks == mission::kPreflightReadinessMask && masks_disjoint &&
         record.checksum == checkpointChecksum(record);
}

bool resetPreservesRtcMemory() {
  const esp_reset_reason_t reason = esp_reset_reason();
  return reason != ESP_RST_POWERON && reason != ESP_RST_DEEPSLEEP;
}
} // 無名名前空間

bool markerValid() { return mission::validRecoveryMarker(recovery_marker); }
bool wakeCauseValid() {
  return (esp_sleep_get_wakeup_causes() &
          (uint32_t{1} << ESP_SLEEP_WAKEUP_TIMER)) != 0;
}
uint32_t wakeSequence() {
  return mission::validRecoveryMarker(recovery_marker)
             ? recovery_marker.wake_sequence
             : 0;
}
void prepareMarker() {
  recovery_marker = mission::makeRecoveryMarker(wakeSequence() + 1U);
}
void clearMarker() { recovery_marker = {}; }

bool loadFlightCheckpoint(mission::ResetCheckpoint &checkpoint) {
  checkpoint = {};
  if (!resetPreservesRtcMemory() || !validFlightCheckpoint(flight_checkpoint))
    return false;
  uint64_t elapsed = flight_checkpoint.elapsed_us;
  if (flight_checkpoint.elapsed_valid) {
    const uint64_t rtc_now_us = esp_rtc_get_time_us();
    if (rtc_now_us < flight_checkpoint.rtc_time_us)
      return false;
    const uint64_t reset_elapsed_us = rtc_now_us - flight_checkpoint.rtc_time_us;
    if (UINT64_MAX - elapsed < reset_elapsed_us)
      return false;
    elapsed += reset_elapsed_us;
  }
  checkpoint.valid = true;
  checkpoint.state = static_cast<protocol::MissionState>(flight_checkpoint.state);
  checkpoint.flight_epoch = flight_checkpoint.flight_epoch;
  checkpoint.elapsed_valid = flight_checkpoint.elapsed_valid;
  checkpoint.elapsed_us = elapsed;
  checkpoint.deployment_started = flight_checkpoint.deployment_started;
  checkpoint.power_cutoff_latched = flight_checkpoint.power_cutoff_latched;
  checkpoint.forced_start = flight_checkpoint.forced_start;
  checkpoint.preflight_generation = flight_checkpoint.preflight_generation;
  checkpoint.preflight_captured_at_us = flight_checkpoint.preflight_captured_at_us;
  checkpoint.preflight_ready_mask = flight_checkpoint.preflight_ready_mask;
  checkpoint.preflight_missing_mask = flight_checkpoint.preflight_missing_mask;
  return true;
}

void storeFlightCheckpoint(const mission::MissionSnapshot &snapshot) {
  if (snapshot.flight_epoch == 0 ||
      snapshot.state == protocol::MissionState::command_receive) {
    clearFlightCheckpoint();
    return;
  }
  const bool pre_liftoff =
      snapshot.state == protocol::MissionState::liftoff_detection;
  if (!pre_liftoff && !snapshot.liftoff_time_valid) {
    clearFlightCheckpoint();
    return;
  }
  FlightCheckpointRecord record{};
  record.magic = kFlightCheckpointMagic;
  record.version = kFlightCheckpointVersion;
  record.flight_epoch = snapshot.flight_epoch;
  record.elapsed_us = pre_liftoff ? 0 : snapshot.elapsed_us;
  record.rtc_time_us = esp_rtc_get_time_us();
  record.preflight_captured_at_us = snapshot.preflight_captured_at_us;
  record.preflight_generation = snapshot.preflight_generation;
  record.state = static_cast<uint8_t>(snapshot.state);
  record.preflight_ready_mask = snapshot.preflight_ready_mask;
  record.preflight_missing_mask = snapshot.preflight_missing_mask;
  record.elapsed_valid = !pre_liftoff;
  record.deployment_started = snapshot.deployment_started;
  record.power_cutoff_latched = snapshot.deployment_power_cutoff_latched;
  record.forced_start = snapshot.forced_start;
  record.checksum = checkpointChecksum(record);
  flight_checkpoint = record;
}
void clearFlightCheckpoint() { flight_checkpoint = {}; }

bool loadFlightParachuteConfiguration(
    actuators::FlightParachuteConfiguration &configuration) {
  configuration = {};
  if (!resetPreservesRtcMemory() ||
      parachute_checkpoint.magic != kParachuteCheckpointMagic ||
      parachute_checkpoint.version != kParachuteCheckpointVersion ||
      (parachute_checkpoint.valid_mask & ~(kOpenValid | kCloseValid)) != 0 ||
      parachute_checkpoint.checksum !=
          parachuteCheckpointChecksum(parachute_checkpoint))
    return false;
  if ((parachute_checkpoint.valid_mask & kOpenValid) != 0) {
    configuration.open = actuators::AbsoluteParachuteAngle::fromCount(
        parachute_checkpoint.open_count);
    if (!configuration.open.has_value())
      return false;
  }
  if ((parachute_checkpoint.valid_mask & kCloseValid) != 0) {
    configuration.close = actuators::AbsoluteParachuteAngle::fromCount(
        parachute_checkpoint.close_count);
    if (!configuration.close.has_value())
      return false;
  }
  return true;
}

void storeFlightParachuteConfiguration(
    const actuators::FlightParachuteConfiguration &configuration) {
  ParachuteCheckpointRecord record{};
  record.magic = kParachuteCheckpointMagic;
  record.version = kParachuteCheckpointVersion;
  if (configuration.open.has_value()) {
    record.valid_mask |= kOpenValid;
    record.open_count = configuration.open->count();
  }
  if (configuration.close.has_value()) {
    record.valid_mask |= kCloseValid;
    record.close_count = configuration.close->count();
  }
  record.checksum = parachuteCheckpointChecksum(record);
  parachute_checkpoint = record;
}
void clearFlightParachuteConfiguration() { parachute_checkpoint = {}; }

[[noreturn]] void enterPeriodicDeepSleep() {
  prepareMarker();
  (void)esp_sleep_enable_timer_wakeup(mission::RecoveryRuntime::periodicWakeUs());
  esp_deep_sleep_start();
}

} // 名前空間 runtime::recovery_boot
''')

# MotorProfileValidをreadiness gateへ公開する。
replace_once(
    "src/config/flight_config.hpp",
    "[[nodiscard]] inline bool productionFlightConfigurationReady() {\n  return board::kFlightMotorA.parameters_valid &&\n         board::kFlightMotorA.polarity !=\n             board::MotorPolarity::unconfigured &&",
    "[[nodiscard]] inline bool motorProfileValid() {\n  return board::kFlightMotorA.parameters_valid &&\n         board::kFlightMotorA.polarity != board::MotorPolarity::unconfigured;\n}\n\n[[nodiscard]] inline bool productionFlightConfigurationReady() {\n  return motorProfileValid() &&"
)

# wire layoutを変えず、MissionEvent bit15をdeployment failureへ割り当てる。
replace_once(
    "src/protocol/can_protocol.hpp",
    "  fin_control_disabled_by_ground = 1U << 14U,\n};",
    "  fin_control_disabled_by_ground = 1U << 14U,\n  parachute_deployment_failure = 1U << 15U,\n};"
)

# production runtimeのrequestへ同一readiness snapshotを持たせる。
replace_once(
    "src/runtime/production_runtime.cpp",
    "struct MissionCommandEnvelope {\n  protocol::GenericCommandRequest request{};\n  mission::CommandDecision decision{};\n};",
    "struct MissionCommandEnvelope {\n  protocol::GenericCommandRequest request{};\n  mission::CommandDecision decision{};\n  mission::PreflightReadinessSnapshot readiness{};\n};"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "struct ParachuteCommandRequest {\n  enum class Kind : uint8_t { generic, start_preparation } kind{Kind::generic};\n  protocol::GenericCommandRequest command{};\n};",
    "struct ParachuteCommandRequest {\n  enum class Kind : uint8_t { generic, start_preparation } kind{Kind::generic};\n  protocol::GenericCommandRequest command{};\n  mission::PreflightReadinessSnapshot readiness{};\n};"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "std::atomic<bool> parachute_persistence_ready{false};\nstd::atomic<bool> parachute_open_configured{false};",
    "std::atomic<bool> parachute_persistence_ready{false};\nstd::atomic<bool> parachute_persistence_corrupt{false};\nstd::atomic<uint8_t> parachute_deployment_failure{};\nstd::atomic<bool> parachute_open_configured{false};"
)

# deployment failure code。0は未記録で、最初のfailureだけを保持する。
replace_once(
    "src/runtime/production_runtime.cpp",
    "protocol::CommandReason transitionReason(mission::TransitionResult result);",
    "enum class ParachuteDeploymentFailure : uint8_t {\n  none = 0,\n  open_not_configured = 1,\n  current_angle_unavailable = 2,\n  ambiguous_half_turn = 3,\n  move_command_failed = 4,\n  retry_exhausted = 5,\n  hold_failed = 6,\n  persistence_corrupt = 7,\n};\n\nprotocol::CommandReason transitionReason(mission::TransitionResult result);"
)

# RTC optional snapshot復元。
replace_once(
    "src/runtime/production_runtime.cpp",
    "    actuators::FlightParachuteConfiguration restored{\n        *actuators::AbsoluteParachuteAngle::fromCount(0),\n        *actuators::AbsoluteParachuteAngle::fromCount(0)};",
    "    actuators::FlightParachuteConfiguration restored{};"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "  enum class DesiredState : uint8_t { powered_off, holding, open };\n  DesiredState desired = DesiredState::powered_off;",
    "  enum class DesiredState : uint8_t { powered_off, free, holding, open };\n  // boot直後のGPIO安全化後はCommandReceiveからHold要求を維持する。\n  DesiredState desired = DesiredState::holding;"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "  bool controller_started = false;\n  esp_err_t last_initialization_error = ESP_ERR_INVALID_STATE;",
    "  bool controller_started = false;\n  bool hold_established = false;\n  esp_err_t last_initialization_error = ESP_ERR_INVALID_STATE;"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "  constexpr uint32_t kDetailQueueUnavailable = 4;",
    "  constexpr uint32_t kDetailQueueUnavailable = 4;\n  constexpr uint32_t kDetailCurrentOpenHalfTurn = 5;"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "    controller_started = false;\n    last_initialization_error = ESP_ERR_INVALID_STATE;",
    "    controller_started = false;\n    hold_established = false;\n    last_initialization_error = ESP_ERR_INVALID_STATE;"
)

# persistence corruptをHealth/eventへ保持する。
replace_once(
    "src/runtime/production_runtime.cpp",
    "        parachute_persistence_ready.store(\n            persistence_response.persistence_ready,\n            std::memory_order_release);\n        parachute_config_load_complete.store(true, std::memory_order_release);",
    "        parachute_persistence_ready.store(\n            persistence_response.persistence_ready,\n            std::memory_order_release);\n        parachute_persistence_corrupt.store(\n            persistence_response.corruption_detected, std::memory_order_release);\n        if (persistence_response.corruption_detected)\n          parachute_deployment_failure.compare_exchange_strong(\n              *new uint8_t{0},\n              static_cast<uint8_t>(ParachuteDeploymentFailure::persistence_corrupt));\n        parachute_config_load_complete.store(true, std::memory_order_release);"
)

# 上のnewは使わず安全なcompare_exchangeへ直す。
p = ROOT / "src/runtime/production_runtime.cpp"
text = p.read_text(encoding="utf-8")
text = text.replace(
    "          parachute_deployment_failure.compare_exchange_strong(\n              *new uint8_t{0},\n              static_cast<uint8_t>(ParachuteDeploymentFailure::persistence_corrupt));",
    "          {\n            uint8_t expected = 0;\n            (void)parachute_deployment_failure.compare_exchange_strong(\n                expected, static_cast<uint8_t>(\n                              ParachuteDeploymentFailure::persistence_corrupt));\n          }"
)
p.write_text(text, encoding="utf-8")

# discardはCancel時snapshotだけを捨て、電源を落とさずHoldへ戻す。
replace_once(
    "src/runtime/production_runtime.cpp",
    "          if (request.kind == ParaRequest::Kind::discard_snapshot) {\n            configuration.discardFlightSnapshot();\n            recovery_boot::clearFlightParachuteConfiguration();\n          }\n          const bool free_requested = request.kind == ParaRequest::Kind::free;",
    "          if (request.kind == ParaRequest::Kind::discard_snapshot) {\n            configuration.discardFlightSnapshot();\n            recovery_boot::clearFlightParachuteConfiguration();\n            desired = DesiredState::holding;\n            hold_established = false;\n            continue;\n          }\n          const bool free_requested = request.kind == ParaRequest::Kind::free;"
)

# flight snapshotが存在してもOpenはoptional。missingで電源OFFしない。
replace_once(
    "src/runtime/production_runtime.cpp",
    "      if (!configuration.flightSnapshotValid()) {\n        std::printf(\"parachute open rejected: flight snapshot unavailable\\n\");\n        powerOff(false, true, protocol::ParaMode::powered_off);\n        continue;\n      }",
    "      if (!configuration.flightSnapshotValid()) {\n        std::printf(\"parachute open failed: flight snapshot unavailable\\n\");\n        uint8_t expected = 0;\n        (void)parachute_deployment_failure.compare_exchange_strong(\n            expected, static_cast<uint8_t>(\n                          ParachuteDeploymentFailure::open_not_configured));\n        desired = DesiredState::holding;\n        hold_established = false;\n        continue;\n      }"
)

# ParaFreeは通常commandではtorque OFFのみ。powerは維持する。
replace_once(
    "src/runtime/production_runtime.cpp",
    "        if (command_request.kind == ParachuteCommandRequest::Kind::generic &&\n            code == mission::CommandCode::para_free) {\n          powerOff(false, true, protocol::ParaMode::free);\n          requestFinish(protocol::CommandReason::none);",
    "        if (command_request.kind == ParachuteCommandRequest::Kind::generic &&\n            code == mission::CommandCode::para_free) {\n          desired = DesiredState::free;\n          hold_established = false;\n          if (servo.initialized())\n            (void)servo.disableTorque();\n          para_mode_actual.store(protocol::ParaMode::free,\n                                 std::memory_order_release);\n          requestFinish(protocol::CommandReason::none);"
)

# start preparationをStage 2へ置換。
old_start = r'''        if (read != ESP_OK) {
          requestFinish(commandReasonForSts(read), kDetailInvalidPosition);
        } else if (pending.request.kind ==
                   ParachuteCommandRequest::Kind::start_preparation) {
          const esp_err_t hold = servo.holdCurrentPosition(
              {STS3215::TorqueLimit::percent(
                  flight_config::kParachute.torque_limit_percent)});
          sts_ready.store(hold == ESP_OK, std::memory_order_release);
          if (hold != ESP_OK) {
            requestFinish(commandReasonForSts(hold));
          } else {
            desired = DesiredState::holding;
            para_mode_actual.store(protocol::ParaMode::hold,
                                   std::memory_order_release);
          }
          if (hold == ESP_OK &&
              !parachute_config_load_complete.load(
                         std::memory_order_acquire)) {
            requestFinish(protocol::CommandReason::busy,
                          kDetailConfigurationLoad);
          } else if (hold == ESP_OK &&
                     !parachute_persistence_ready.load(
                         std::memory_order_acquire)) {
            requestFinish(protocol::CommandReason::persistence_error,
                          kDetailConfigurationLoad);
          } else if (hold == ESP_OK) {
            const auto prepared = configuration.freezeFlightSnapshot(current);
            if (!prepared.ready()) {
              requestFinish(protocol::CommandReason::not_configured,
                            static_cast<uint32_t>(prepared.error));
            } else {
              recovery_boot::storeFlightParachuteConfiguration(
                  *configuration.flightSnapshot());
              requestFinish(protocol::CommandReason::none);
            }
          }
        } else {'''
new_start = r'''        if (pending.request.kind ==
            ParachuteCommandRequest::Kind::start_preparation) {
          const auto code = static_cast<mission::CommandCode>(
              pending.request.command.command);
          const bool forced =
              code == mission::CommandCode::force_start_sequence;
          if (!parachute_config_load_complete.load(
                  std::memory_order_acquire)) {
            requestFinish(protocol::CommandReason::busy,
                          kDetailConfigurationLoad);
          } else if (!parachute_persistence_ready.load(
                         std::memory_order_acquire)) {
            requestFinish(protocol::CommandReason::persistence_error,
                          kDetailConfigurationLoad);
          } else if (!forced && pending.request.readiness.missingMask() != 0) {
            requestFinish(protocol::CommandReason::not_configured,
                          pending.request.readiness.missingMask());
          } else if (forced) {
            // ForceではSTS失敗をvalidへ偽装せず、snapshotだけはoptionalのままfreezeする。
            if (read == ESP_OK) {
              const esp_err_t hold = servo.holdCurrentPosition(
                  {STS3215::TorqueLimit::percent(
                      flight_config::kParachute.torque_limit_percent)});
              sts_ready.store(hold == ESP_OK, std::memory_order_release);
              hold_established = hold == ESP_OK;
              if (hold == ESP_OK) {
                desired = DesiredState::holding;
                para_mode_actual.store(protocol::ParaMode::hold,
                                       std::memory_order_release);
              }
            } else {
              sts_ready.store(false, std::memory_order_release);
              hold_established = false;
              desired = DesiredState::holding;
            }
            configuration.freezeFlightSnapshotForced();
            recovery_boot::storeFlightParachuteConfiguration(
                *configuration.flightSnapshot());
            requestFinish(protocol::CommandReason::none,
                          pending.request.readiness.missingMask());
          } else if (read != ESP_OK) {
            requestFinish(commandReasonForSts(read), kDetailInvalidPosition);
          } else {
            const esp_err_t hold = servo.holdCurrentPosition(
                {STS3215::TorqueLimit::percent(
                    flight_config::kParachute.torque_limit_percent)});
            sts_ready.store(hold == ESP_OK, std::memory_order_release);
            if (hold != ESP_OK) {
              requestFinish(commandReasonForSts(hold));
            } else {
              desired = DesiredState::holding;
              hold_established = true;
              para_mode_actual.store(protocol::ParaMode::hold,
                                     std::memory_order_release);
              const auto prepared = configuration.freezeFlightSnapshot(current);
              if (!prepared.ready()) {
                if (prepared.error ==
                    actuators::FlightParachutePreparationError::
                        current_open_exactly_half_turn)
                  requestFinish(protocol::CommandReason::safety_interlock,
                                kDetailCurrentOpenHalfTurn);
                else
                  requestFinish(protocol::CommandReason::not_configured,
                                pending.request.readiness.missingMask());
              } else {
                recovery_boot::storeFlightParachuteConfiguration(
                    *configuration.flightSnapshot());
                requestFinish(protocol::CommandReason::none);
              }
            }
          }
        } else if (read != ESP_OK) {
          requestFinish(commandReasonForSts(read), kDetailInvalidPosition);
        } else {'''
replace_once("src/runtime/production_runtime.cpp", old_start, new_start)

# initialize timeoutはForce startだけfailureにしない。Hold maintenanceへ戻す。
replace_once(
    "src/runtime/production_runtime.cpp",
    "      } else if (now_us >= pending.started_at_us &&\n                 now_us - pending.started_at_us >=\n                     static_cast<uint64_t>(\n                         flight_config::kParachute.initialization_deadline_ms) *\n                         1'000ULL) {\n        requestFinish(commandReasonForSts(last_initialization_error));\n      }",
    "      } else if (now_us >= pending.started_at_us &&\n                 now_us - pending.started_at_us >=\n                     static_cast<uint64_t>(\n                         flight_config::kParachute.initialization_deadline_ms) *\n                         1'000ULL) {\n        const bool forced_start =\n            pending.request.kind == ParachuteCommandRequest::Kind::start_preparation &&\n            static_cast<mission::CommandCode>(pending.request.command.command) ==\n                mission::CommandCode::force_start_sequence;\n        if (forced_start &&\n            parachute_config_load_complete.load(std::memory_order_acquire) &&\n            parachute_persistence_ready.load(std::memory_order_acquire)) {\n          configuration.freezeFlightSnapshotForced();\n          recovery_boot::storeFlightParachuteConfiguration(\n              *configuration.flightSnapshot());\n          desired = DesiredState::holding;\n          hold_established = false;\n          requestFinish(protocol::CommandReason::none,\n                        pending.request.readiness.missingMask());\n        } else {\n          requestFinish(commandReasonForSts(last_initialization_error));\n        }\n      }"
)

# generic move完了Holdを追跡。
replace_once(
    "src/runtime/production_runtime.cpp",
    "            desired = DesiredState::holding;\n            para_mode_actual.store(protocol::ParaMode::hold,",
    "            desired = DesiredState::holding;\n            hold_established = true;\n            para_mode_actual.store(protocol::ParaMode::hold,"
)

# Open loop全体をStage 2へ差し替える。
start_marker = "    if (desired == DesiredState::open && !pending.active) {\n"
end_marker = "\n    finishPending();"
p = ROOT / "src/runtime/production_runtime.cpp"
text = p.read_text(encoding="utf-8")
start = text.find(start_marker)
end = text.find(end_marker, start)
if start < 0 or end < 0:
    raise RuntimeError("open loop markers not found")
new_open = r'''    auto recordParachuteFailure = [&](ParachuteDeploymentFailure failure,
                                      uint16_t detail = 0) {
      uint8_t expected = 0;
      if (parachute_deployment_failure.compare_exchange_strong(
              expected, static_cast<uint8_t>(failure))) {
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::parachute_deployment_failure),
                     protocol::MissionState::descent, 0,
                     detail != 0 ? detail : static_cast<uint16_t>(failure));
      }
    };

    // Hold要求中はpower-on/reconnectを継続し、fresh currentを取得できた時点でHoldする。
    if (desired == DesiredState::holding && !pending.active) {
      if (!requestPower(now_us)) {
        sts_ready.store(false, std::memory_order_release);
        hold_established = false;
      } else if (!hold_established && initializeServo(now_us)) {
        actuators::AbsoluteParachuteAngle current =
            *actuators::AbsoluteParachuteAngle::fromCount(0);
        bool moving = false;
        const esp_err_t read = readCurrent(current, moving);
        sts_ready.store(read == ESP_OK, std::memory_order_release);
        if (read == ESP_OK) {
          const esp_err_t hold = servo.holdCurrentPosition(
              {STS3215::TorqueLimit::percent(
                  flight_config::kParachute.torque_limit_percent)});
          sts_ready.store(hold == ESP_OK, std::memory_order_release);
          hold_established = hold == ESP_OK;
          if (hold_established)
            para_mode_actual.store(protocol::ParaMode::hold,
                                   std::memory_order_release);
          else
            recordParachuteFailure(ParachuteDeploymentFailure::hold_failed);
        }
      }
    }

    if (desired == DesiredState::free && !pending.active) {
      if (requestPower(now_us) && initializeServo(now_us)) {
        (void)servo.disableTorque();
        para_mode_actual.store(protocol::ParaMode::free,
                               std::memory_order_release);
      }
    }

    if (desired == DesiredState::open && !pending.active) {
      const auto *snapshot = configuration.flightSnapshot();
      if (snapshot == nullptr || !snapshot->open.has_value()) {
        recordParachuteFailure(ParachuteDeploymentFailure::open_not_configured);
        desired = DesiredState::holding;
        hold_established = false;
      } else if (!requestPower(now_us)) {
        sts_ready.store(false, std::memory_order_release);
      } else if (initializeServo(now_us)) {
        auto current = *actuators::AbsoluteParachuteAngle::fromCount(0);
        bool moving = false;
        const esp_err_t read = readCurrent(current, moving);
        sts_ready.store(read == ESP_OK, std::memory_order_release);
        if (read == ESP_OK) {
          const auto displacement = actuators::shortestParachuteDisplacement(
              current, *snapshot->open);
          if (!displacement.valid()) {
            recordParachuteFailure(
                ParachuteDeploymentFailure::ambiguous_half_turn);
            desired = DesiredState::holding;
            hold_established = false;
          } else if (!controller_started) {
            controller_started =
                controller.startOpen(open_requested_at_us, current.count()) ==
                actuators::ParachuteAction::command_open;
            if (controller_started &&
                moveToAbsolute(current, *snapshot->open) != ESP_OK) {
              sts_ready.store(false, std::memory_order_release);
              recordParachuteFailure(
                  ParachuteDeploymentFailure::move_command_failed);
            }
          }

          if (controller_started && desired == DesiredState::open) {
            const auto latest_displacement =
                actuators::shortestParachuteDisplacement(current,
                                                         *snapshot->open);
            const bool reached = latest_displacement.valid() && !moving &&
                                 latest_displacement.counts <=
                                     target_tolerance_count &&
                                 latest_displacement.counts >=
                                     -target_tolerance_count;
            const auto action = controller.tick(
                {now_us, true, current.count(), reached});
            if (action == actuators::ParachuteAction::retry_open) {
              const auto retry_displacement =
                  actuators::shortestParachuteDisplacement(current,
                                                           *snapshot->open);
              if (!retry_displacement.valid()) {
                recordParachuteFailure(
                    ParachuteDeploymentFailure::ambiguous_half_turn);
                desired = DesiredState::holding;
                hold_established = false;
              } else if (moveToAbsolute(current, *snapshot->open) != ESP_OK) {
                sts_ready.store(false, std::memory_order_release);
                recordParachuteFailure(
                    ParachuteDeploymentFailure::move_command_failed);
              }
            } else if (action == actuators::ParachuteAction::hold_open) {
              const esp_err_t hold = servo.holdCurrentPosition(
                  {STS3215::TorqueLimit::percent(
                      flight_config::kParachute.torque_limit_percent)});
              hold_established = hold == ESP_OK;
              if (!hold_established)
                recordParachuteFailure(
                    ParachuteDeploymentFailure::hold_failed);
              desired = DesiredState::holding;
              para_mode_actual.store(protocol::ParaMode::hold,
                                     std::memory_order_release);
            } else if (action ==
                       actuators::ParachuteAction::stop_retrying) {
              recordParachuteFailure(
                  ParachuteDeploymentFailure::retry_exhausted);
              const esp_err_t hold = servo.holdCurrentPosition(
                  {STS3215::TorqueLimit::percent(
                      flight_config::kParachute.torque_limit_percent)});
              hold_established = hold == ESP_OK;
              if (!hold_established)
                recordParachuteFailure(
                    ParachuteDeploymentFailure::hold_failed);
              desired = DesiredState::holding;
              para_mode_actual.store(protocol::ParaMode::hold,
                                     std::memory_order_release);
            }
          }
        }
      }

      // 5秒はOpen retryだけを終了する。電源は+25秒cutoffまで維持する。
      if (desired == DesiredState::open && open_requested_at_us != 0 &&
          now_us >= open_requested_at_us &&
          now_us - open_requested_at_us >= 5'000'000) {
        recordParachuteFailure(
            sts_ready.load(std::memory_order_acquire)
                ? ParachuteDeploymentFailure::retry_exhausted
                : ParachuteDeploymentFailure::current_angle_unavailable);
        desired = DesiredState::holding;
        hold_established = false;
      }
    }
'''
text = text[:start] + new_open + text[end:]
p.write_text(text, encoding="utf-8")

# MissionRealtimeにpreflight calibration stateを追加。
replace_once(
    "src/runtime/production_runtime.cpp",
    "  MissionCommandEnvelope pending_start{};\n  bool start_preparation_pending = false;",
    "  MissionCommandEnvelope pending_start{};\n  bool start_preparation_pending = false;\n  uint32_t preflight_generation = 0;\n  bool preflight_gyro_bias_valid = false;\n  bool gravity_reference_valid = false;\n  double preflight_gyro_bias_rad_s = 0.0;\n  struct CalibrationState {\n    bool active{};\n    uint8_t transaction_id{};\n    uint64_t started_at_us{};\n    uint32_t gyro_samples{};\n    uint32_t accel_samples{};\n    double gyro_sum_rad_s{};\n    double accel_sum_x_g{};\n    double accel_sum_y_g{};\n    double accel_sum_z_g{};\n  } calibration;"
)

# sensor sampleをcalibrationへ集計。
replace_once(
    "src/runtime/production_runtime.cpp",
    "            gyro_history.push(gyro);\n            if (gyro.valid && !gyro.format_fault)",
    "            gyro_history.push(gyro);\n            if (calibration.active && gyro.valid && !gyro.format_fault &&\n                std::isfinite(gyro.roll_rate_rad_s)) {\n              calibration.gyro_sum_rad_s += gyro.roll_rate_rad_s;\n              ++calibration.gyro_samples;\n            }\n            if (calibration.active && sample.acceleration_valid &&\n                std::isfinite(sample.acceleration_g[0]) &&\n                std::isfinite(sample.acceleration_g[1]) &&\n                std::isfinite(sample.acceleration_g[2])) {\n              calibration.accel_sum_x_g += sample.acceleration_g[0];\n              calibration.accel_sum_y_g += sample.acceleration_g[1];\n              calibration.accel_sum_z_g += sample.acceleration_g[2];\n              ++calibration.accel_samples;\n            }\n            if (gyro.valid && !gyro.format_fault)"
)

# start responseは同一readinessでFSMへ遷移し、Force Completed.detailを保持する。
old_response = r'''      protocol::CommandReason reason = start_response.reason;
      if (reason == protocol::CommandReason::none) {
        const mission::SequenceConfiguration configuration{
            fin_zero_configured.load(std::memory_order_acquire),
            parachute_open_configured.load(std::memory_order_acquire),
            parachute_close_configured.load(std::memory_order_acquire), true};
        const auto transition = state_machine.startSequence(
            static_cast<uint64_t>(esp_timer_get_time()), configuration);
        reason = transitionReason(transition);
        if (transition == mission::TransitionResult::completed) {
          actuator_output_inhibited.store(false, std::memory_order_release);
        } else {
          const ParaRequest discard{ParaRequest::Kind::discard_snapshot, 0,
                                    false};
          (void)xQueueSendToFront(para_queue, &discard, 0);
        }
      } else {
        const ParaRequest discard{ParaRequest::Kind::discard_snapshot, 0,
                                  false};
        (void)xQueueSendToFront(para_queue, &discard, 0);
      }
      xSemaphoreGive(state_mutex);
      const auto final = command_executor.finish(
          pending_start.request.transaction_id,
          reason == protocol::CommandReason::none
              ? protocol::CommandPhase::completed
              : protocol::CommandPhase::failed,
          reason, start_response.detail);'''
new_response = r'''      protocol::CommandReason reason = start_response.reason;
      uint32_t final_detail = start_response.detail;
      if (reason == protocol::CommandReason::none) {
        const bool forced =
            static_cast<mission::CommandCode>(pending_start.request.command) ==
            mission::CommandCode::force_start_sequence;
        const auto transition = state_machine.startSequence(
            static_cast<uint64_t>(esp_timer_get_time()),
            pending_start.readiness,
            forced ? mission::StartMode::forced : mission::StartMode::normal);
        reason = transitionReason(transition);
        if (transition == mission::TransitionResult::completed) {
          actuator_output_inhibited.store(false, std::memory_order_release);
          if (forced)
            final_detail = pending_start.readiness.missingMask();
          recovery_boot::storeFlightCheckpoint(state_machine.snapshot());
        } else {
          const ParaRequest discard{ParaRequest::Kind::discard_snapshot, 0,
                                    false};
          (void)xQueueSendToFront(para_queue, &discard, 0);
        }
      } else {
        const ParaRequest discard{ParaRequest::Kind::discard_snapshot, 0,
                                  false};
        (void)xQueueSendToFront(para_queue, &discard, 0);
      }
      xSemaphoreGive(state_mutex);
      const auto final = command_executor.finish(
          pending_start.request.transaction_id,
          reason == protocol::CommandReason::none
              ? protocol::CommandPhase::completed
              : protocol::CommandPhase::failed,
          reason, final_detail);'''
replace_once("src/runtime/production_runtime.cpp", old_response, new_response)

# transition queue: Start/Forceはreadiness capture、Calibrationは3秒windowを開始。
old_transition = r'''      } else if (code == mission::CommandCode::start_sequence) {
        const ParachuteCommandRequest preparation{
            ParachuteCommandRequest::Kind::start_preparation,
            command_envelope.request};
        if (!start_preparation_pending &&
            xQueueSend(parachute_command_queue, &preparation, 0) == pdTRUE) {
          pending_start = command_envelope;
          start_preparation_pending = true;
          asynchronous_transition = true;
        } else {
          direct_reason = protocol::CommandReason::busy;
        }
      }'''
new_transition = r'''      } else if (code == mission::CommandCode::start_sequence ||
                 code == mission::CommandCode::force_start_sequence) {
        mission::PreflightReadinessSnapshot readiness{};
        ++preflight_generation;
        if (preflight_generation == 0)
          ++preflight_generation;
        readiness.generation = preflight_generation;
        readiness.captured_at_us =
            static_cast<uint64_t>(esp_timer_get_time());
        readiness.fin_zero_configured =
            fin_zero_configured.load(std::memory_order_acquire);
        readiness.parachute_open_configured =
            parachute_open_configured.load(std::memory_order_acquire);
        readiness.parachute_close_configured =
            parachute_close_configured.load(std::memory_order_acquire);
        readiness.motor_profile_valid = flight_config::motorProfileValid();
        readiness.gyro_bias_valid = preflight_gyro_bias_valid;
        readiness.gravity_reference_valid = gravity_reference_valid;
        readiness.ssc_zero_valid = latest_air_data.ssc_zero_valid;
        readiness.resources_preallocated = true;
        command_envelope.readiness = readiness;
        const ParachuteCommandRequest preparation{
            ParachuteCommandRequest::Kind::start_preparation,
            command_envelope.request, readiness};
        if (!start_preparation_pending &&
            xQueueSend(parachute_command_queue, &preparation, 0) == pdTRUE) {
          pending_start = command_envelope;
          start_preparation_pending = true;
          asynchronous_transition = true;
        } else {
          direct_reason = protocol::CommandReason::busy;
        }
      } else if (code == mission::CommandCode::run_preflight_calibration) {
        if (!calibration.active) {
          calibration = {};
          calibration.active = true;
          calibration.transaction_id = command_envelope.request.transaction_id;
          calibration.started_at_us =
              static_cast<uint64_t>(esp_timer_get_time());
          // 最新attemptだけを有効にする。途中失敗時に古い値へrollbackしない。
          preflight_gyro_bias_valid = false;
          gravity_reference_valid = false;
          asynchronous_transition = true;
        } else {
          direct_reason = protocol::CommandReason::busy;
        }
      }'''
replace_once("src/runtime/production_runtime.cpp", old_transition, new_transition)

# calibration終了処理をsensor取得後に追加。
replace_once(
    "src/runtime/production_runtime.cpp",
    "    const uint64_t imu_now_us = static_cast<uint64_t>(esp_timer_get_time());",
    "    const uint64_t imu_now_us = static_cast<uint64_t>(esp_timer_get_time());\n    if (calibration.active && imu_now_us >= calibration.started_at_us &&\n        imu_now_us - calibration.started_at_us >= 3'000'000) {\n      // TODO(HW_TEST): sample数・静置判定・gravity norm閾値を実機で確定する。\n      const bool gyro_samples_ok = calibration.gyro_samples >= 1'500;\n      const bool accel_samples_ok = calibration.accel_samples >= 1'500;\n      if (gyro_samples_ok) {\n        preflight_gyro_bias_rad_s =\n            calibration.gyro_sum_rad_s / calibration.gyro_samples;\n        preflight_gyro_bias_valid =\n            std::isfinite(preflight_gyro_bias_rad_s) &&\n            std::abs(preflight_gyro_bias_rad_s) <=\n                5.0 * 0.017453292519943295;\n      }\n      if (accel_samples_ok) {\n        const double x = calibration.accel_sum_x_g / calibration.accel_samples;\n        const double y = calibration.accel_sum_y_g / calibration.accel_samples;\n        const double z = calibration.accel_sum_z_g / calibration.accel_samples;\n        const double norm = std::sqrt(x * x + y * y + z * z);\n        gravity_reference_valid = std::isfinite(norm) && norm >= 0.8 && norm <= 1.2;\n      }\n      uint32_t detail = 0;\n      if (!preflight_gyro_bias_valid)\n        detail |= 1U << 4U;\n      if (!gravity_reference_valid)\n        detail |= 1U << 5U;\n      if (!latest_air_data.ssc_zero_valid)\n        detail |= 1U << 6U;\n      if (xSemaphoreTake(executor_mutex, 0) == pdTRUE) {\n        const auto result = command_executor.finish(\n            calibration.transaction_id, protocol::CommandPhase::completed,\n            protocol::CommandReason::none, detail);\n        xSemaphoreGive(executor_mutex);\n        enqueueResult(result, false);\n        calibration = {};\n      }\n    }"
)

# liftoff attitudeは最新PreflightCalibrationのbiasだけを使用する。Forceでinvalidなら姿勢はinvalidのまま。
old_att = r'''    if (mission_snapshot.liftoff_time_valid &&
        attitude_epoch != mission_snapshot.flight_epoch) {
      double gyro_bias = 0.0;
      if (estimatePreflightGyroBias(gyro_history,
                                   mission_snapshot.liftoff_time_us,
                                   gyro_bias) &&
          attitude.beginFlight(gyro_history, mission_snapshot.liftoff_time_us,
                               gyro_bias))
        attitude_epoch = mission_snapshot.flight_epoch;
      else
        attitude_epoch = 0;
    }'''
new_att = r'''    if (mission_snapshot.liftoff_time_valid &&
        attitude_epoch != mission_snapshot.flight_epoch) {
      if (preflight_gyro_bias_valid &&
          attitude.beginFlight(gyro_history, mission_snapshot.liftoff_time_us,
                               preflight_gyro_bias_rad_s))
        attitude_epoch = mission_snapshot.flight_epoch;
      else
        attitude_epoch = 0;
    }'''
replace_once("src/runtime/production_runtime.cpp", old_att, new_att)

# command contextのruntime invariant/persistence/calibration support。
replace_once(
    "src/runtime/production_runtime.cpp",
    "      context.sequence_configured =\n          flight_config::productionFlightConfigurationReady();\n      context.resources_preallocated = true;",
    "      context.resources_preallocated = true;\n      context.persistence_load_complete =\n          parachute_config_load_complete.load(std::memory_order_acquire);\n      context.persistence_runtime_available =\n          parachute_persistence_ready.load(std::memory_order_acquire);"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "      context.fin_safe_commands_supported = false;",
    "      context.fin_safe_commands_supported = false;\n      context.calibration_supported = true;"
)
replace_once(
    "src/runtime/production_runtime.cpp",
    "        parachute_command = {ParachuteCommandRequest::Kind::generic,\n                             envelope.request};",
    "        parachute_command = {ParachuteCommandRequest::Kind::generic,\n                             envelope.request, {}};"
)

# TransitionResult追加値。
replace_once(
    "src/runtime/production_runtime.cpp",
    "  case mission::TransitionResult::not_configured:\n    return protocol::CommandReason::not_configured;\n  }",
    "  case mission::TransitionResult::not_configured:\n    return protocol::CommandReason::not_configured;\n  case mission::TransitionResult::runtime_unavailable:\n    return protocol::CommandReason::internal_error;\n  }"
)

# Descent statusの予約領域へ最初のdeployment failure codeを載せる。
replace_once(
    "src/runtime/production_runtime.cpp",
    "          const protocol::DescentCoreTelemetry descent{\n              sequences.next(protocol::CanId::descent_core_telemetry),\n              0x1FFF, static_cast<uint8_t>(\n                          protocol::quantization::ParachuteAngleError::unavailable)};",
    "          const uint16_t failure = static_cast<uint16_t>(\n              parachute_deployment_failure.load(std::memory_order_acquire) & 0x0FU);\n          const protocol::DescentCoreTelemetry descent{\n              sequences.next(protocol::CanId::descent_core_telemetry),\n              static_cast<uint16_t>(0x1F00U | failure),\n              static_cast<uint8_t>(\n                  protocol::quantization::ParachuteAngleError::unavailable)};"
)

# READMEへStage 2運用の要点を追記する。
p = ROOT / "README.md"
readme = p.read_text(encoding="utf-8")
if "## ForceStartSequence / Parachute Stage 2" not in readme:
    readme += r'''

## ForceStartSequence / Parachute Stage 2

- `ForceStartSequence`はcommand code `0x04`で、通常Startの7項目preflight missing gateだけをbypassします。
- Open/Closeは独立optionalの1回転絶対角としてflight snapshotへfreezeし、Open/Close相互のhalf-turnはStart拒否理由にしません。
- deploymentでは毎回fresh currentからsnapshot targetへのshortest pathを計算し、exact half-turnは動かしません。
- ForceStartではSTS unavailable/read/Hold failureだけでLiftoffDetection遷移を失敗させず、Healthを正常へ偽装しません。
- Open retryの約5秒deadlineはretry終了期限であり電源遮断期限ではありません。Hold/reconnectとパラシュート電源は離床+25秒まで維持し、+25秒で絶対cutoffします。
- `forced_start`とpreflight snapshot/missing mask、optional parachute snapshotはsoftware/watchdog reset時にRTC checkpointから復元します。
'''
    p.write_text(readme, encoding="utf-8")

print("ForceStart/Parachute Stage 2 core transform applied")
