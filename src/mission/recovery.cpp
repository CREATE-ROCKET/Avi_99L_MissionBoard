#include "mission/recovery.hpp"

namespace mission {
namespace {

constexpr uint32_t kRecoveryMagic = 0x39394C52;
constexpr uint16_t kRecoveryVersion = 2;

} // 無名名前空間

uint16_t recoveryMarkerChecksum(const RecoveryMarker &marker) {
  uint64_t value = marker.magic ^
                   (static_cast<uint32_t>(marker.version) << 16U) ^
                   marker.wake_sequence ^ marker.entry_rtc_time_us ^
                   (marker.entry_rtc_time_us >> 32U) ^
                   (marker.recovery_requested ? 0xA55AA55AU : 0U);
  value ^= value >> 32U;
  value ^= value >> 16U;
  return static_cast<uint16_t>(value);
}

RecoveryMarker makeRecoveryMarker(uint32_t wake_sequence,
                                  uint64_t entry_rtc_time_us) {
  RecoveryMarker marker{kRecoveryMagic, kRecoveryVersion, 0, wake_sequence,
                        entry_rtc_time_us, true};
  marker.checksum = recoveryMarkerChecksum(marker);
  return marker;
}

bool validRecoveryMarker(const RecoveryMarker &marker) {
  return marker.magic == kRecoveryMagic &&
         marker.version == kRecoveryVersion && marker.recovery_requested &&
         marker.checksum == recoveryMarkerChecksum(marker);
}

BootPath selectBootPath(const BootEvidence &evidence) {
  if (evidence.reset_kind == ResetKind::deep_sleep)
    return evidence.recovery_wake_cause &&
                   validRecoveryMarker(evidence.recovery_marker)
               ? BootPath::recovery_only
               : BootPath::fail_safe;
  if (!evidence.flight_checkpoint_valid)
    return BootPath::normal;
  if (evidence.reset_kind == ResetKind::software_or_watchdog)
    return evidence.rtc_elapsed_valid ? BootPath::resume_flight_safely
                                      : BootPath::fail_safe;
  if (evidence.absolute_liftoff_time_valid &&
      evidence.current_absolute_time_valid)
    return BootPath::resume_flight_safely;
  return BootPath::await_absolute_time;
}

bool RecoveryRuntime::requestEntry() {
  if (state_ != RecoveryRuntimeState::inactive)
    return false;
  state_ = RecoveryRuntimeState::flushing_and_safeing;
  return true;
}

bool RecoveryRuntime::markResourcesSafeAndFlushed() {
  if (state_ != RecoveryRuntimeState::flushing_and_safeing)
    return false;
  state_ = RecoveryRuntimeState::ready_for_deep_sleep;
  return true;
}

bool RecoveryRuntime::wake(bool marker_valid, bool wake_cause_valid) {
  if (!marker_valid || !wake_cause_valid)
    return false;
  state_ = RecoveryRuntimeState::wake_command_window;
  return true;
}

bool RecoveryRuntime::beginTransfer() {
  if (state_ != RecoveryRuntimeState::wake_command_window)
    return false;
  state_ = RecoveryRuntimeState::transfer_in_progress;
  return true;
}

void RecoveryRuntime::finishTransfer() {
  if (state_ == RecoveryRuntimeState::transfer_in_progress)
    state_ = RecoveryRuntimeState::ready_for_deep_sleep;
}

bool RecoveryRuntime::mayEnterDeepSleep() const {
  return state_ == RecoveryRuntimeState::ready_for_deep_sleep ||
         state_ == RecoveryRuntimeState::wake_command_window;
}

} // 名前空間 mission
