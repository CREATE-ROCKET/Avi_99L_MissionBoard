#pragma once

#include <cstdint>

namespace mission {

enum class ResetKind : uint8_t { power_on, software_or_watchdog, deep_sleep };
enum class BootPath : uint8_t {
  normal,
  resume_flight_safely,
  await_absolute_time,
  recovery_only,
  fail_safe,
};

struct RecoveryMarker {
  uint32_t magic{};
  uint16_t version{};
  uint16_t checksum{};
  uint32_t wake_sequence{};
  uint64_t entry_rtc_time_us{};
  bool recovery_requested{};
};

struct BootEvidence {
  ResetKind reset_kind{ResetKind::power_on};
  bool flight_checkpoint_valid{};
  bool rtc_elapsed_valid{};
  bool absolute_liftoff_time_valid{};
  bool current_absolute_time_valid{};
  bool recovery_wake_cause{};
  RecoveryMarker recovery_marker{};
};

[[nodiscard]] uint16_t recoveryMarkerChecksum(const RecoveryMarker &marker);
[[nodiscard]] RecoveryMarker makeRecoveryMarker(
    uint32_t wake_sequence, uint64_t entry_rtc_time_us = 0);
[[nodiscard]] bool validRecoveryMarker(const RecoveryMarker &marker);
[[nodiscard]] BootPath selectBootPath(const BootEvidence &evidence);

enum class RecoveryRuntimeState : uint8_t {
  inactive,
  flushing_and_safeing,
  ready_for_deep_sleep,
  wake_command_window,
  transfer_in_progress,
};

class RecoveryRuntime {
public:
  [[nodiscard]] bool requestEntry();
  [[nodiscard]] bool markResourcesSafeAndFlushed();
  [[nodiscard]] bool wake(bool marker_valid, bool wake_cause_valid);
  [[nodiscard]] bool beginTransfer();
  void finishTransfer();
  [[nodiscard]] bool mayEnterDeepSleep() const;
  [[nodiscard]] RecoveryRuntimeState state() const { return state_; }
  [[nodiscard]] static constexpr uint64_t periodicWakeUs() {
    // TODO(HW_TEST): wake周期、command window、CAN再initializationを確定する。
    return 10'000'000;
  }

private:
  RecoveryRuntimeState state_{RecoveryRuntimeState::inactive};
};

} // 名前空間 mission
