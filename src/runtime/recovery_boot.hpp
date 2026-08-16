#pragma once

#include <cstdint>

#include "esp_err.h"
#include "mission/mission_state.hpp"

namespace runtime::recovery_boot {

struct StartReadinessAudit {
  bool valid{};
  bool forced{};
  uint32_t generation{};
  uint64_t captured_at_us{};
  uint8_t ready_mask{};
  uint8_t missing_mask{};
};

[[nodiscard]] bool loadStartReadinessAudit(StartReadinessAudit &audit);
void storeStartReadinessAudit(
    const mission::PreflightReadinessSnapshot &readiness, bool forced);
void clearStartReadinessAudit();

[[nodiscard]] bool markerValid();
[[nodiscard]] bool wakeCauseValid();
[[nodiscard]] uint32_t wakeSequence();
[[nodiscard]] bool commitRecoveryEntryMarker();
[[nodiscard]] uint16_t recoveryElapsedRaw(bool recovery_requested,
                                          bool recovery_only);
void prepareMarker();
void clearMarker();
[[nodiscard]] bool loadFlightCheckpoint(mission::ResetCheckpoint &checkpoint);
void storeFlightCheckpoint(const mission::MissionSnapshot &snapshot);
void clearFlightCheckpoint();
[[noreturn]] void enterPeriodicDeepSleep();

} // 名前空間 runtime::recovery_boot