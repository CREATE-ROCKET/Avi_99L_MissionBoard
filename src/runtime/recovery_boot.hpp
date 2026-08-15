#pragma once

#include <cstdint>

#include "actuators/parachute_configuration.hpp"
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
void prepareMarker();
void clearMarker();
[[nodiscard]] bool loadFlightCheckpoint(mission::ResetCheckpoint &checkpoint);
void storeFlightCheckpoint(const mission::MissionSnapshot &snapshot);
void clearFlightCheckpoint();
[[nodiscard]] bool loadFlightParachuteConfiguration(
    actuators::FlightParachuteConfiguration &configuration);
void storeFlightParachuteConfiguration(
    const actuators::FlightParachuteConfiguration &configuration);
void clearFlightParachuteConfiguration();
[[noreturn]] void enterPeriodicDeepSleep();

} // 名前空間 runtime::recovery_boot
