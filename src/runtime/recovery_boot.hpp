#pragma once

#include <cstdint>

#include "esp_err.h"
#include "mission/mission_state.hpp"

namespace runtime::recovery_boot {

[[nodiscard]] bool markerValid();
[[nodiscard]] bool wakeCauseValid();
[[nodiscard]] uint32_t wakeSequence();
void prepareMarker();
void clearMarker();
[[nodiscard]] bool loadFlightCheckpoint(mission::ResetCheckpoint &checkpoint);
void storeFlightCheckpoint(const mission::MissionSnapshot &snapshot);
void clearFlightCheckpoint();
[[noreturn]] void enterPeriodicDeepSleep();

} // 名前空間 runtime::recovery_boot
