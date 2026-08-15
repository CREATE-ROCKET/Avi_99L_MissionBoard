#pragma once

#include <cstdint>

#include "actuators/parachute_configuration.hpp"
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
[[nodiscard]] bool loadFlightParachuteConfiguration(
    actuators::FlightParachuteConfiguration &configuration);
void storeFlightParachuteConfiguration(
    const actuators::FlightParachuteConfiguration &configuration);
void clearFlightParachuteConfiguration();
[[noreturn]] void enterPeriodicDeepSleep();

} // 名前空間 runtime::recovery_boot
