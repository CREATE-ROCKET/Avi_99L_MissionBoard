#pragma once

#include <cstdint>

#include "protocol/can_protocol.hpp"
#include "sensors/flight_detectors.hpp"

namespace sensors {

struct AirDataFlightEvent {
  uint32_t flight_epoch{};
  bool lps_liftoff_detected{};
  bool pressure_apex_detected{};
};

class AirDataFlightLogic {
public:
  [[nodiscard]] AirDataFlightEvent update(
      uint32_t flight_epoch, protocol::MissionState state,
      uint64_t elapsed_since_liftoff_us, double pressure_hpa, bool valid);

private:
  void reset(uint32_t flight_epoch);

  LpsLiftoffDetector liftoff_detector_{};
  PressureApexDetector apex_detector_{};
  uint32_t flight_epoch_{};
};

} // 名前空間 sensors
