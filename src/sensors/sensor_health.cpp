#include "sensors/sensor_health.hpp"

#include <algorithm>
#include <cmath>

namespace sensors {

bool AirspeedGate::update(bool available, double airspeed_mps) {
  if (!available || !std::isfinite(airspeed_mps)) {
    entry_count_ = 0;
    stop_count_ = kStopSamples;
    above_threshold_ = false;
    return false;
  }
  if (airspeed_mps > 60.0) {
    stop_count_ = 0;
    entry_count_ = std::min<uint16_t>(kEntrySamples, entry_count_ + 1);
    if (entry_count_ >= kEntrySamples)
      above_threshold_ = true;
  } else {
    entry_count_ = 0;
    stop_count_ = std::min<uint16_t>(kStopSamples, stop_count_ + 1);
    if (stop_count_ >= kStopSamples)
      above_threshold_ = false;
  }
  return above_threshold_;
}

void AirspeedGate::reset() { *this = {}; }

} // 名前空間 sensors
