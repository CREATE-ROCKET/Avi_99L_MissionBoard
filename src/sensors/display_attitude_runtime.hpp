#pragma once

#include <cstdint>

namespace bringup {
struct ImuSample;
}

namespace sensors::display_attitude_runtime {

struct WireTelemetry {
  uint8_t magnitude_raw{122};
  uint16_t direction_raw{511};
};

void observe(const bringup::ImuSample &sample);
void calibrationAccepted(uint8_t transaction_id);
void calibrationFinished(uint8_t transaction_id, bool success);
void invalidateForDataLoss();
void invalidateForReset();
[[nodiscard]] WireTelemetry wireTelemetry(uint64_t now_us);

} // namespace sensors::display_attitude_runtime
