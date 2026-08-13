#include "sensors/air_data_flight_logic.hpp"

namespace sensors {

AirDataFlightEvent AirDataFlightLogic::update(
    uint32_t flight_epoch, protocol::MissionState state,
    uint64_t elapsed_since_liftoff_us, double pressure_hpa, bool valid) {
  if (flight_epoch != flight_epoch_ ||
      state == protocol::MissionState::command_receive)
    reset(flight_epoch);

  AirDataFlightEvent event{};
  event.flight_epoch = flight_epoch;
  if (state == protocol::MissionState::liftoff_detection)
    event.lps_liftoff_detected =
        liftoff_detector_.update(pressure_hpa, valid);
  else
    liftoff_detector_.reset();

  const bool apex_state = state == protocol::MissionState::engine_burn ||
                          state == protocol::MissionState::control;
  event.pressure_apex_detected = apex_detector_.update(
      pressure_hpa, valid && apex_state, elapsed_since_liftoff_us);
  return event;
}

void AirDataFlightLogic::reset(uint32_t flight_epoch) {
  liftoff_detector_.reset();
  apex_detector_.reset();
  flight_epoch_ = flight_epoch;
}

} // 名前空間 sensors
