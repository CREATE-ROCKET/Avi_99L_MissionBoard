#include "runtime/recovery_boot.hpp"

#include "esp_attr.h"
#include "esp_rtc_time.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "mission/recovery.hpp"

namespace runtime::recovery_boot {
namespace {

RTC_DATA_ATTR mission::RecoveryMarker recovery_marker{};

constexpr uint32_t kFlightCheckpointMagic = 0x39394C43;
constexpr uint16_t kFlightCheckpointVersion = 2;

struct FlightCheckpointRecord {
  uint32_t magic{};
  uint16_t version{};
  uint16_t checksum{};
  uint32_t flight_epoch{};
  uint64_t elapsed_us{};
  uint64_t rtc_time_us{};
  uint8_t state{};
  bool elapsed_valid{};
  bool deployment_started{};
  bool power_cutoff_latched{};
  bool forced_start{};
  uint8_t preflight_missing_mask{};
};

RTC_DATA_ATTR FlightCheckpointRecord flight_checkpoint{};

constexpr uint32_t kParachuteCheckpointMagic = 0x39395043;

constexpr uint16_t kParachuteCheckpointVersion = 2;
constexpr uint8_t kParachuteOpenPresent = 1U << 0U;
constexpr uint8_t kParachuteClosePresent = 1U << 1U;

struct ParachuteCheckpointRecord {
  uint32_t magic{};
  uint16_t version{};
  uint8_t flags{};
  uint8_t reserved{};
  uint16_t open_count{};
  uint16_t close_count{};
  uint16_t checksum{};
};

RTC_DATA_ATTR ParachuteCheckpointRecord parachute_checkpoint{};

uint16_t parachuteCheckpointChecksum(
    const ParachuteCheckpointRecord &record) {
  uint32_t value = record.magic ^
                   (static_cast<uint32_t>(record.version) << 16U) ^
                   (static_cast<uint32_t>(record.flags) << 8U) ^
                   (static_cast<uint32_t>(record.open_count) << 16U) ^
                   record.close_count;
  value ^= value >> 16U;
  return static_cast<uint16_t>(value);
}

uint16_t checkpointChecksum(const FlightCheckpointRecord &record) {
  uint64_t value = record.magic ^
                   (static_cast<uint32_t>(record.version) << 16U) ^
                   record.flight_epoch ^ record.elapsed_us ^
                   (record.elapsed_us >> 32U) ^
                   record.rtc_time_us ^ (record.rtc_time_us >> 32U) ^
                   (static_cast<uint32_t>(record.state) << 24U) ^
                   (record.elapsed_valid ? 0xA55AU : 0U) ^
                   (record.deployment_started ? 0x5AA5U : 0U) ^
                   (record.power_cutoff_latched ? 0x9696U : 0U) ^
                   (record.forced_start ? 0xC33CU : 0U) ^
                   (static_cast<uint32_t>(record.preflight_missing_mask)
                    << 8U);
  value ^= value >> 32U;
  value ^= value >> 16U;
  return static_cast<uint16_t>(value);
}

bool validFlightCheckpoint(const FlightCheckpointRecord &record) {
  const auto state = static_cast<protocol::MissionState>(record.state);
  const bool state_valid =
      state == protocol::MissionState::liftoff_detection ||
      state == protocol::MissionState::engine_burn ||
      state == protocol::MissionState::control ||
      state == protocol::MissionState::descent;
  const bool elapsed_state_valid =
      state == protocol::MissionState::liftoff_detection
          ? !record.elapsed_valid
          : record.elapsed_valid;
  return record.magic == kFlightCheckpointMagic &&
         record.version == kFlightCheckpointVersion && state_valid &&
         elapsed_state_valid && record.flight_epoch != 0 &&
         (record.preflight_missing_mask & 0x80U) == 0 &&
         record.checksum == checkpointChecksum(record);
}

bool resetPreservesRtcMemory() {
  const esp_reset_reason_t reason = esp_reset_reason();
  return reason != ESP_RST_POWERON && reason != ESP_RST_DEEPSLEEP;
}

} // 無名名前空間

bool markerValid() { return mission::validRecoveryMarker(recovery_marker); }

bool wakeCauseValid() {
  return (esp_sleep_get_wakeup_causes() &
          (uint32_t{1} << ESP_SLEEP_WAKEUP_TIMER)) != 0;
}

uint32_t wakeSequence() {
  return mission::validRecoveryMarker(recovery_marker)
             ? recovery_marker.wake_sequence
             : 0;
}

void prepareMarker() {
  recovery_marker = mission::makeRecoveryMarker(wakeSequence() + 1U);
}

void clearMarker() { recovery_marker = {}; }

bool loadFlightCheckpoint(mission::ResetCheckpoint &checkpoint) {
  checkpoint = {};
  if (!resetPreservesRtcMemory() || !validFlightCheckpoint(flight_checkpoint))
    return false;
  uint64_t elapsed_us = flight_checkpoint.elapsed_us;
  if (flight_checkpoint.elapsed_valid) {
    const uint64_t rtc_now_us = esp_rtc_get_time_us();
    if (rtc_now_us < flight_checkpoint.rtc_time_us)
      return false;
    const uint64_t reset_elapsed_us =
        rtc_now_us - flight_checkpoint.rtc_time_us;
    if (UINT64_MAX - elapsed_us < reset_elapsed_us)
      return false;
    elapsed_us += reset_elapsed_us;
  }
  checkpoint = {
      true,
      static_cast<protocol::MissionState>(flight_checkpoint.state),
      flight_checkpoint.flight_epoch,
      flight_checkpoint.elapsed_valid,
      elapsed_us,
      flight_checkpoint.deployment_started,
      flight_checkpoint.power_cutoff_latched,
      flight_checkpoint.forced_start,
      flight_checkpoint.preflight_missing_mask,
  };
  return true;
}

void storeFlightCheckpoint(const mission::MissionSnapshot &snapshot) {
  if (snapshot.flight_epoch == 0 ||
      snapshot.state == protocol::MissionState::command_receive) {
    clearFlightCheckpoint();
    return;
  }
  FlightCheckpointRecord record{};
  record.magic = kFlightCheckpointMagic;
  record.version = kFlightCheckpointVersion;
  record.flight_epoch = snapshot.flight_epoch;
  record.elapsed_us = snapshot.elapsed_us;
  record.rtc_time_us = esp_rtc_get_time_us();
  record.state = static_cast<uint8_t>(snapshot.state);
  record.elapsed_valid = snapshot.liftoff_time_valid;
  record.deployment_started = snapshot.deployment_started;
  record.power_cutoff_latched = snapshot.deployment_power_cutoff_latched;
  record.forced_start = snapshot.forced_start;
  record.preflight_missing_mask = snapshot.preflight_missing_mask;
  record.checksum = checkpointChecksum(record);
  flight_checkpoint = record;
}

void clearFlightCheckpoint() { flight_checkpoint = {}; }

bool loadFlightParachuteConfiguration(
    actuators::FlightParachuteConfiguration &configuration) {
  configuration = {};
  if (!resetPreservesRtcMemory() ||
      parachute_checkpoint.magic != kParachuteCheckpointMagic ||
      parachute_checkpoint.version != kParachuteCheckpointVersion ||
      parachute_checkpoint.reserved != 0 ||
      (parachute_checkpoint.flags &
       static_cast<uint8_t>(~(kParachuteOpenPresent |
                              kParachuteClosePresent))) != 0 ||
      parachute_checkpoint.checksum !=
          parachuteCheckpointChecksum(parachute_checkpoint))
    return false;

  if ((parachute_checkpoint.flags & kParachuteOpenPresent) != 0) {
    configuration.open = actuators::AbsoluteParachuteAngle::fromCount(
        parachute_checkpoint.open_count);
    if (!configuration.open.has_value())
      return false;
  }
  if ((parachute_checkpoint.flags & kParachuteClosePresent) != 0) {
    configuration.close = actuators::AbsoluteParachuteAngle::fromCount(
        parachute_checkpoint.close_count);
    if (!configuration.close.has_value())
      return false;
  }
  return true;
}

void storeFlightParachuteConfiguration(
    const actuators::FlightParachuteConfiguration &configuration) {
  ParachuteCheckpointRecord record{};
  record.magic = kParachuteCheckpointMagic;
  record.version = kParachuteCheckpointVersion;
  if (configuration.open.has_value()) {
    record.flags |= kParachuteOpenPresent;
    record.open_count = configuration.open->count();
  }
  if (configuration.close.has_value()) {
    record.flags |= kParachuteClosePresent;
    record.close_count = configuration.close->count();
  }
  record.checksum = parachuteCheckpointChecksum(record);
  parachute_checkpoint = record;
}

void clearFlightParachuteConfiguration() { parachute_checkpoint = {}; }

[[noreturn]] void enterPeriodicDeepSleep() {
  prepareMarker();
  (void)esp_sleep_enable_timer_wakeup(
      mission::RecoveryRuntime::periodicWakeUs());
  esp_deep_sleep_start();
}

} // 名前空間 runtime::recovery_boot
