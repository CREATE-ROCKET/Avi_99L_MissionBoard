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
constexpr uint16_t kFlightCheckpointVersion = 1;

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
};

RTC_DATA_ATTR FlightCheckpointRecord flight_checkpoint{};

constexpr uint32_t kParachuteCheckpointMagic = 0x39395043;

struct ParachuteCheckpointRecord {
  uint32_t magic{};
  uint16_t open_count{};
  uint16_t close_count{};
  uint16_t checksum{};
};

RTC_DATA_ATTR ParachuteCheckpointRecord parachute_checkpoint{};

uint16_t parachuteCheckpointChecksum(
    const ParachuteCheckpointRecord &record) {
  uint32_t value = record.magic ^
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
                   (record.power_cutoff_latched ? 0x9696U : 0U);
  value ^= value >> 32U;
  value ^= value >> 16U;
  return static_cast<uint16_t>(value);
}

bool validFlightCheckpoint(const FlightCheckpointRecord &record) {
  const auto state = static_cast<protocol::MissionState>(record.state);
  return record.magic == kFlightCheckpointMagic &&
         record.version == kFlightCheckpointVersion && record.elapsed_valid &&
         record.flight_epoch != 0 &&
         (state == protocol::MissionState::engine_burn ||
          state == protocol::MissionState::control ||
          state == protocol::MissionState::descent) &&
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
  const uint64_t rtc_now_us = esp_rtc_get_time_us();
  if (rtc_now_us < flight_checkpoint.rtc_time_us)
    return false;
  const uint64_t reset_elapsed_us =
      rtc_now_us - flight_checkpoint.rtc_time_us;
  if (UINT64_MAX - flight_checkpoint.elapsed_us < reset_elapsed_us)
    return false;
  checkpoint = {
      true,
      static_cast<protocol::MissionState>(flight_checkpoint.state),
      flight_checkpoint.flight_epoch,
      true,
      flight_checkpoint.elapsed_us + reset_elapsed_us,
      flight_checkpoint.deployment_started,
      flight_checkpoint.power_cutoff_latched,
  };
  return true;
}

void storeFlightCheckpoint(const mission::MissionSnapshot &snapshot) {
  if (!snapshot.liftoff_time_valid || snapshot.flight_epoch == 0 ||
      snapshot.state == protocol::MissionState::command_receive ||
      snapshot.state == protocol::MissionState::liftoff_detection) {
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
  record.elapsed_valid = true;
  record.deployment_started = snapshot.deployment_started;
  record.power_cutoff_latched = snapshot.deployment_power_cutoff_latched;
  record.checksum = checkpointChecksum(record);
  flight_checkpoint = record;
}

void clearFlightCheckpoint() { flight_checkpoint = {}; }

bool loadFlightParachuteConfiguration(
    actuators::FlightParachuteConfiguration &configuration) {
  if (!resetPreservesRtcMemory() ||
      parachute_checkpoint.magic != kParachuteCheckpointMagic ||
      parachute_checkpoint.checksum !=
          parachuteCheckpointChecksum(parachute_checkpoint))
    return false;
  const auto open = actuators::AbsoluteParachuteAngle::fromCount(
      parachute_checkpoint.open_count);
  const auto close = actuators::AbsoluteParachuteAngle::fromCount(
      parachute_checkpoint.close_count);
  if (!open.has_value() || !close.has_value() ||
      !actuators::shortestParachuteDisplacement(*close, *open).valid())
    return false;
  configuration = {*open, *close};
  return true;
}

void storeFlightParachuteConfiguration(
    const actuators::FlightParachuteConfiguration &configuration) {
  ParachuteCheckpointRecord record{};
  record.magic = kParachuteCheckpointMagic;
  record.open_count = configuration.open.count();
  record.close_count = configuration.close.count();
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
