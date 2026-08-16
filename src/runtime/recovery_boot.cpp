#include "runtime/recovery_boot.hpp"

#include "esp_attr.h"
#include "esp_rtc_time.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "mission/recovery.hpp"
#include "protocol/quantization.hpp"

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
  uint64_t preflight_captured_at_us{};
  uint32_t preflight_generation{};
  uint8_t state{};
  uint8_t preflight_ready_mask{};
  uint8_t preflight_missing_mask{};
  bool elapsed_valid{};
  bool deployment_started{};
  bool power_cutoff_latched{};
  bool forced_start{};
};
RTC_DATA_ATTR FlightCheckpointRecord flight_checkpoint{};

constexpr uint32_t kStartReadinessAuditMagic = 0x39395341;
constexpr uint8_t kStartReadinessAuditVersion = 1;
struct StartReadinessAuditRecord {
  uint32_t magic{};
  uint8_t version{};
  uint8_t ready_mask{};
  uint8_t missing_mask{};
  bool forced{};
  uint32_t generation{};
  uint64_t captured_at_us{};
  uint16_t checksum{};
};
RTC_DATA_ATTR StartReadinessAuditRecord start_readiness_audit{};

uint16_t startReadinessAuditChecksum(const StartReadinessAuditRecord &record) {
  uint64_t value = record.magic ^
                   (static_cast<uint32_t>(record.version) << 24U) ^
                   (static_cast<uint32_t>(record.ready_mask) << 16U) ^
                   (static_cast<uint32_t>(record.missing_mask) << 8U) ^
                   record.generation ^ record.captured_at_us ^
                   (record.captured_at_us >> 32U) ^
                   (record.forced ? 0x5AA5U : 0xA55AU);
  value ^= value >> 32U;
  value ^= value >> 16U;
  return static_cast<uint16_t>(value);
}

bool validStartReadinessAudit(const StartReadinessAuditRecord &record) {
  const uint8_t masks = static_cast<uint8_t>(
      (record.ready_mask | record.missing_mask) &
      mission::kPreflightReadinessMask);
  return record.magic == kStartReadinessAuditMagic &&
         record.version == kStartReadinessAuditVersion &&
         record.generation != 0 && masks == mission::kPreflightReadinessMask &&
         (record.ready_mask & record.missing_mask) == 0 &&
         record.checksum == startReadinessAuditChecksum(record);
}

uint16_t checkpointChecksum(const FlightCheckpointRecord &record) {
  uint64_t value = record.magic ^
                   (static_cast<uint32_t>(record.version) << 16U) ^
                   record.flight_epoch ^ record.elapsed_us ^
                   (record.elapsed_us >> 32U) ^ record.rtc_time_us ^
                   (record.rtc_time_us >> 32U) ^
                   record.preflight_captured_at_us ^
                   (record.preflight_captured_at_us >> 32U) ^
                   record.preflight_generation ^
                   (static_cast<uint32_t>(record.state) << 24U) ^
                   (static_cast<uint32_t>(record.preflight_ready_mask) << 8U) ^
                   record.preflight_missing_mask ^
                   (record.elapsed_valid ? 0xA55AU : 0U) ^
                   (record.deployment_started ? 0x5AA5U : 0U) ^
                   (record.power_cutoff_latched ? 0x9696U : 0U) ^
                   (record.forced_start ? 0xC33CU : 0U);
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
  const bool elapsed_contract =
      state == protocol::MissionState::liftoff_detection
          ? !record.elapsed_valid
          : record.elapsed_valid;
  const uint8_t masks = static_cast<uint8_t>(
      (record.preflight_ready_mask | record.preflight_missing_mask) &
      mission::kPreflightReadinessMask);
  const bool masks_disjoint =
      (record.preflight_ready_mask & record.preflight_missing_mask) == 0;
  return record.magic == kFlightCheckpointMagic &&
         record.version == kFlightCheckpointVersion &&
         record.flight_epoch != 0 && state_valid && elapsed_contract &&
         masks == mission::kPreflightReadinessMask && masks_disjoint &&
         record.checksum == checkpointChecksum(record);
}

bool resetPreservesRtcMemory() {
  const esp_reset_reason_t reason = esp_reset_reason();
  return reason != ESP_RST_POWERON && reason != ESP_RST_DEEPSLEEP;
}

bool markerStoragePresent() {
  return recovery_marker.magic != 0 || recovery_marker.version != 0 ||
         recovery_marker.checksum != 0 || recovery_marker.wake_sequence != 0 ||
         recovery_marker.entry_rtc_time_us != 0 ||
         recovery_marker.recovery_requested;
}

uint16_t recoveryTimeError(protocol::quantization::TimeError error) {
  return static_cast<uint16_t>(0xFFF0U | static_cast<uint8_t>(error));
}
} // 無名名前空間

bool loadStartReadinessAudit(StartReadinessAudit &audit) {
  audit = {};
  if (!resetPreservesRtcMemory() ||
      !validStartReadinessAudit(start_readiness_audit))
    return false;
  audit.valid = true;
  audit.forced = start_readiness_audit.forced;
  audit.generation = start_readiness_audit.generation;
  audit.captured_at_us = start_readiness_audit.captured_at_us;
  audit.ready_mask = start_readiness_audit.ready_mask;
  audit.missing_mask = start_readiness_audit.missing_mask;
  return true;
}

void storeStartReadinessAudit(
    const mission::PreflightReadinessSnapshot &readiness, bool forced) {
  StartReadinessAuditRecord record{};
  record.magic = kStartReadinessAuditMagic;
  record.version = kStartReadinessAuditVersion;
  record.ready_mask = readiness.readyMask();
  record.missing_mask = readiness.missingMask();
  record.forced = forced;
  record.generation = readiness.generation;
  record.captured_at_us = readiness.captured_at_us;
  record.checksum = startReadinessAuditChecksum(record);
  start_readiness_audit = record;
}

void clearStartReadinessAudit() { start_readiness_audit = {}; }

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

bool commitRecoveryEntryMarker() {
  if (mission::validRecoveryMarker(recovery_marker) &&
      recovery_marker.entry_rtc_time_us != 0)
    return true;

  const uint64_t now_us = esp_rtc_get_time_us();
  if (now_us == 0)
    return false;
  recovery_marker = mission::makeRecoveryMarker(0, now_us);
  return mission::validRecoveryMarker(recovery_marker) &&
         recovery_marker.entry_rtc_time_us == now_us;
}

uint16_t recoveryElapsedRaw(bool recovery_requested, bool recovery_only) {
  if (mission::validRecoveryMarker(recovery_marker)) {
    if (recovery_marker.entry_rtc_time_us == 0)
      return recoveryTimeError(
          protocol::quantization::TimeError::rtc_recovery_failed);
    if (recovery_only && !wakeCauseValid())
      return recoveryTimeError(
          protocol::quantization::TimeError::rtc_recovery_failed);

    const uint64_t now_us = esp_rtc_get_time_us();
    if (now_us < recovery_marker.entry_rtc_time_us)
      return recoveryTimeError(
          protocol::quantization::TimeError::time_inconsistent);
    const double elapsed_seconds =
        static_cast<double>(now_us - recovery_marker.entry_rtc_time_us) / 1.0e6;
    return protocol::quantization::encodeRecoveryElapsed(
        elapsed_seconds, protocol::quantization::TimeError::internal_error);
  }

  if (recovery_only) {
    return recoveryTimeError(
        markerStoragePresent()
            ? protocol::quantization::TimeError::checkpoint_invalid
            : protocol::quantization::TimeError::checkpoint_missing);
  }
  if (recovery_requested)
    return recoveryTimeError(
        protocol::quantization::TimeError::recovery_in_progress);
  return recoveryTimeError(protocol::quantization::TimeError::unavailable);
}

void prepareMarker() {
  if (!mission::validRecoveryMarker(recovery_marker) ||
      recovery_marker.entry_rtc_time_us == 0)
    return;
  recovery_marker = mission::makeRecoveryMarker(
      wakeSequence() + 1U, recovery_marker.entry_rtc_time_us);
}

void clearMarker() { recovery_marker = {}; }

bool loadFlightCheckpoint(mission::ResetCheckpoint &checkpoint) {
  checkpoint = {};
  if (!resetPreservesRtcMemory() || !validFlightCheckpoint(flight_checkpoint))
    return false;
  uint64_t elapsed = flight_checkpoint.elapsed_us;
  if (flight_checkpoint.elapsed_valid) {
    const uint64_t rtc_now_us = esp_rtc_get_time_us();
    if (rtc_now_us < flight_checkpoint.rtc_time_us)
      return false;
    const uint64_t reset_elapsed_us =
        rtc_now_us - flight_checkpoint.rtc_time_us;
    if (UINT64_MAX - elapsed < reset_elapsed_us)
      return false;
    elapsed += reset_elapsed_us;
  }
  checkpoint.valid = true;
  checkpoint.state =
      static_cast<protocol::MissionState>(flight_checkpoint.state);
  checkpoint.flight_epoch = flight_checkpoint.flight_epoch;
  checkpoint.elapsed_valid = flight_checkpoint.elapsed_valid;
  checkpoint.elapsed_us = elapsed;
  checkpoint.deployment_started = flight_checkpoint.deployment_started;
  checkpoint.power_cutoff_latched = flight_checkpoint.power_cutoff_latched;
  checkpoint.forced_start = flight_checkpoint.forced_start;
  checkpoint.preflight_generation = flight_checkpoint.preflight_generation;
  checkpoint.preflight_captured_at_us = flight_checkpoint.preflight_captured_at_us;
  checkpoint.preflight_ready_mask = flight_checkpoint.preflight_ready_mask;
  checkpoint.preflight_missing_mask = flight_checkpoint.preflight_missing_mask;
  return true;
}

void storeFlightCheckpoint(const mission::MissionSnapshot &snapshot) {
  if (snapshot.flight_epoch == 0 ||
      snapshot.state == protocol::MissionState::command_receive) {
    clearFlightCheckpoint();
    return;
  }
  const bool pre_liftoff =
      snapshot.state == protocol::MissionState::liftoff_detection;
  if (!pre_liftoff && !snapshot.liftoff_time_valid) {
    clearFlightCheckpoint();
    return;
  }
  FlightCheckpointRecord record{};
  record.magic = kFlightCheckpointMagic;
  record.version = kFlightCheckpointVersion;
  record.flight_epoch = snapshot.flight_epoch;
  record.elapsed_us = pre_liftoff ? 0 : snapshot.elapsed_us;
  record.rtc_time_us = esp_rtc_get_time_us();
  record.preflight_captured_at_us = snapshot.preflight_captured_at_us;
  record.preflight_generation = snapshot.preflight_generation;
  record.state = static_cast<uint8_t>(snapshot.state);
  record.preflight_ready_mask = snapshot.preflight_ready_mask;
  record.preflight_missing_mask = snapshot.preflight_missing_mask;
  record.elapsed_valid = !pre_liftoff;
  record.deployment_started = snapshot.deployment_started;
  record.power_cutoff_latched = snapshot.deployment_power_cutoff_latched;
  record.forced_start = snapshot.forced_start;
  record.checksum = checkpointChecksum(record);
  flight_checkpoint = record;
}

void clearFlightCheckpoint() { flight_checkpoint = {}; }

[[noreturn]] void enterPeriodicDeepSleep() {
  prepareMarker();
  (void)esp_sleep_enable_timer_wakeup(
      mission::RecoveryRuntime::periodicWakeUs());
  esp_deep_sleep_start();
}

} // 名前空間 runtime::recovery_boot