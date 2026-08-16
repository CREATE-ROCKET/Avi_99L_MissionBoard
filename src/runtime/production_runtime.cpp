#include "runtime/production_runtime.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "CANCREATE.h"
#include "I2CCREATE.h"
#include "LPS25HB.h"
#include "SSCDRRN005PD2A5.h"
#include "STS3215.h"
#include "STSCREATE.h"
#include "avi_esp_libs/timeout.h"
#include "bringup/encoder_bringup.hpp"
#include "bringup/imu_bringup.hpp"
#include "bringup/power_bringup.hpp"
#include "bringup/safe_outputs.hpp"
#include "bringup/spi_bringup.hpp"
#include "config/board_config.hpp"
#include "config/flight_config.hpp"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mission/command_executor.hpp"
#include "mission/mission_state.hpp"
#include "mission/recovery.hpp"
#include "nvs.h"
#include "nvs_flash.h"
#include "actuators/parachute_configuration.hpp"
#include "actuators/production_motor.hpp"
#include "actuators/safety_core.hpp"
#include "control/control_pipeline.hpp"
#include "protocol/can_protocol.hpp"
#include "protocol/quantization.hpp"
#include "runtime/task_architecture.hpp"
#include "runtime/recovery_boot.hpp"
#include "runtime/emergency_latch.hpp"
#include "runtime/flight_log.hpp"
#include "runtime/flight_runtime_metadata.hpp"
#include "runtime/flight_storage.hpp"
#include "sensors/air_data_flight_logic.hpp"
#include "sensors/airspeed_estimator.hpp"
#include "sensors/as5047d_health.hpp"
#include "sensors/attitude_estimator.hpp"
#include "sensors/flight_detectors.hpp"
#include "sensors/sensor_health.hpp"
#include "sensors/power_presence_runtime.hpp"

namespace runtime {
namespace {

struct RuntimeStatus {
  protocol::MissionState state{protocol::MissionState::command_receive};
  uint16_t flight_status{};
  uint8_t config_flags{};
  protocol::FinMode fin_mode{protocol::FinMode::brake};
  protocol::ParaMode para_mode{protocol::ParaMode::powered_off};
  uint16_t lps_pressure_raw{
      static_cast<uint16_t>(protocol::quantization::LpsPressureError::unavailable)};
  uint8_t lps_temperature_raw{
      static_cast<uint8_t>(protocol::quantization::LpsTemperatureError::unavailable)};
  uint8_t airspeed_raw{
      static_cast<uint8_t>(
          protocol::quantization::AirspeedError::ssc_not_initialized)};
  bool lps_sample_valid{};
  bool airspeed_sample_valid{};
  bool deployment_power_cutoff{};
  uint64_t flight_elapsed_us{};
  float static_pressure_pa{};
  float ssc_temperature_celsius{};
  float airspeed_mps{};
  uint16_t roll_raw{
      static_cast<uint16_t>(protocol::quantization::RollError::unavailable)};
  uint16_t roll_rate_raw{
      static_cast<uint16_t>(protocol::quantization::RollError::unavailable)};
  uint8_t fin_angle_raw{static_cast<uint8_t>(
      protocol::quantization::FinAngleError::not_initialized)};
  uint16_t fin_rate_raw{static_cast<uint16_t>(
      protocol::quantization::FinRateError::estimator_not_ready)};
  uint16_t requested_torque_raw{static_cast<uint16_t>(
      protocol::quantization::TorqueError::unavailable)};
  uint16_t control_roll_reference_unwrapped_raw{
      static_cast<uint16_t>(protocol::quantization::RollError::unavailable)};
  uint16_t roll_deviation_unwrapped_raw{
      static_cast<uint16_t>(protocol::quantization::RollError::unavailable)};
  uint8_t control_roll_flags{};
  uint8_t control_roll_reference_capture_event_sequence{};
  double roll_estimate_liftoff_relative_unwrapped_rad{};
  double control_roll_reference_unwrapped_rad{};
  double roll_deviation_unwrapped_rad{};
  uint64_t control_roll_reference_capture_tick{};
  uint64_t control_roll_reference_estimator_timestamp_us{};
};

struct AirDataSnapshot {
  sensors::AirDataFlightEvent flight{};
  uint64_t lps_monotonic_us{};
  uint64_t ssc_monotonic_us{};
  uint16_t pressure_raw{
      static_cast<uint16_t>(protocol::quantization::LpsPressureError::unavailable)};
  uint8_t temperature_raw{
      static_cast<uint8_t>(protocol::quantization::LpsTemperatureError::unavailable)};
  uint8_t airspeed_raw{
      static_cast<uint8_t>(
          protocol::quantization::AirspeedError::ssc_not_initialized)};
  double static_pressure_pa{};
  double ssc_temperature_celsius{};
  double airspeed_mps{};
  bool lps_valid{};
  bool ssc_valid{};
  bool ssc_zero_valid{};
  bool airspeed_valid{};
};

struct PowerRequest {
  bool auxiliary_5v{};
  bool parachute_power{};
  bool cutoff{};
  // 各taskが所有していない電源railを意図せず上書きしないための更新mask。
  bool update_auxiliary_5v{true};
  bool update_parachute_power{true};
};

struct MissionCommandEnvelope {
  protocol::GenericCommandRequest request{};
  mission::CommandDecision decision{};
  mission::PreflightReadinessSnapshot readiness{};
};

struct EmergencyEnvelope {
  uint8_t transaction_id{};
  bool liftoff_detection{};
};

struct ParaRequest {
  enum class Kind : uint8_t {
    open,
    free,
    power_off,
    discard_snapshot,
  } kind{Kind::power_off};
  uint32_t flight_epoch{};
  bool safety_authorized{};
};

struct ParachuteCommandRequest {
  enum class Kind : uint8_t { generic, start_preparation } kind{Kind::generic};
  protocol::GenericCommandRequest command{};
  mission::PreflightReadinessSnapshot readiness{};
};

struct ParachuteStartResponse {
  uint8_t transaction_id{};
  protocol::CommandReason reason{protocol::CommandReason::none};
  uint32_t detail{};
};

struct ParachutePersistenceRequest {
  uint8_t transaction_id{};
  actuators::ParachuteEndpoint endpoint{actuators::ParachuteEndpoint::open};
  uint16_t count{};
  bool previous_valid{};
  uint16_t previous_count{};
};

struct ParachutePersistenceResponse {
  enum class Kind : uint8_t { load, save } kind{Kind::load};
  uint8_t transaction_id{};
  actuators::ParachuteEndpoint endpoint{actuators::ParachuteEndpoint::open};
  bool success{};
  bool persistence_ready{};
  bool corruption_detected{};
  bool open_valid{};
  uint16_t open_count{};
  bool close_valid{};
  uint16_t close_count{};
};

struct RecoveryRequest {
  enum class Kind : uint8_t { control, enter } kind{Kind::control};
  protocol::RecoveryControl control{};
  protocol::RecoveryModeReason reason{
      protocol::RecoveryModeReason::auto_elapsed_120};
};

struct RecoveryDumpCursor {
  bool active{};
  protocol::RecoveryControl control{};
  uint32_t next_offset{};
  uint32_t remaining{};
  uint8_t sequence{};
};

struct StorageExportRequest {
  uint8_t transaction_id{};
};

struct TimeState {
  protocol::TimeSource source{protocol::TimeSource::invalid};
  uint32_t unix_seconds{};
  uint16_t milliseconds{};
  uint64_t received_monotonic_us{};
  uint8_t request_id{};
  bool valid{};
};

struct EventRequest {
  uint16_t flags{};
  protocol::MissionState state{protocol::MissionState::command_receive};
  uint16_t elapsed_raw{};
  uint16_t detail{};
};

enum class PersistenceSignal : uint8_t { flush_and_safe };

enum class ParachuteDeploymentFailure : uint8_t {
  none = 0,
  open_not_configured = 1,
  current_angle_unavailable = 2,
  ambiguous_half_turn = 3,
  move_command_failed = 4,
  retry_exhausted = 5,
  hold_failed = 6,
};

protocol::CommandReason transitionReason(mission::TransitionResult result);

StaticQueue_t status_queue_storage;
StaticQueue_t power_queue_storage;
StaticQueue_t command_queue_storage;
StaticQueue_t transition_queue_storage;
StaticQueue_t emergency_queue_storage;
StaticQueue_t result_queue_storage;
StaticQueue_t para_queue_storage;
StaticQueue_t parachute_command_queue_storage;
StaticQueue_t parachute_start_response_queue_storage;
StaticQueue_t parachute_persistence_request_queue_storage;
StaticQueue_t parachute_persistence_response_queue_storage;
StaticQueue_t air_data_queue_storage;
StaticQueue_t recovery_queue_storage;
StaticQueue_t recovery_status_queue_storage;
StaticQueue_t recovery_log_data_queue_storage;
StaticQueue_t sd_recovery_queue_storage;
StaticQueue_t storage_export_queue_storage;
StaticQueue_t flash_log_queue_storage;
StaticQueue_t sd_log_queue_storage;
StaticQueue_t event_queue_storage;
StaticQueue_t persistence_queue_storage;
std::array<uint8_t, sizeof(RuntimeStatus)> status_queue_buffer{};
std::array<uint8_t, sizeof(PowerRequest) * 4> power_queue_buffer{};
std::array<uint8_t, sizeof(MissionCommandEnvelope) * 32>
    command_queue_buffer{};
std::array<uint8_t, sizeof(MissionCommandEnvelope) * 8>
    transition_queue_buffer{};
std::array<uint8_t, sizeof(EmergencyEnvelope) * 16> emergency_queue_buffer{};
std::array<uint8_t, sizeof(protocol::CommandResult) * 32>
    result_queue_buffer{};
std::array<uint8_t, sizeof(ParaRequest) * 4> para_queue_buffer{};
std::array<uint8_t, sizeof(ParachuteCommandRequest) * 8>
    parachute_command_queue_buffer{};
std::array<uint8_t, sizeof(ParachuteStartResponse) * 2>
    parachute_start_response_queue_buffer{};
std::array<uint8_t, sizeof(ParachutePersistenceRequest) * 4>
    parachute_persistence_request_queue_buffer{};
std::array<uint8_t, sizeof(ParachutePersistenceResponse) * 4>
    parachute_persistence_response_queue_buffer{};
std::array<uint8_t, sizeof(AirDataSnapshot)> air_data_queue_buffer{};
std::array<uint8_t, sizeof(RecoveryRequest) * 4> recovery_queue_buffer{};
std::array<uint8_t, sizeof(protocol::RecoveryStatusMessage) * 4>
    recovery_status_queue_buffer{};
std::array<uint8_t, sizeof(protocol::RecoveryLogData) * 16>
    recovery_log_data_queue_buffer{};
std::array<uint8_t, sizeof(protocol::RecoveryControl) * 4>
    sd_recovery_queue_buffer{};
std::array<uint8_t, sizeof(StorageExportRequest) * 2>
    storage_export_queue_buffer{};
std::array<uint8_t, sizeof(flight_log::SerializedRecord) * 32>
    flash_log_queue_buffer{};
std::array<uint8_t, sizeof(flight_log::SerializedRecord) * 64>
    sd_log_queue_buffer{};
std::array<uint8_t, sizeof(EventRequest) * 16> event_queue_buffer{};
std::array<uint8_t, sizeof(PersistenceSignal)> persistence_queue_buffer{};
QueueHandle_t status_queue{};
QueueHandle_t power_queue{};
QueueHandle_t command_queue{};
QueueHandle_t transition_queue{};
QueueHandle_t emergency_queue{};
QueueHandle_t result_queue{};
QueueHandle_t para_queue{};
QueueHandle_t parachute_command_queue{};
QueueHandle_t parachute_start_response_queue{};
QueueHandle_t parachute_persistence_request_queue{};
QueueHandle_t parachute_persistence_response_queue{};
QueueHandle_t air_data_queue{};
QueueHandle_t recovery_queue{};
QueueHandle_t recovery_status_queue{};
QueueHandle_t recovery_log_data_queue{};
QueueHandle_t sd_recovery_queue{};
QueueHandle_t storage_export_queue{};
QueueHandle_t flash_log_queue{};
QueueHandle_t sd_log_queue{};
QueueHandle_t event_queue{};
QueueHandle_t persistence_queue{};
std::atomic<uint32_t> result_queue_overflow{};
std::atomic<uint32_t> emergency_metadata_overflow{};
std::atomic<uint32_t> flash_log_drop_count{};
std::atomic<uint32_t> sd_log_drop_count{};
std::atomic<bool> flash_log_ready{false};
std::atomic<bool> sd_log_ready{false};
std::atomic<bool> flash_log_failed{false};
std::atomic<bool> sd_log_failed{false};
std::atomic<uint16_t> event_overflow_latch{};
std::atomic<uint16_t> parachute_failure_overflow_detail{};
std::atomic<bool> runtime_started{false};
std::atomic<bool> imu_ready{false};
std::atomic<bool> encoder_ready{false};
std::atomic<bool> motor_ready{false};
std::atomic<bool> fin_zero_configured{false};
std::atomic<bool> lps_ready{false};
std::atomic<bool> ssc_ready{false};
std::atomic<bool> sts_ready{false};
std::atomic<uint8_t> logic_voltage_raw{static_cast<uint8_t>(
    protocol::quantization::BatteryError::unavailable)};
std::atomic<uint8_t> motor_voltage_raw{static_cast<uint8_t>(
    protocol::quantization::BatteryError::unavailable)};
std::atomic<uint32_t> motor_bus_millivolts{9'000};
std::atomic<bool> motor_bus_voltage_valid{false};
std::atomic<bool> logic_power_present{false};
std::atomic<bool> motor_power_present{false};
std::atomic<uint8_t> parachute_angle_raw{
    static_cast<uint8_t>(
        protocol::quantization::ParachuteAngleError::unavailable)};
// SafetyTaskがGPIO44へ最後に正常適用したPara電源状態。
// queue投入完了と物理GPIO適用完了を同一視しない。
std::atomic<bool> parachute_power_applied{false};
std::atomic<bool> parachute_config_load_complete{false};
std::atomic<bool> parachute_persistence_ready{false};
std::atomic<bool> parachute_persistence_corrupt{false};
std::atomic<uint8_t> parachute_deployment_failure{};
std::atomic<bool> parachute_open_configured{false};
std::atomic<bool> parachute_close_configured{false};
std::atomic<protocol::ParaMode> para_mode_actual{
    protocol::ParaMode::powered_off};
std::atomic<bool> can_healthy{false};
EmergencyLatch actuator_emergency_latch;
EmergencyLatch liftoff_emergency_latch;
std::atomic<bool> recovery_requested{false};
std::atomic<bool> persistence_flushed{false};
std::atomic<bool> recovery_only_mode{false};
std::atomic<bool> recovery_wake_valid{false};
std::atomic<bool> emergency_power_safe_requested{false};
std::atomic<bool> recovery_power_safe{false};
std::atomic<bool> recovery_motor_safe{false};
std::atomic<bool> recovery_sd_flushed{false};
std::atomic<bool> recovery_mode_command_pending{false};
std::atomic<bool> recovery_mode_command_sent{false};
std::atomic<uint8_t> recovery_mode_reason{static_cast<uint8_t>(
    protocol::RecoveryModeReason::auto_elapsed_120)};
std::atomic<uint32_t> pressure_deployment_epoch{};
std::atomic<uint32_t> preflight_calibration_generation{};
std::atomic<bool> preflight_calibration_active{false};
TimeState time_state;
SemaphoreHandle_t time_mutex{};
StaticSemaphore_t time_mutex_storage;
std::atomic<bool> fin_zero_hold_valid{false};
std::atomic<bool> actuator_output_inhibited{false};
mission::MissionStateMachine state_machine;
mission::CommandExecutor command_executor;
SemaphoreHandle_t state_mutex{};
StaticSemaphore_t state_mutex_storage;
SemaphoreHandle_t executor_mutex{};
StaticSemaphore_t executor_mutex_storage;
flight_storage::InternalFlashLog internal_flash_log;
flight_storage::SdFlightLog sd_flight_log;

constexpr std::size_t kTaskCount = kTaskArchitecture.size();
constexpr uint32_t kTaskStackWords = 6'144;
std::array<StaticTask_t, kTaskCount> task_controls{};
std::array<std::array<StackType_t, kTaskStackWords>, kTaskCount> task_stacks{};

void addWatchdog() {
  const esp_err_t result = esp_task_wdt_add(nullptr);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE)
    std::printf("watchdog add failed: %s\n", esp_err_to_name(result));
}

void resetWatchdog() { (void)esp_task_wdt_reset(); }

esp_err_t setTrackedParaPower(bool enabled) {
  const bool was_applied =
      parachute_power_applied.load(std::memory_order_acquire);
  const esp_err_t result = bringup::safe_outputs::setParaPower(enabled);
  // OFF失敗時もUART ownerは安全側に倒して「給電済み」と扱わない。
  parachute_power_applied.store(result == ESP_OK && enabled,
                                std::memory_order_release);
  if (result != ESP_OK) {
    parachute_angle_raw.store(
        static_cast<uint8_t>(protocol::quantization::ParachuteAngleError::unknown),
        std::memory_order_release);
  } else if (!enabled) {
    parachute_angle_raw.store(
        static_cast<uint8_t>(
            protocol::quantization::ParachuteAngleError::powered_off),
        std::memory_order_release);
  } else if (!was_applied) {
    parachute_angle_raw.store(
        static_cast<uint8_t>(
            protocol::quantization::ParachuteAngleError::not_initialized),
        std::memory_order_release);
  }
  return result;
}

void failSafeOutputs() {
  (void)bringup::safe_outputs::motorCoast();
  (void)bringup::safe_outputs::setAux5v(false);
  (void)setTrackedParaPower(false);
}

void latchPhysicalEmergency(uint8_t transaction_id, bool liftoff) {
  const EmergencyEnvelope envelope{transaction_id, liftoff};
  if (xQueueSendToFront(emergency_queue, &envelope, 0) == pdTRUE) {
    if (!liftoff)
      emergency_power_safe_requested.store(true, std::memory_order_release);
    return;
  }
  if (liftoff) {
    if (liftoff_emergency_latch.signal(transaction_id))
      emergency_metadata_overflow.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (actuator_emergency_latch.signal(transaction_id))
    emergency_metadata_overflow.fetch_add(1, std::memory_order_relaxed);
  emergency_power_safe_requested.store(true, std::memory_order_release);
}

void enqueueResult(const protocol::CommandResult &result, bool priority) {
  const BaseType_t queued = priority
                                ? xQueueSendToFront(result_queue, &result, 0)
                                : xQueueSend(result_queue, &result, 0);
  if (queued != pdTRUE)
    result_queue_overflow.fetch_add(1, std::memory_order_relaxed);
}

void enqueueEvent(uint16_t flags, protocol::MissionState state,
                  uint16_t elapsed_raw = 0, uint16_t detail = 0) {
  const EventRequest event{flags, state, elapsed_raw, detail};
  if (xQueueSend(event_queue, &event, 0) != pdTRUE) {
    const uint16_t failure_flag = protocol::eventFlag(
        protocol::MissionEventFlag::parachute_deployment_failure);
    if ((flags & failure_flag) != 0)
      parachute_failure_overflow_detail.store(detail,
                                              std::memory_order_release);
    event_overflow_latch.fetch_or(flags, std::memory_order_relaxed);
  }
}

CANCREATE::Config canConfig() {
  CANCREATE::Config config{};
  config.tx = board::kCanTx;
  config.rx = board::kCanRx;
  config.bitrate = CANCREATE::Bitrate::kbps125;
  config.mode = CANCREATE::Mode::normal;
  config.rx_queue_depth = 32;
  config.allow_diagnostic_frames = false;
  return config;
}

protocol::CanFrame toProtocol(const CANCREATE::Frame &frame) {
  protocol::CanFrame result{};
  result.identifier = static_cast<uint16_t>(frame.identifier);
  result.data_length = frame.data_length;
  std::copy_n(frame.data, frame.data_length, result.data.begin());
  result.extended = frame.extended;
  result.remote = frame.remote;
  return result;
}

esp_err_t writeFrame(CANCREATE &can, const protocol::CanFrame &frame) {
  CANCREATE::Frame output{};
  output.identifier = frame.identifier;
  output.data_length = frame.data_length;
  std::copy_n(frame.data.begin(), frame.data_length, output.data);
  // TX完了callbackまで1枠を占有するため、連続telemetryを有限時間で直列化する。
  return can.write(output, avi::Timeout::milliseconds(5));
}

void safetyTask(void *) {
  addWatchdog();
  bool cutoff_latched = false;
  uint32_t tracked_epoch = 0;
  bool liftoff_valid = false;
  uint64_t liftoff_time_us = 0;
  uint64_t elapsed_offset_us = 0;
  bool deployment_requested = false;
  bool recovery_entry_queued = false;
  protocol::MissionState tracked_state = protocol::MissionState::command_receive;
  for (;;) {
    if (emergency_power_safe_requested.exchange(false,
                                                std::memory_order_acq_rel)) {
      // Emergency latchはqueue容量と無関係にSafetyTaskからpowerを落とす。
      (void)bringup::safe_outputs::setAux5v(false);
      (void)setTrackedParaPower(false);
      sts_ready.store(false, std::memory_order_release);
      para_mode_actual.store(protocol::ParaMode::powered_off,
                             std::memory_order_release);
    }
    if (recovery_requested.load(std::memory_order_acquire)) {
      const bool aux_safe = bringup::safe_outputs::setAux5v(false) == ESP_OK;
      const bool para_safe =
          setTrackedParaPower(false) == ESP_OK;
      recovery_power_safe.store(aux_safe && para_safe,
                                std::memory_order_release);
      if (para_safe) {
        sts_ready.store(false, std::memory_order_release);
        para_mode_actual.store(protocol::ParaMode::powered_off,
                               std::memory_order_release);
      }
    }
    PowerRequest request{};
    while (xQueueReceive(power_queue, &request, 0) == pdTRUE) {
      cutoff_latched = cutoff_latched || request.cutoff;
      if (cutoff_latched) {
        (void)bringup::safe_outputs::setAux5v(false);
        (void)setTrackedParaPower(false);
        sts_ready.store(false, std::memory_order_release);
        para_mode_actual.store(protocol::ParaMode::powered_off,
                               std::memory_order_release);
      } else {
        if (request.update_auxiliary_5v)
          (void)bringup::safe_outputs::setAux5v(request.auxiliary_5v);
        if (request.update_parachute_power)
          (void)setTrackedParaPower(request.parachute_power);
      }
    }
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      const auto snapshot = state_machine.snapshot();
      tracked_state = snapshot.state;
      if (snapshot.flight_epoch != tracked_epoch) {
        tracked_epoch = snapshot.flight_epoch;
        recovery_entry_queued = false;
        cutoff_latched = snapshot.deployment_power_cutoff_latched;
        liftoff_valid = snapshot.liftoff_time_valid;
        liftoff_time_us = snapshot.liftoff_time_us;
        elapsed_offset_us = snapshot.reset_invalidated
                                ? snapshot.elapsed_us
                                : 0;
        deployment_requested = snapshot.deployment_started;
      } else if (snapshot.liftoff_time_valid && !liftoff_valid) {
        liftoff_valid = true;
        liftoff_time_us = snapshot.liftoff_time_us;
        elapsed_offset_us = snapshot.reset_invalidated
                                ? snapshot.elapsed_us
                                : 0;
      }
      deployment_requested = deployment_requested ||
                             snapshot.deployment_started;
      xSemaphoreGive(state_mutex);
    }
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t elapsed_us =
        liftoff_valid && now >= liftoff_time_us
            ? elapsed_offset_us + now - liftoff_time_us
            : 0;
    const bool pressure_deploy =
        pressure_deployment_epoch.load(std::memory_order_acquire) ==
        tracked_epoch;
    if (!deployment_requested &&
        ((pressure_deploy && elapsed_us >= 10'000'000) ||
         elapsed_us >= 17'000'000)) {
      const ParaRequest para{ParaRequest::Kind::open, tracked_epoch, true};
      deployment_requested =
          xQueueSendToFront(para_queue, &para, 0) == pdTRUE;
    }
    if (!cutoff_latched && liftoff_valid && now >= liftoff_time_us &&
        elapsed_us >= 25'000'000) {
      cutoff_latched = true;
      (void)bringup::safe_outputs::setAux5v(false);
      (void)setTrackedParaPower(false);
      sts_ready.store(false, std::memory_order_release);
      para_mode_actual.store(protocol::ParaMode::powered_off,
                             std::memory_order_release);
      const ParaRequest para{ParaRequest::Kind::power_off, tracked_epoch,
                             true};
      (void)xQueueSend(para_queue, &para, 0);
    }
    if (!recovery_entry_queued && tracked_state == protocol::MissionState::descent &&
        cutoff_latched && liftoff_valid && elapsed_us >= 120'000'000) {
      RecoveryRequest recovery_request{};
      recovery_request.kind = RecoveryRequest::Kind::enter;
      recovery_request.reason = protocol::RecoveryModeReason::auto_elapsed_120;
      recovery_entry_queued =
          xQueueSend(recovery_queue, &recovery_request, 0) == pdTRUE;
    }
    resetWatchdog();
    vTaskDelay(1);
  }
}

void parachuteTask(void *) {
  addWatchdog();
  STSCREATE bus;
  STS3215 servo;
  actuators::ParachuteController controller;
  actuators::ParachuteConfigurationState configuration;
  bool mission_state_known = false;
  bool restored_flight = false;
  if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    mission_state_known = true;
    const auto state = state_machine.snapshot().state;
    restored_flight = state == protocol::MissionState::engine_burn ||
                      state == protocol::MissionState::control ||
                      state == protocol::MissionState::descent;
    xSemaphoreGive(state_mutex);
  }
  if (restored_flight) {
    actuators::FlightParachuteConfiguration restored{};
    if (recovery_boot::loadFlightParachuteConfiguration(restored))
      configuration.restoreFlightSnapshot(restored);
  } else if (mission_state_known) {
    recovery_boot::clearFlightParachuteConfiguration();
  }

  enum class DesiredState : uint8_t { powered_off, free, holding, open };
  // boot直後のGPIO安全化後はCommandReceiveからHold要求を維持する。
  DesiredState desired = DesiredState::holding;
  enum class OperationStage : uint8_t {
    initialize,
    moving,
    waiting_persistence,
    finishing,
  };
  struct PendingOperation {
    bool active{};
    bool interrupted{};
    ParachuteCommandRequest request{};
    OperationStage stage{OperationStage::initialize};
    uint64_t started_at_us{};
    protocol::CommandReason result{protocol::CommandReason::none};
    uint32_t detail{};
    std::optional<actuators::AbsoluteParachuteAngle> target{};
    actuators::ParachuteConfiguration candidate{};
  } pending;
  uint32_t active_epoch = 0;
  uint64_t power_enabled_at_us = 0;
  uint64_t next_initialization_attempt_us = 0;
  uint64_t next_power_request_us = 0;
  uint64_t open_requested_at_us = 0;
  uint64_t last_position_read_attempt_us = 0;
  uint64_t last_position_valid_us = 0;
  bool power_enabled = false;
  bool mode_prepared = false;
  bool controller_started = false;
  bool hold_established = false;
  esp_err_t last_initialization_error = ESP_ERR_INVALID_STATE;

  constexpr uint32_t kDetailExactHalfTurn = 1;
  constexpr uint32_t kDetailInvalidPosition = 2;
  constexpr uint32_t kDetailConfigurationLoad = 3;
  constexpr uint32_t kDetailQueueUnavailable = 4;
  constexpr uint32_t kDetailCurrentOpenHalfTurn = 5;
  constexpr uint64_t kPositionTelemetryPollIntervalUs = 500'000;
  constexpr uint64_t kPositionTelemetryStaleUs =
      2 * kPositionTelemetryPollIntervalUs;
  const int16_t target_tolerance_count = static_cast<int16_t>(std::ceil(
      flight_config::kParachute.target_tolerance_deg /
      actuators::kParachuteDegreesPerCount));

  auto queuePower = [](const PowerRequest &request) {
    if (xQueueSend(power_queue, &request, 0) == pdTRUE)
      return true;
    // queue飽和時も電源OFF要求だけはGPIOへ直接反映する。
    if (request.cutoff || !request.parachute_power)
      (void)setTrackedParaPower(false);
    if (request.cutoff || !request.auxiliary_5v)
      (void)bringup::safe_outputs::setAux5v(false);
    return false;
  };

  auto powerOff = [&](bool latch_cutoff, bool preserve_auxiliary_5v,
                      protocol::ParaMode final_mode) {
    if (servo.initialized()) {
      (void)servo.disableTorque();
      (void)servo.end();
    }
    if (bus.initialized())
      (void)bus.end();
    const PowerRequest power{preserve_auxiliary_5v && !latch_cutoff, false,
                             latch_cutoff};
    (void)queuePower(power);
    // OFF要求を出した時点からUART ownerは給電済みと仮定しない。
    parachute_power_applied.store(false, std::memory_order_release);
    controller.notifyPowerCutoff();
    sts_ready.store(false, std::memory_order_release);
    para_mode_actual.store(final_mode, std::memory_order_release);
    desired = DesiredState::powered_off;
    active_epoch = 0;
    power_enabled_at_us = 0;
    next_initialization_attempt_us = 0;
    next_power_request_us = 0;
    open_requested_at_us = 0;
    power_enabled = false;
    mode_prepared = false;
    controller_started = false;
    hold_established = false;
    last_initialization_error = ESP_ERR_INVALID_STATE;
  };

  auto reconnectIntervalMs = [&]() {
    protocol::MissionState state = protocol::MissionState::unknown;
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      state = state_machine.snapshot().state;
      xSemaphoreGive(state_mutex);
    }
    return state == protocol::MissionState::command_receive
               ? flight_config::kParachuteCommandReceiveReconnectMs
               : flight_config::kParachute.retry_interval_ms;
  };

  auto resetServoTransport = [&]() {
    if (servo.initialized())
      (void)servo.end();
    if (bus.initialized())
      (void)bus.end();
    mode_prepared = false;
    controller_started = false;
    hold_established = false;
    sts_ready.store(false, std::memory_order_release);
    last_initialization_error = ESP_ERR_INVALID_STATE;
  };

  auto requestPower = [&](uint64_t now_us) {
    if (parachute_power_applied.load(std::memory_order_acquire)) {
      if (!power_enabled) {
        // SafetyTaskのGPIO44適用成功を観測してから安定待ちを開始する。
        power_enabled = true;
        power_enabled_at_us = now_us;
        next_initialization_attempt_us =
            now_us + static_cast<uint64_t>(
                         flight_config::kParachute.power_stabilization_ms) *
                         1'000ULL;
      }
      next_power_request_us = 0;
      return true;
    }

    if (power_enabled) {
      // 別ownerによるOFFまたはGPIO再設定を検出したらstale transportを破棄する。
      resetServoTransport();
      power_enabled = false;
      power_enabled_at_us = 0;
      next_initialization_attempt_us = 0;
    }

    if (now_us < next_power_request_us)
      return true;

    const uint64_t retry_us =
        static_cast<uint64_t>(reconnectIntervalMs()) * 1'000ULL;
    const PowerRequest power{true, true, false};
    if (!queuePower(power)) {
      next_power_request_us = now_us + retry_us;
      return false;
    }
    // queue投入だけでは給電済みにしない。SafetyTaskの適用結果を待つ。
    next_power_request_us = now_us + retry_us;
    return true;
  };

  auto initializeServo = [&](uint64_t now_us) {
    if (!parachute_power_applied.load(std::memory_order_acquire) ||
        !power_enabled ||
        now_us < power_enabled_at_us +
                     static_cast<uint64_t>(
                         flight_config::kParachute.power_stabilization_ms) *
                         1'000ULL ||
        now_us < next_initialization_attempt_us)
      return false;
    next_initialization_attempt_us =
        now_us + static_cast<uint64_t>(reconnectIntervalMs()) * 1'000ULL;

    if (!bus.initialized()) {
      STSCREATE::Config config{};
      config.port = board::kParaUart;
      config.tx = board::kParaTx;
      config.rx = board::kParaRx;
      config.direction_enable = board::kParaDirectionEnable;
      config.direction_polarity = STSCREATE::DirectionPolarity::tx_high;
      config.baudrate = STSCREATE::Baudrate::bps1000000;
      config.lock_timeout = avi::Timeout::noWait();
      config.tx_timeout =
          avi::Timeout::milliseconds(board::kParaTxTimeoutMs);
      config.response_timeout =
          avi::Timeout::milliseconds(board::kParaResponseTimeoutMs);
      last_initialization_error = bus.begin(config);
      if (last_initialization_error != ESP_OK)
        return false;
    }
    if (!servo.initialized()) {
      last_initialization_error = servo.begin(bus, board::kParaServoId);
      if (last_initialization_error != ESP_OK)
        return false;
    }
    if (!servo.configurationValid()) {
      // 不完全な初期化状態を保持せず、次回retryでbeginからやり直す。
      (void)servo.end();
      last_initialization_error = ESP_ERR_INVALID_RESPONSE;
      return false;
    }
    if (!mode_prepared) {
      last_initialization_error =
          servo.verifyOperatingMode(STS3215::OperatingMode::step);
      if (last_initialization_error != ESP_OK)
        last_initialization_error = servo.configureStepMode(
            STS3215::Persistence::volatile_only);
      if (last_initialization_error == ESP_OK)
        last_initialization_error = servo.setFeedbackMode(
            STS3215::FeedbackMode::single_turn,
            STS3215::Persistence::volatile_only);
      if (last_initialization_error != ESP_OK)
        return false;
      mode_prepared =
          servo.verifyOperatingMode(STS3215::OperatingMode::step) ==
          ESP_OK;
    }
    last_initialization_error =
        mode_prepared ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
    sts_ready.store(mode_prepared, std::memory_order_release);
    return mode_prepared;
  };

  auto motion = []() {
    STS3215::Motion result{};
    result.speed_deg_s = flight_config::kParachute.speed_deg_s;
    result.acceleration_deg_s2 =
        flight_config::kParachute.acceleration_deg_s2;
    result.torque_limit = STS3215::TorqueLimit::percent(
        flight_config::kParachute.torque_limit_percent);
    return result;
  };

  auto updateConfigurationMirrors = [&]() {
    parachute_open_configured.store(configuration.active().openConfigured(),
                                    std::memory_order_release);
    parachute_close_configured.store(configuration.active().closeConfigured(),
                                     std::memory_order_release);
  };

  auto commandReasonForSts = [](esp_err_t result) {
    return result == ESP_ERR_TIMEOUT ? protocol::CommandReason::timeout
                                     : protocol::CommandReason::device_unavailable;
  };

  auto readCurrent = [&](actuators::AbsoluteParachuteAngle &angle,
                         bool &moving) {
    last_position_read_attempt_us =
        static_cast<uint64_t>(esp_timer_get_time());
    STS3215::RawData data{};
    const esp_err_t result = servo.readRaw(data);
    if (result != ESP_OK) {
      // Vaultのsemantic方針に従い、freshなvalid値は単発errorで捨てない。
      if (parachute_angle_raw.load(std::memory_order_acquire) > 240U) {
        const auto error =
            result == ESP_ERR_TIMEOUT
                ? protocol::quantization::ParachuteAngleError::uart_timeout
            : result == ESP_ERR_INVALID_STATE
                ? protocol::quantization::ParachuteAngleError::not_initialized
            : result == ESP_ERR_INVALID_RESPONSE
                ? protocol::quantization::ParachuteAngleError::uart_protocol_error
            : result == ESP_ERR_INVALID_ARG
                ? protocol::quantization::ParachuteAngleError::position_invalid
                : protocol::quantization::ParachuteAngleError::device_error_response;
        parachute_angle_raw.store(static_cast<uint8_t>(error),
                                  std::memory_order_release);
      }
      return result;
    }
    const auto current =
        actuators::AbsoluteParachuteAngle::fromCount(data.position);
    if (!current.has_value()) {
      parachute_angle_raw.store(
          static_cast<uint8_t>(
              protocol::quantization::ParachuteAngleError::position_out_of_range),
          std::memory_order_release);
      return ESP_ERR_INVALID_RESPONSE;
    }
    angle = *current;
    moving = data.moving;
    const double degrees = static_cast<double>(current->count()) *
                           actuators::kParachuteDegreesPerCount;
    parachute_angle_raw.store(
        protocol::quantization::encodeParachuteAngle(
            degrees,
            protocol::quantization::ParachuteAngleError::position_invalid),
        std::memory_order_release);
    last_position_valid_us = last_position_read_attempt_us;
    return ESP_OK;
  };

  auto moveToAbsolute = [&](actuators::AbsoluteParachuteAngle current,
                            actuators::AbsoluteParachuteAngle target) {
    const auto displacement =
        actuators::shortestParachuteDisplacement(current, target);
    if (!displacement.valid())
      return ESP_ERR_INVALID_ARG;
    return servo.moveRelativeDegrees(
        static_cast<float>(displacement.degrees()), motion());
  };

  auto requestFinish = [&](protocol::CommandReason reason,
                           uint32_t detail = 0) {
    if (reason != protocol::CommandReason::none &&
        desired == DesiredState::powered_off && power_enabled)
      powerOff(false, true, protocol::ParaMode::powered_off);
    pending.result = reason;
    pending.detail = detail;
    pending.stage = OperationStage::finishing;
  };

  auto finishPending = [&]() {
    if (!pending.active || pending.stage != OperationStage::finishing)
      return;
    if (pending.request.kind ==
        ParachuteCommandRequest::Kind::start_preparation) {
      const ParachuteStartResponse response{
          pending.request.command.transaction_id, pending.result,
          pending.detail};
      if (xQueueSend(parachute_start_response_queue, &response, 0) == pdTRUE)
        pending = {};
      return;
    }
    if (pending.interrupted) {
      pending = {};
      return;
    }
    if (xSemaphoreTake(executor_mutex, 0) != pdTRUE)
      return;
    const auto result = command_executor.finish(
        pending.request.command.transaction_id,
        pending.result == protocol::CommandReason::none
            ? protocol::CommandPhase::completed
            : protocol::CommandPhase::failed,
        pending.result, pending.detail);
    xSemaphoreGive(executor_mutex);
    // Emergencyが先にtransactionを終端した場合は二重resultを送らない。
    if (result.command != 0)
      enqueueResult(result, false);
    pending = {};
  };

  for (;;) {
    if (recovery_requested.load(std::memory_order_acquire) &&
        desired != DesiredState::powered_off)
      powerOff(false, false, protocol::ParaMode::powered_off);

    ParachutePersistenceResponse persistence_response{};
    while (xQueueReceive(parachute_persistence_response_queue,
                         &persistence_response, 0) == pdTRUE) {
      if (persistence_response.kind ==
          ParachutePersistenceResponse::Kind::load) {
        actuators::ParachuteConfiguration loaded{};
        if (persistence_response.open_valid)
          loaded.open = actuators::AbsoluteParachuteAngle::fromCount(
              persistence_response.open_count);
        if (persistence_response.close_valid)
          loaded.close = actuators::AbsoluteParachuteAngle::fromCount(
              persistence_response.close_count);
        configuration.replaceLoadedConfiguration(loaded);
        updateConfigurationMirrors();
        parachute_persistence_ready.store(
            persistence_response.persistence_ready,
            std::memory_order_release);
        parachute_persistence_corrupt.store(
            persistence_response.corruption_detected, std::memory_order_release);
        parachute_config_load_complete.store(true, std::memory_order_release);
        continue;
      }
      if (!pending.active ||
          pending.stage != OperationStage::waiting_persistence ||
          pending.request.command.transaction_id !=
              persistence_response.transaction_id)
        continue;
      if (persistence_response.success) {
        configuration.activatePersistedCandidate(pending.candidate);
        updateConfigurationMirrors();
        if (!pending.interrupted) {
          desired = DesiredState::holding;
          para_mode_actual.store(protocol::ParaMode::hold,
                                 std::memory_order_release);
        }
        requestFinish(protocol::CommandReason::none);
      } else {
        if (!pending.interrupted) {
          desired = DesiredState::holding;
          hold_established = false;
        }
        requestFinish(protocol::CommandReason::persistence_error);
      }
    }

    ParaRequest request{};
    while (xQueueReceive(para_queue, &request, 0) == pdTRUE) {
      if (request.kind == ParaRequest::Kind::free ||
          request.kind == ParaRequest::Kind::power_off ||
          request.kind == ParaRequest::Kind::discard_snapshot) {
        bool current_request = request.safety_authorized ||
                               request.flight_epoch == 0;
        if (!current_request && xSemaphoreTake(state_mutex, 0) == pdTRUE) {
          const auto snapshot = state_machine.snapshot();
          current_request = request.flight_epoch == snapshot.flight_epoch;
          xSemaphoreGive(state_mutex);
        } else if (!current_request) {
          (void)xQueueSendToFront(para_queue, &request, 0);
          break;
        }
        if (current_request) {
          if (request.kind == ParaRequest::Kind::discard_snapshot) {
            configuration.discardFlightSnapshot();
            recovery_boot::clearFlightParachuteConfiguration();
            desired = DesiredState::holding;
            hold_established = false;
            continue;
          }
          const bool free_requested = request.kind == ParaRequest::Kind::free;
          const bool absolute_cutoff =
              request.kind == ParaRequest::Kind::power_off &&
              request.safety_authorized;
          const bool preserve_auxiliary_5v =
              !absolute_cutoff &&
              !recovery_requested.load(std::memory_order_acquire);
          if (free_requested && pending.active) {
            if (pending.request.kind ==
                ParachuteCommandRequest::Kind::start_preparation) {
              requestFinish(
                  protocol::CommandReason::interrupted_by_emergency);
            } else {
              pending.interrupted = true;
              if (pending.stage != OperationStage::waiting_persistence)
                pending = {};
            }
          }
          powerOff(absolute_cutoff, preserve_auxiliary_5v,
                   free_requested ? protocol::ParaMode::free
                                  : protocol::ParaMode::powered_off);
          if (free_requested) {
            desired = DesiredState::free;
            next_power_request_us =
                static_cast<uint64_t>(esp_timer_get_time()) +
                static_cast<uint64_t>(
                    flight_config::kParachuteCommandReceiveReconnectMs) *
                    1'000ULL;
          }
        }
        continue;
      }

      bool current_request = false;
      if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
        const auto snapshot = state_machine.snapshot();
        current_request = request.flight_epoch != 0 &&
                          request.flight_epoch == snapshot.flight_epoch &&
                          (request.kind != ParaRequest::Kind::open ||
                           request.safety_authorized ||
                           snapshot.state == protocol::MissionState::descent);
        xSemaphoreGive(state_mutex);
      } else {
        (void)xQueueSendToFront(para_queue, &request, 0);
        break;
      }
      if (!current_request)
        continue;

      const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
      const bool same_epoch = active_epoch == request.flight_epoch;
      if (same_epoch && desired == DesiredState::open)
        continue;
      if (!configuration.flightSnapshotValid()) {
        std::printf("parachute open failed: flight snapshot unavailable\n");
        uint8_t expected = 0;
        if (parachute_deployment_failure.compare_exchange_strong(
                expected, static_cast<uint8_t>(
                              ParachuteDeploymentFailure::open_not_configured))) {
          enqueueEvent(protocol::eventFlag(
                           protocol::MissionEventFlag::parachute_deployment_failure),
                       protocol::MissionState::descent, 0,
                       static_cast<uint16_t>(
                           ParachuteDeploymentFailure::open_not_configured));
        }
        desired = DesiredState::holding;
        hold_established = false;
        continue;
      }
      active_epoch = request.flight_epoch;
      desired = DesiredState::open;
      open_requested_at_us = now_us;
      controller = actuators::ParachuteController{};
      controller_started = false;
      if (requestPower(now_us)) {
        para_mode_actual.store(protocol::ParaMode::opening_or_retrying,
                               std::memory_order_release);
      } else {
        para_mode_actual.store(protocol::ParaMode::powered_off,
                               std::memory_order_release);
      }
    }

    if (!pending.active) {
      ParachuteCommandRequest command_request{};
      if (xQueueReceive(parachute_command_queue, &command_request, 0) ==
          pdTRUE) {
        pending.active = true;
        pending.request = command_request;
        pending.started_at_us =
            static_cast<uint64_t>(esp_timer_get_time());
        const auto code = static_cast<mission::CommandCode>(
            command_request.command.command);
        if (command_request.kind == ParachuteCommandRequest::Kind::generic &&
            code == mission::CommandCode::para_free) {
          desired = DesiredState::free;
          hold_established = false;
          if (servo.initialized())
            (void)servo.disableTorque();
          para_mode_actual.store(protocol::ParaMode::free,
                                 std::memory_order_release);
          requestFinish(protocol::CommandReason::none);
        } else if (command_request.kind ==
                       ParachuteCommandRequest::Kind::generic &&
                   (code == mission::CommandCode::para_open ||
                    code == mission::CommandCode::para_close) &&
                   !parachute_config_load_complete.load(
                       std::memory_order_acquire)) {
          requestFinish(protocol::CommandReason::busy,
                        kDetailConfigurationLoad);
        } else if (command_request.kind ==
                       ParachuteCommandRequest::Kind::generic &&
                   (code == mission::CommandCode::para_open ||
                    code == mission::CommandCode::para_close) &&
                   !parachute_persistence_ready.load(
                       std::memory_order_acquire)) {
          requestFinish(protocol::CommandReason::persistence_error,
                        kDetailConfigurationLoad);
        } else if (command_request.kind ==
                       ParachuteCommandRequest::Kind::generic &&
                   code == mission::CommandCode::para_open &&
                   !configuration.active().openConfigured()) {
          requestFinish(protocol::CommandReason::not_configured);
        } else if (command_request.kind ==
                       ParachuteCommandRequest::Kind::generic &&
                   code == mission::CommandCode::para_close &&
                   !configuration.active().closeConfigured()) {
          requestFinish(protocol::CommandReason::not_configured);
        } else if (command_request.kind ==
                       ParachuteCommandRequest::Kind::start_preparation &&
                   !parachute_config_load_complete.load(
                       std::memory_order_acquire)) {
          requestFinish(protocol::CommandReason::busy,
                        kDetailConfigurationLoad);
        } else if (command_request.kind ==
                       ParachuteCommandRequest::Kind::start_preparation &&
                   !parachute_persistence_ready.load(
                       std::memory_order_acquire)) {
          requestFinish(protocol::CommandReason::persistence_error,
                        kDetailConfigurationLoad);
        } else if (command_request.kind ==
                       ParachuteCommandRequest::Kind::start_preparation &&
                   code == mission::CommandCode::start_sequence &&
                   command_request.readiness.missingMask() != 0) {
          requestFinish(protocol::CommandReason::not_configured,
                        command_request.readiness.missingMask());
        } else if (!requestPower(pending.started_at_us)) {
          requestFinish(protocol::CommandReason::busy,
                        kDetailQueueUnavailable);
        }
      }
    }

    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (pending.active && pending.stage == OperationStage::initialize) {
      const bool power_request_ok = requestPower(now_us);
      if (!power_request_ok) {
        requestFinish(protocol::CommandReason::busy, kDetailQueueUnavailable);
      } else if (initializeServo(now_us)) {
        auto current = *actuators::AbsoluteParachuteAngle::fromCount(0);
        bool moving = false;
        const esp_err_t read = readCurrent(current, moving);
        sts_ready.store(read == ESP_OK, std::memory_order_release);
        if (pending.request.kind ==
            ParachuteCommandRequest::Kind::start_preparation) {
          const auto code = static_cast<mission::CommandCode>(
              pending.request.command.command);
          const bool forced =
              code == mission::CommandCode::force_start_sequence;
          if (!parachute_config_load_complete.load(
                  std::memory_order_acquire)) {
            requestFinish(protocol::CommandReason::busy,
                          kDetailConfigurationLoad);
          } else if (!parachute_persistence_ready.load(
                         std::memory_order_acquire)) {
            requestFinish(protocol::CommandReason::persistence_error,
                          kDetailConfigurationLoad);
          } else if (!forced && pending.request.readiness.missingMask() != 0) {
            requestFinish(protocol::CommandReason::not_configured,
                          pending.request.readiness.missingMask());
          } else if (forced) {
            if (read == ESP_OK) {
              const esp_err_t hold = servo.holdCurrentPosition(
                  {STS3215::TorqueLimit::percent(
                      flight_config::kParachute.torque_limit_percent)});
              sts_ready.store(hold == ESP_OK, std::memory_order_release);
              hold_established = hold == ESP_OK;
              if (hold == ESP_OK) {
                desired = DesiredState::holding;
                para_mode_actual.store(protocol::ParaMode::hold,
                                       std::memory_order_release);
              }
            } else {
              sts_ready.store(false, std::memory_order_release);
              hold_established = false;
              desired = DesiredState::holding;
            }
            configuration.freezeFlightSnapshotForced();
            recovery_boot::storeFlightParachuteConfiguration(
                *configuration.flightSnapshot());
            requestFinish(protocol::CommandReason::none,
                          pending.request.readiness.missingMask());
          } else if (read != ESP_OK) {
            requestFinish(commandReasonForSts(read), kDetailInvalidPosition);
          } else {
            const esp_err_t hold = servo.holdCurrentPosition(
                {STS3215::TorqueLimit::percent(
                    flight_config::kParachute.torque_limit_percent)});
            sts_ready.store(hold == ESP_OK, std::memory_order_release);
            if (hold != ESP_OK) {
              requestFinish(commandReasonForSts(hold));
            } else {
              desired = DesiredState::holding;
              hold_established = true;
              para_mode_actual.store(protocol::ParaMode::hold,
                                     std::memory_order_release);
              const auto prepared = configuration.freezeFlightSnapshot(current);
              if (!prepared.ready()) {
                if (prepared.error ==
                    actuators::FlightParachutePreparationError::
                        current_open_exactly_half_turn)
                  requestFinish(protocol::CommandReason::safety_interlock,
                                kDetailCurrentOpenHalfTurn);
                else
                  requestFinish(protocol::CommandReason::not_configured,
                                pending.request.readiness.missingMask());
              } else {
                recovery_boot::storeFlightParachuteConfiguration(
                    *configuration.flightSnapshot());
                requestFinish(protocol::CommandReason::none);
              }
            }
          }
        } else if (read != ESP_OK) {
          requestFinish(commandReasonForSts(read), kDetailInvalidPosition);
        } else {
          const auto code = static_cast<mission::CommandCode>(
              pending.request.command.command);
          if (code == mission::CommandCode::para_hold) {
            const esp_err_t hold = servo.holdCurrentPosition(
                {STS3215::TorqueLimit::percent(
                    flight_config::kParachute.torque_limit_percent)});
            sts_ready.store(hold == ESP_OK, std::memory_order_release);
            hold_established = hold == ESP_OK;
            if (hold_established) {
              desired = DesiredState::holding;
              para_mode_actual.store(protocol::ParaMode::hold,
                                     std::memory_order_release);
            }
            requestFinish(hold == ESP_OK ? protocol::CommandReason::none
                                         : commandReasonForSts(hold));
          } else if (code == mission::CommandCode::set_para_open ||
                     code == mission::CommandCode::set_para_close) {
            if (!parachute_config_load_complete.load(
                    std::memory_order_acquire)) {
              requestFinish(protocol::CommandReason::busy,
                            kDetailConfigurationLoad);
            } else if (!parachute_persistence_ready.load(
                           std::memory_order_acquire)) {
              requestFinish(protocol::CommandReason::persistence_error,
                            kDetailConfigurationLoad);
            } else {
              const auto endpoint =
                  code == mission::CommandCode::set_para_open
                      ? actuators::ParachuteEndpoint::open
                      : actuators::ParachuteEndpoint::close;
              pending.candidate = configuration.candidateWith(endpoint, current);
              const auto previous = endpoint == actuators::ParachuteEndpoint::open
                                        ? configuration.active().open
                                        : configuration.active().close;
              const ParachutePersistenceRequest save{
                  pending.request.command.transaction_id, endpoint,
                  current.count(), previous.has_value(),
                  previous.has_value() ? previous->count() : uint16_t{0}};
              if (xQueueSend(parachute_persistence_request_queue, &save, 0) ==
                  pdTRUE) {
                pending.stage = OperationStage::waiting_persistence;
              } else {
                requestFinish(protocol::CommandReason::busy,
                              kDetailQueueUnavailable);
              }
            }
          } else if (code == mission::CommandCode::para_open ||
                     code == mission::CommandCode::para_close) {
            pending.target = code == mission::CommandCode::para_open
                                 ? configuration.active().open
                                 : configuration.active().close;
            if (!pending.target.has_value()) {
              requestFinish(protocol::CommandReason::not_configured);
            } else {
              const auto displacement =
                  actuators::shortestParachuteDisplacement(current,
                                                           *pending.target);
              if (!displacement.valid()) {
                requestFinish(protocol::CommandReason::safety_interlock,
                              kDetailExactHalfTurn);
              } else {
                const esp_err_t move = moveToAbsolute(current, *pending.target);
                sts_ready.store(move == ESP_OK, std::memory_order_release);
                if (move == ESP_OK) {
                  pending.stage = OperationStage::moving;
                  para_mode_actual.store(
                      code == mission::CommandCode::para_open
                          ? protocol::ParaMode::opening_or_retrying
                          : protocol::ParaMode::closing,
                      std::memory_order_release);
                } else {
                  requestFinish(commandReasonForSts(move));
                }
              }
            }
          } else if (code == mission::CommandCode::para_move_relative) {
            const uint16_t raw =
                static_cast<uint16_t>(pending.request.command.arguments[0]) |
                static_cast<uint16_t>(pending.request.command.arguments[1])
                    << 8U;
            const int16_t tenths = static_cast<int16_t>(raw);
            const esp_err_t move = servo.moveRelativeDegrees(
                static_cast<float>(tenths) * 0.1F, motion());
            sts_ready.store(move == ESP_OK, std::memory_order_release);
            if (move == ESP_OK) {
              pending.stage = OperationStage::moving;
              para_mode_actual.store(protocol::ParaMode::relative_move,
                                     std::memory_order_release);
            } else {
              requestFinish(commandReasonForSts(move));
            }
          } else {
            requestFinish(protocol::CommandReason::not_supported);
          }
        }
      } else if (now_us >= pending.started_at_us &&
                 now_us - pending.started_at_us >=
                     static_cast<uint64_t>(
                         flight_config::kParachute.initialization_deadline_ms) *
                         1'000ULL) {
        const bool forced_start =
            pending.request.kind == ParachuteCommandRequest::Kind::start_preparation &&
            static_cast<mission::CommandCode>(pending.request.command.command) ==
                mission::CommandCode::force_start_sequence;
        if (forced_start &&
            parachute_config_load_complete.load(std::memory_order_acquire) &&
            parachute_persistence_ready.load(std::memory_order_acquire)) {
          configuration.freezeFlightSnapshotForced();
          recovery_boot::storeFlightParachuteConfiguration(
              *configuration.flightSnapshot());
          desired = DesiredState::holding;
          hold_established = false;
          requestFinish(protocol::CommandReason::none,
                        pending.request.readiness.missingMask());
        } else {
          requestFinish(commandReasonForSts(last_initialization_error));
        }
      }
    }

    if (pending.active && pending.stage == OperationStage::moving) {
      auto current = *actuators::AbsoluteParachuteAngle::fromCount(0);
      bool moving = false;
      const esp_err_t read = readCurrent(current, moving);
      sts_ready.store(read == ESP_OK, std::memory_order_release);
      if (read != ESP_OK) {
        requestFinish(commandReasonForSts(read), kDetailInvalidPosition);
      } else {
        bool reached = !moving;
        if (pending.target.has_value()) {
          const auto displacement = actuators::shortestParachuteDisplacement(
              current, *pending.target);
          if (!displacement.valid()) {
            requestFinish(protocol::CommandReason::safety_interlock,
                          kDetailExactHalfTurn);
            reached = false;
          } else {
            reached = reached &&
                      displacement.counts <= target_tolerance_count &&
                      displacement.counts >= -target_tolerance_count;
          }
        }
        if (reached) {
          const esp_err_t hold = servo.holdCurrentPosition(
              {STS3215::TorqueLimit::percent(
                  flight_config::kParachute.torque_limit_percent)});
          sts_ready.store(hold == ESP_OK, std::memory_order_release);
          if (hold == ESP_OK) {
            desired = DesiredState::holding;
            hold_established = true;
            para_mode_actual.store(protocol::ParaMode::hold,
                                   std::memory_order_release);
          }
          requestFinish(hold == ESP_OK ? protocol::CommandReason::none
                                       : commandReasonForSts(hold));
        } else if (pending.stage == OperationStage::moving &&
                   now_us >= pending.started_at_us &&
                   now_us - pending.started_at_us >= 5'000'000) {
          requestFinish(protocol::CommandReason::timeout);
        }
      }
    }

    auto recordParachuteFailure = [&](ParachuteDeploymentFailure failure,
                                      uint16_t detail = 0) {
      std::printf("parachute deployment failure: code=%u detail=%u\n",
                  static_cast<unsigned>(failure),
                  static_cast<unsigned>(detail));
      uint8_t expected = 0;
      if (parachute_deployment_failure.compare_exchange_strong(
              expected, static_cast<uint8_t>(failure))) {
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::parachute_deployment_failure),
                     protocol::MissionState::descent, 0,
                     static_cast<uint16_t>(failure));
      }
    };

    if (desired == DesiredState::holding && !pending.active) {
      if (!requestPower(now_us)) {
        sts_ready.store(false, std::memory_order_release);
        hold_established = false;
      } else if (!hold_established && initializeServo(now_us)) {
        actuators::AbsoluteParachuteAngle current =
            *actuators::AbsoluteParachuteAngle::fromCount(0);
        bool moving = false;
        const esp_err_t read = readCurrent(current, moving);
        sts_ready.store(read == ESP_OK, std::memory_order_release);
        if (read == ESP_OK) {
          const esp_err_t hold = servo.holdCurrentPosition(
              {STS3215::TorqueLimit::percent(
                  flight_config::kParachute.torque_limit_percent)});
          sts_ready.store(hold == ESP_OK, std::memory_order_release);
          hold_established = hold == ESP_OK;
          if (hold_established) {
            para_mode_actual.store(protocol::ParaMode::hold,
                                   std::memory_order_release);
          } else if (active_epoch != 0) {
            recordParachuteFailure(
                ParachuteDeploymentFailure::hold_failed);
          } else {
            std::printf("parachute hold failed outside deployment\n");
          }
        }
      }
    }

    if (desired == DesiredState::free && !pending.active) {
      if (requestPower(now_us) && initializeServo(now_us)) {
        (void)servo.disableTorque();
        para_mode_actual.store(protocol::ParaMode::free,
                               std::memory_order_release);
      }
    }

    if (desired == DesiredState::open && !pending.active) {
      const auto *snapshot = configuration.flightSnapshot();
      if (snapshot == nullptr || !snapshot->open.has_value()) {
        recordParachuteFailure(ParachuteDeploymentFailure::open_not_configured);
        desired = DesiredState::holding;
        hold_established = false;
      } else if (!requestPower(now_us)) {
        sts_ready.store(false, std::memory_order_release);
      } else if (initializeServo(now_us)) {
        auto current = *actuators::AbsoluteParachuteAngle::fromCount(0);
        bool moving = false;
        const esp_err_t read = readCurrent(current, moving);
        sts_ready.store(read == ESP_OK, std::memory_order_release);
        if (read == ESP_OK) {
          const auto displacement = actuators::shortestParachuteDisplacement(
              current, *snapshot->open);
          if (!displacement.valid()) {
            recordParachuteFailure(
                ParachuteDeploymentFailure::ambiguous_half_turn);
            desired = DesiredState::holding;
            hold_established = false;
          } else if (!controller_started) {
            controller_started =
                controller.startOpen(open_requested_at_us, current.count()) ==
                actuators::ParachuteAction::command_open;
            if (controller_started &&
                moveToAbsolute(current, *snapshot->open) != ESP_OK) {
              sts_ready.store(false, std::memory_order_release);
              recordParachuteFailure(
                  ParachuteDeploymentFailure::move_command_failed);
            }
          }

          if (controller_started && desired == DesiredState::open) {
            const auto latest_displacement =
                actuators::shortestParachuteDisplacement(current,
                                                         *snapshot->open);
            const bool reached = latest_displacement.valid() && !moving &&
                                 latest_displacement.counts <=
                                     target_tolerance_count &&
                                 latest_displacement.counts >=
                                     -target_tolerance_count;
            const auto action = controller.tick(
                {now_us, true, current.count(), reached});
            if (action == actuators::ParachuteAction::retry_open) {
              const auto retry_displacement =
                  actuators::shortestParachuteDisplacement(current,
                                                           *snapshot->open);
              if (!retry_displacement.valid()) {
                recordParachuteFailure(
                    ParachuteDeploymentFailure::ambiguous_half_turn);
                desired = DesiredState::holding;
                hold_established = false;
              } else if (moveToAbsolute(current, *snapshot->open) != ESP_OK) {
                sts_ready.store(false, std::memory_order_release);
                recordParachuteFailure(
                    ParachuteDeploymentFailure::move_command_failed);
              }
            } else if (action == actuators::ParachuteAction::hold_open) {
              const esp_err_t hold = servo.holdCurrentPosition(
                  {STS3215::TorqueLimit::percent(
                      flight_config::kParachute.torque_limit_percent)});
              hold_established = hold == ESP_OK;
              if (!hold_established)
                recordParachuteFailure(
                    ParachuteDeploymentFailure::hold_failed);
              desired = DesiredState::holding;
              para_mode_actual.store(protocol::ParaMode::hold,
                                     std::memory_order_release);
            } else if (action ==
                       actuators::ParachuteAction::stop_retrying) {
              recordParachuteFailure(
                  ParachuteDeploymentFailure::retry_exhausted);
              const esp_err_t hold = servo.holdCurrentPosition(
                  {STS3215::TorqueLimit::percent(
                      flight_config::kParachute.torque_limit_percent)});
              hold_established = hold == ESP_OK;
              if (!hold_established)
                recordParachuteFailure(
                    ParachuteDeploymentFailure::hold_failed);
              desired = DesiredState::holding;
              para_mode_actual.store(protocol::ParaMode::hold,
                                     std::memory_order_release);
            }
          }
        }
      }

      if (desired == DesiredState::open && open_requested_at_us != 0 &&
          now_us >= open_requested_at_us &&
          now_us - open_requested_at_us >= 5'000'000) {
        recordParachuteFailure(
            sts_ready.load(std::memory_order_acquire)
                ? ParachuteDeploymentFailure::retry_exhausted
                : ParachuteDeploymentFailure::current_angle_unavailable);
        desired = DesiredState::holding;
        hold_established = false;
      }
    }

    if (servo.initialized() && mode_prepared &&
        parachute_power_applied.load(std::memory_order_acquire) &&
        (last_position_read_attempt_us == 0 ||
         (now_us >= last_position_read_attempt_us &&
          now_us - last_position_read_attempt_us >=
              kPositionTelemetryPollIntervalUs))) {
      auto current = *actuators::AbsoluteParachuteAngle::fromCount(0);
      bool moving = false;
      const esp_err_t read = readCurrent(current, moving);
      sts_ready.store(read == ESP_OK, std::memory_order_release);
      if (read != ESP_OK)
        resetServoTransport();
    }

    const uint8_t angle_raw =
        parachute_angle_raw.load(std::memory_order_acquire);
    if (angle_raw <= 240U && last_position_valid_us != 0 &&
        parachute_power_applied.load(std::memory_order_acquire) &&
        now_us >= last_position_valid_us &&
        now_us - last_position_valid_us >= kPositionTelemetryStaleUs) {
      parachute_angle_raw.store(
          static_cast<uint8_t>(protocol::quantization::ParachuteAngleError::stale),
          std::memory_order_release);
    }

    finishPending();

    if (!servo.initialized())
      sts_ready.store(false, std::memory_order_release);
    resetWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void missionRealtimeTask(void *) {
  std::printf("MissionRealtimeTask start\n");
  addWatchdog();
  bringup::SpiBringup spi;
  static bringup::ImuBringup imu;
  bringup::EncoderBringup encoder;
  static sensors::GyroHistoryRing gyro_history;
  sensors::ImuLiftoffDetector liftoff_detector;
  sensors::AttitudeEstimator attitude;
  sensors::AirspeedGate airspeed_gate;
  control::QuadraticN3FinVelocityEstimator fin_velocity;
  control::ZeroHoldController zero_hold_controller{
      control::ZeroHoldConfig{
          2.32, 0.296,
          board::kControlAuthorityLimits
              .zero_hold_requested_torque_limit_Nm,
          0.017453292519943295, 0.08726646259971647, 100}};
  control::RollController roll_controller{flight_config::kRollGainSchedule};
  control::TorqueMapper torque_mapper{
      flight_config::kActiveFlightMotorProfile, board::kFinSoftwareLimits};
  actuators::ProductionMotorDriver motor_driver;
  bool liftoff_detected = false;
  bool imu_liftoff_latched = false;
  bool lps_liftoff_latched = false;
  bool imu_data_loss_latched = false;
  uint32_t detector_epoch = 0;
  AirDataSnapshot latest_air_data{};
  protocol::MissionState previous_state =
      protocol::MissionState::command_receive;
  bool previous_liftoff = false;
  bool previous_deployment = false;
  bool previous_cutoff = false;
  bool previous_imu_error = false;
  bool previous_encoder_error = false;
  bool previous_air_data_error = false;
  bool previous_motor_saturation = false;
  bool previous_reset_invalidated = false;
  uint64_t last_imu_host_sample_us = 0;
  uint64_t last_imu_recovery_attempt_us = 0;
  bool timestamp_offset_valid = false;
  uint64_t sensor_to_host_offset_us = 0;
  uint32_t attitude_epoch = 0;
  bool fin_angle_available = false;
  bool fin_zero_available = false;
  bool command_fin_auto_hold_allowed = true;
  bool command_fin_boot_hold_active = false;
  bool command_fin_target_is_unwrapped = false;
  uint16_t latest_encoder_angle_raw = flight_log::kUnknownEncoderZeroCount;
  flight_runtime_metadata::FinZeroMetadata fin_zero_metadata{};
  flight_runtime_metadata::EncoderTimingMetadata encoder_timing{};
  enum class CommandFinMode : uint8_t {
    free,
    zero_hold,
    position_hold,
    relative_move,
  };
  CommandFinMode command_fin_mode = CommandFinMode::free;
  double command_fin_target_rad = 0.0;
  struct PendingFinMove {
    bool active{};
    uint8_t transaction_id{};
    uint64_t deadline_us{};
    double target_rad{};
  } pending_fin_move;
  double previous_wrapped_fin_rad = 0.0;
  double unwrapped_fin_rad = 0.0;
  double fin_zero_reference_rad = 0.0;
  double fin_angle_rad = 0.0;
  double fin_rate_rad_s = 0.0;
  bool fin_rate_valid = false;
  uint64_t control_tick = 0;
  MissionCommandEnvelope pending_start{};
  bool start_preparation_pending = false;
  uint32_t preflight_generation = 0;
  bool preflight_gyro_bias_valid = false;
  bool gravity_reference_valid = false;
  double preflight_gyro_bias_rad_s = 0.0;
  struct CalibrationState {
    bool active{};
    uint8_t transaction_id{};
    uint64_t started_at_us{};
    uint32_t gyro_samples{};
    uint32_t accel_samples{};
    double gyro_sum_rad_s{};
    double accel_sum_x_g{};
    double accel_sum_y_g{};
    double accel_sum_z_g{};
  } calibration;
  std::printf("MissionRealtimeTask spi begin start\n");
  const esp_err_t spi_result = spi.begin();
  std::printf("MissionRealtimeTask spi begin result=%s\n",
              esp_err_to_name(spi_result));
  std::printf("MissionRealtimeTask imu begin start\n");
  const esp_err_t imu_result =
      spi_result == ESP_OK ? imu.begin(spi, true) : ESP_ERR_INVALID_STATE;
  std::printf("MissionRealtimeTask imu begin result=%s\n",
              esp_err_to_name(imu_result));
  std::printf("MissionRealtimeTask encoder begin start\n");
  const esp_err_t encoder_result =
      spi_result == ESP_OK ? encoder.begin(spi) : ESP_ERR_INVALID_STATE;
  std::printf("MissionRealtimeTask encoder begin result=%s\n",
              esp_err_to_name(encoder_result));
  AS5047D::Status encoder_status{};
  esp_err_t encoder_status_result =
      encoder_result == ESP_OK ? encoder.getStatus(encoder_status)
                               : ESP_ERR_INVALID_STATE;
  if (encoder_status_result == ESP_OK &&
      sensors::as5047d_health::statusFaulted(encoder_status))
    encoder_status_result = ESP_ERR_INVALID_RESPONSE;
  std::printf("MissionRealtimeTask encoder status result=%s\n",
              esp_err_to_name(encoder_status_result));
  const esp_err_t pipeline_result =
      encoder_status_result == ESP_OK ? encoder.startPipelinedRead()
                                      : ESP_ERR_INVALID_STATE;
  if (encoder_result == ESP_OK && encoder_status_result != ESP_OK)
    (void)encoder.end();
  imu_ready.store(imu_result == ESP_OK, std::memory_order_release);
  encoder_ready.store(pipeline_result == ESP_OK, std::memory_order_release);
  const uint64_t encoder_initialization_us =
      static_cast<uint64_t>(esp_timer_get_time());
  encoder_timing.pipeline_state =
      pipeline_result == ESP_OK
          ? flight_runtime_metadata::EncoderPipelineState::warming_up
          : flight_runtime_metadata::EncoderPipelineState::faulted;
  encoder_timing.pipeline_ready_timestamp_us =
      pipeline_result == ESP_OK ? encoder_initialization_us : 0;
  flight_runtime_metadata::publishEncoderTiming(encoder_timing);
  uint64_t next_encoder_reconnect_us =
      pipeline_result == ESP_OK ? 0 : encoder_initialization_us + 1'000'000;
  const esp_err_t motor_result = motor_driver.initialize();
  motor_ready.store(motor_result == ESP_OK, std::memory_order_release);
  std::printf("MissionRealtimeTask encoder pipeline result=%s motor=%s\n",
              esp_err_to_name(pipeline_result), esp_err_to_name(motor_result));
  std::printf("MissionRealtimeTask stack free min bytes=%u\n",
              static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));

  uint32_t timestamp_epoch = 1;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, 1);
    if (recovery_requested.load(std::memory_order_acquire)) {
      recovery_motor_safe.store(motor_driver.coast() == ESP_OK,
                                std::memory_order_release);
    }
    EmergencyEnvelope emergency{};
    while (xQueueReceive(emergency_queue, &emergency, 0) == pdTRUE) {
      if (xSemaphoreTake(executor_mutex, 0) != pdTRUE) {
        if (xQueueSendToFront(emergency_queue, &emergency, 0) != pdTRUE)
          result_queue_overflow.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      if (xSemaphoreTake(state_mutex, 0) != pdTRUE) {
        xSemaphoreGive(executor_mutex);
        if (xQueueSendToFront(emergency_queue, &emergency, 0) != pdTRUE)
          result_queue_overflow.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      if (emergency.liftoff_detection) {
        const auto transition =
            state_machine.liftoffDetectionEmergencyStop();
        const auto event_state = state_machine.snapshot().state;
        const auto result = command_executor.liftoffEmergencyResult(
            emergency.transaction_id,
            transition == mission::TransitionResult::completed);
        xSemaphoreGive(state_mutex);
        xSemaphoreGive(executor_mutex);
        enqueueResult(result, true);
        if (transition == mission::TransitionResult::completed)
          enqueueEvent(protocol::eventFlag(
                           protocol::MissionEventFlag::
                               liftoff_emergency_rollback),
                       event_state);
      } else {
        const auto event_state = state_machine.snapshot().state;
        const auto decision = command_executor.actuatorEmergency(
            emergency.transaction_id, event_state);
        xSemaphoreGive(state_mutex);
        xSemaphoreGive(executor_mutex);
        enqueueResult(decision.result, true);
        if (decision.execute) {
          actuator_output_inhibited.store(true, std::memory_order_release);
          pending_fin_move = {};
          command_fin_mode = CommandFinMode::free;
          command_fin_auto_hold_allowed = false;
          command_fin_boot_hold_active = false;
          command_fin_target_is_unwrapped = false;
          (void)motor_driver.coast();
          const PowerRequest power{true, false, false};
          const ParaRequest para{ParaRequest::Kind::free, 0, false};
          (void)xQueueSend(power_queue, &power, 0);
          (void)xQueueSendToFront(para_queue, &para, 0);
          for (std::size_t index = 0; index < decision.interrupted_count;
               ++index)
            enqueueResult(decision.interrupted[index], true);
          enqueueEvent(protocol::eventFlag(
                           protocol::MissionEventFlag::actuator_emergency_stop),
                       event_state);
        }
      }
    }
    uint8_t emergency_transaction = 0;
    if (liftoff_emergency_latch.take(emergency_transaction)) {
      const EmergencyEnvelope envelope{emergency_transaction, true};
      if (xQueueSendToFront(emergency_queue, &envelope, 0) != pdTRUE) {
        if (liftoff_emergency_latch.signal(emergency_transaction))
          emergency_metadata_overflow.fetch_add(1,
                                                std::memory_order_relaxed);
      }
    }
    if (actuator_emergency_latch.take(emergency_transaction)) {
      const EmergencyEnvelope envelope{emergency_transaction, false};
      if (xQueueSendToFront(emergency_queue, &envelope, 0) != pdTRUE) {
        if (actuator_emergency_latch.signal(emergency_transaction))
          emergency_metadata_overflow.fetch_add(1,
                                                std::memory_order_relaxed);
      }
      actuator_output_inhibited.store(true, std::memory_order_release);
      (void)motor_driver.coast();
    }

    ParachuteStartResponse start_response{};
    while (xQueueReceive(parachute_start_response_queue, &start_response, 0) ==
           pdTRUE) {
      if (!start_preparation_pending ||
          start_response.transaction_id !=
              pending_start.request.transaction_id)
        continue;
      if (xSemaphoreTake(executor_mutex, 0) != pdTRUE) {
        (void)xQueueSendToFront(parachute_start_response_queue,
                                &start_response, 0);
        break;
      }
      if (xSemaphoreTake(state_mutex, 0) != pdTRUE) {
        xSemaphoreGive(executor_mutex);
        (void)xQueueSendToFront(parachute_start_response_queue,
                                &start_response, 0);
        break;
      }
      protocol::CommandReason reason = start_response.reason;
      uint32_t final_detail = start_response.detail;
      if (reason == protocol::CommandReason::none) {
        const bool forced =
            static_cast<mission::CommandCode>(pending_start.request.command) ==
            mission::CommandCode::force_start_sequence;
        const auto transition = state_machine.startSequence(
            static_cast<uint64_t>(esp_timer_get_time()),
            pending_start.readiness,
            forced ? mission::StartMode::forced : mission::StartMode::normal);
        reason = transitionReason(transition);
        if (transition == mission::TransitionResult::completed) {
          actuator_output_inhibited.store(false, std::memory_order_release);
          if (forced)
            final_detail = pending_start.readiness.missingMask();
          recovery_boot::storeFlightCheckpoint(state_machine.snapshot());
        } else {
          const ParaRequest discard{ParaRequest::Kind::discard_snapshot, 0,
                                    false};
          (void)xQueueSendToFront(para_queue, &discard, 0);
        }
      } else {
        const ParaRequest discard{ParaRequest::Kind::discard_snapshot, 0,
                                  false};
        (void)xQueueSendToFront(para_queue, &discard, 0);
      }
      xSemaphoreGive(state_mutex);
      const auto final = command_executor.finish(
          pending_start.request.transaction_id,
          reason == protocol::CommandReason::none
              ? protocol::CommandPhase::completed
              : protocol::CommandPhase::failed,
          reason, final_detail);
      xSemaphoreGive(executor_mutex);
      enqueueResult(final, false);
      start_preparation_pending = false;
    }

    MissionCommandEnvelope command_envelope{};
    while (xQueueReceive(transition_queue, &command_envelope, 0) == pdTRUE) {
      if (xSemaphoreTake(executor_mutex, 0) != pdTRUE) {
        if (xQueueSendToFront(transition_queue, &command_envelope, 0) !=
            pdTRUE)
          result_queue_overflow.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      if (xSemaphoreTake(state_mutex, 0) != pdTRUE) {
        xSemaphoreGive(executor_mutex);
        if (xQueueSendToFront(transition_queue, &command_envelope, 0) !=
            pdTRUE)
          result_queue_overflow.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      mission::TransitionResult transition =
          mission::TransitionResult::not_configured;
      const auto code = static_cast<mission::CommandCode>(
          command_envelope.request.command);
      const auto before_transition = state_machine.snapshot();
      ParaRequest post_transition_para{};
      bool post_transition_para_valid = false;
      bool asynchronous_transition = false;
      protocol::CommandReason direct_reason = protocol::CommandReason::none;
      if (code == mission::CommandCode::cancel_sequence) {
        transition = state_machine.cancelSequence();
        if (transition == mission::TransitionResult::completed) {
          recovery_boot::clearStartReadinessAudit();
          post_transition_para = {ParaRequest::Kind::discard_snapshot,
                                  before_transition.flight_epoch, false};
          post_transition_para_valid = true;
        }
      } else if (code == mission::CommandCode::disable_fin_control) {
        transition = state_machine.disableFinControl();
      } else if (code == mission::CommandCode::fin_free) {
        pending_fin_move = {};
        command_fin_mode = CommandFinMode::free;
        command_fin_auto_hold_allowed = false;
        command_fin_boot_hold_active = false;
        command_fin_target_is_unwrapped = false;
        actuator_output_inhibited.store(false, std::memory_order_release);
        transition = mission::TransitionResult::completed;
      } else if (code == mission::CommandCode::set_fin_zero) {
        if (!encoder_ready.load(std::memory_order_acquire) ||
            !fin_angle_available) {
          direct_reason = protocol::CommandReason::device_unavailable;
        } else {
          fin_zero_reference_rad = unwrapped_fin_rad;
          fin_zero_available = true;
          fin_angle_rad = 0.0;
          fin_rate_valid = false;
          fin_velocity.reset();
          fin_zero_configured.store(true, std::memory_order_release);
          fin_zero_hold_valid.store(false, std::memory_order_release);
          zero_hold_controller.resetValidity();
          pending_fin_move = {};
          fin_zero_metadata.encoder_zero_count = latest_encoder_angle_raw;
          fin_zero_metadata.configured_timestamp_us =
              static_cast<uint64_t>(esp_timer_get_time());
          fin_zero_metadata.approach_direction =
              flight_runtime_metadata::FinZeroApproachDirection::unknown;
          fin_zero_metadata.calibration_method =
              flight_runtime_metadata::FinZeroCalibrationMethod::current_position;
          fin_zero_metadata.ground_verification_status =
              flight_runtime_metadata::FinZeroGroundVerificationStatus::unverified;
          fin_zero_metadata.measured_bidirectional_span_rad = NAN;
          flight_runtime_metadata::publishFinZero(fin_zero_metadata);
          if (command_fin_boot_hold_active) {
            command_fin_target_rad = 0.0;
            command_fin_target_is_unwrapped = false;
            command_fin_mode = CommandFinMode::position_hold;
          } else {
            command_fin_mode = CommandFinMode::free;
            command_fin_target_is_unwrapped = false;
          }
          actuator_output_inhibited.store(false, std::memory_order_release);
          transition = mission::TransitionResult::completed;
        }
      } else if (code == mission::CommandCode::start_fin_zero_hold) {
        const bool usable =
            fin_zero_available &&
            fin_zero_configured.load(std::memory_order_acquire) &&
            encoder_ready.load(std::memory_order_acquire) && fin_rate_valid &&
            motor_ready.load(std::memory_order_acquire) &&
            std::isfinite(fin_angle_rad) && std::isfinite(fin_rate_rad_s);
        if (!fin_zero_available ||
            !fin_zero_configured.load(std::memory_order_acquire)) {
          direct_reason = protocol::CommandReason::not_configured;
        } else if (!usable) {
          direct_reason = protocol::CommandReason::device_unavailable;
        } else {
          pending_fin_move = {};
          command_fin_target_rad = 0.0;
          command_fin_target_is_unwrapped = false;
          command_fin_auto_hold_allowed = false;
          command_fin_boot_hold_active = false;
          command_fin_mode = CommandFinMode::zero_hold;
          actuator_output_inhibited.store(false, std::memory_order_release);
          transition = mission::TransitionResult::completed;
        }
      } else if (code == mission::CommandCode::fin_move_relative) {
        const bool usable =
            fin_zero_available &&
            fin_zero_configured.load(std::memory_order_acquire) &&
            encoder_ready.load(std::memory_order_acquire) && fin_rate_valid &&
            motor_ready.load(std::memory_order_acquire) &&
            std::isfinite(fin_angle_rad) && std::isfinite(fin_rate_rad_s);
        if (!fin_zero_available ||
            !fin_zero_configured.load(std::memory_order_acquire)) {
          direct_reason = protocol::CommandReason::not_configured;
        } else if (!usable) {
          direct_reason = protocol::CommandReason::device_unavailable;
        } else {
          const uint16_t raw =
              static_cast<uint16_t>(command_envelope.request.arguments[0]) |
              static_cast<uint16_t>(command_envelope.request.arguments[1]) << 8U;
          const auto deci_degrees = static_cast<int16_t>(raw);
          constexpr double kDeciDegreeToRad =
              0.0017453292519943296;
          const double target =
              fin_angle_rad + static_cast<double>(deci_degrees) *
                                  kDeciDegreeToRad;
          if (!board::kFinSoftwareLimits.configured ||
              target < board::kFinSoftwareLimits.minimum_rad ||
              target > board::kFinSoftwareLimits.maximum_rad) {
            direct_reason = protocol::CommandReason::invalid_argument;
          } else {
            command_fin_target_rad = target;
            command_fin_target_is_unwrapped = false;
            command_fin_auto_hold_allowed = false;
            command_fin_boot_hold_active = false;
            command_fin_mode = CommandFinMode::relative_move;
            pending_fin_move =
                {true, command_envelope.request.transaction_id,
                 static_cast<uint64_t>(esp_timer_get_time()) + 10'000'000,
                 target};
            actuator_output_inhibited.store(false,
                                             std::memory_order_release);
            asynchronous_transition = true;
          }
        }
      } else if (code == mission::CommandCode::enter_recovery) {
        if (before_transition.state != protocol::MissionState::descent) {
          direct_reason = protocol::CommandReason::invalid_state;
        } else if (!before_transition.deployment_power_cutoff_latched) {
          direct_reason = protocol::CommandReason::safety_interlock;
        } else if (recovery_requested.load(std::memory_order_acquire)) {
          transition = mission::TransitionResult::completed;
        } else {
          RecoveryRequest recovery_request{};
          recovery_request.kind = RecoveryRequest::Kind::enter;
          recovery_request.reason =
              protocol::RecoveryModeReason::ground_requested;
          if (xQueueSend(recovery_queue, &recovery_request, 0) == pdTRUE)
            transition = mission::TransitionResult::completed;
          else
            direct_reason = protocol::CommandReason::busy;
        }
      } else if (code == mission::CommandCode::start_sequence ||
                 code == mission::CommandCode::force_start_sequence) {
        mission::PreflightReadinessSnapshot readiness{};
        ++preflight_generation;
        if (preflight_generation == 0)
          ++preflight_generation;
        readiness.generation = preflight_generation;
        readiness.captured_at_us =
            static_cast<uint64_t>(esp_timer_get_time());
        readiness.fin_zero_configured =
            fin_zero_configured.load(std::memory_order_acquire);
        readiness.parachute_open_configured =
            parachute_open_configured.load(std::memory_order_acquire);
        readiness.parachute_close_configured =
            parachute_close_configured.load(std::memory_order_acquire);
        readiness.motor_profile_valid = flight_config::motorProfileValid();
        readiness.gyro_bias_valid = preflight_gyro_bias_valid;
        readiness.gravity_reference_valid = gravity_reference_valid;
        readiness.ssc_zero_valid = latest_air_data.ssc_zero_valid;
        readiness.resources_preallocated =
            flight_config::nonBypassFlightConfigurationReady();
        command_envelope.readiness = readiness;
        const bool forced_start =
            code == mission::CommandCode::force_start_sequence;
        recovery_boot::storeStartReadinessAudit(readiness, forced_start);
        std::printf(
            "preflight readiness accepted: forced=%u generation=%lu captured_us=%llu ready=0x%02X missing=0x%02X\n",
            forced_start ? 1U : 0U,
            static_cast<unsigned long>(readiness.generation),
            static_cast<unsigned long long>(readiness.captured_at_us),
            static_cast<unsigned>(readiness.readyMask()),
            static_cast<unsigned>(readiness.missingMask()));
        const ParachuteCommandRequest preparation{
            ParachuteCommandRequest::Kind::start_preparation,
            command_envelope.request, readiness};
        if (!start_preparation_pending &&
            xQueueSend(parachute_command_queue, &preparation, 0) == pdTRUE) {
          pending_start = command_envelope;
          start_preparation_pending = true;
          asynchronous_transition = true;
        } else {
          direct_reason = protocol::CommandReason::busy;
        }
      } else if (code == mission::CommandCode::run_preflight_calibration) {
        if (!calibration.active) {
          calibration = {};
          calibration.active = true;
          calibration.transaction_id = command_envelope.request.transaction_id;
          calibration.started_at_us =
              static_cast<uint64_t>(esp_timer_get_time());
          preflight_gyro_bias_valid = false;
          gravity_reference_valid = false;
          latest_air_data.ssc_zero_valid = false;
          latest_air_data.airspeed_valid = false;
          uint32_t next_generation =
              preflight_calibration_generation.fetch_add(
                  1, std::memory_order_acq_rel) +
              1;
          if (next_generation == 0) {
            next_generation = 1;
            preflight_calibration_generation.store(
                next_generation, std::memory_order_release);
          }
          preflight_calibration_active.store(true,
                                             std::memory_order_release);
          asynchronous_transition = true;
        } else {
          direct_reason = protocol::CommandReason::busy;
        }
      }
      const auto transition_state = state_machine.snapshot().state;
      xSemaphoreGive(state_mutex);
      if (asynchronous_transition) {
        xSemaphoreGive(executor_mutex);
        continue;
      }
      if (post_transition_para_valid &&
          xQueueSendToFront(para_queue, &post_transition_para, 0) != pdTRUE) {
        (void)setTrackedParaPower(false);
      }
      const auto reason = direct_reason == protocol::CommandReason::none
                              ? transitionReason(transition)
                              : direct_reason;
      const auto final = command_executor.finish(
          command_envelope.request.transaction_id,
          reason == protocol::CommandReason::none
              ? protocol::CommandPhase::completed
              : protocol::CommandPhase::failed,
          reason);
      xSemaphoreGive(executor_mutex);
      enqueueResult(final, false);
      if (code == mission::CommandCode::disable_fin_control &&
          transition == mission::TransitionResult::completed)
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::
                             fin_control_disabled_by_ground),
                     transition_state);
    }

    protocol::MissionState detector_state =
        protocol::MissionState::command_receive;
    uint32_t current_epoch = 0;
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      const auto snapshot = state_machine.snapshot();
      detector_state = snapshot.state;
      current_epoch = snapshot.flight_epoch;
      xSemaphoreGive(state_mutex);
    }
    if (current_epoch != detector_epoch) {
      detector_epoch = current_epoch;
      if (current_epoch != 0) {
        parachute_deployment_failure.store(0, std::memory_order_release);
        parachute_failure_overflow_detail.store(0, std::memory_order_release);
      }
      liftoff_detector.reset();
      airspeed_gate.reset();
      zero_hold_controller.resetValidity();
      fin_zero_hold_valid.store(false, std::memory_order_release);
      liftoff_detected = false;
      imu_liftoff_latched = false;
      lps_liftoff_latched = false;
      imu_data_loss_latched = false;
    } else if (detector_state == protocol::MissionState::command_receive) {
      liftoff_detector.reset();
      liftoff_detected = false;
      imu_liftoff_latched = false;
      lps_liftoff_latched = false;
      imu_data_loss_latched = false;
    }

    if (imu.initialized()) {
      ICM42688::FifoStatus status{};
      const esp_err_t fifo_status_result = imu.getFifoStatus(status);
      if (fifo_status_result == ESP_OK) {
        if (status.lost_packets != 0 || status.faulted)
          imu_data_loss_latched = true;
        std::array<bringup::ImuSample, bringup::ImuBringup::kMaximumFifoBatch>
            samples{};
        std::size_t count{};
        const esp_err_t read_result =
            imu.readFifo(samples.data(), samples.size(), count);
        if (read_result == ESP_OK) {
          const bool odr_changed =
              std::any_of(samples.begin(), samples.begin() + count,
                          [](const bringup::ImuSample &sample) {
                            return sample.gyro_odr_changed;
                          });
          if (odr_changed) {
            ++timestamp_epoch;
            if (timestamp_epoch == 0)
              timestamp_epoch = 1;
            timestamp_offset_valid = false;
            attitude.invalidateForReset();
            attitude_epoch = 0;
          }
          if (count != 0 && !timestamp_offset_valid &&
              samples[count - 1].host_timestamp_us >=
                  samples[count - 1].sensor_timestamp_us) {
            sensor_to_host_offset_us =
                samples[count - 1].host_timestamp_us -
                samples[count - 1].sensor_timestamp_us;
            timestamp_offset_valid = true;
          }
          for (std::size_t index = 0; index < count; ++index) {
            const auto &sample = samples[index];
            sensors::GyroSample gyro{};
            gyro.timestamp_us = timestamp_offset_valid
                                    ? sample.sensor_timestamp_us +
                                          sensor_to_host_offset_us
                                    : sample.host_timestamp_us;
            gyro.roll_rate_rad_s =
                static_cast<double>(sample.angular_velocity_dps[2]) *
                0.017453292519943295;
            gyro.timestamp_epoch = timestamp_epoch;
            gyro.lost_packets = status.lost_packets;
            gyro.valid = sample.angular_velocity_valid;
            gyro.fifo_full = status.full;
            gyro.format_fault = status.faulted;
            gyro_history.push(gyro);
            if (calibration.active && gyro.valid && !gyro.format_fault &&
                std::isfinite(gyro.roll_rate_rad_s)) {
              calibration.gyro_sum_rad_s += gyro.roll_rate_rad_s;
              ++calibration.gyro_samples;
            }
            if (calibration.active && sample.acceleration_valid &&
                std::isfinite(sample.acceleration_g[0]) &&
                std::isfinite(sample.acceleration_g[1]) &&
                std::isfinite(sample.acceleration_g[2])) {
              calibration.accel_sum_x_g += sample.acceleration_g[0];
              calibration.accel_sum_y_g += sample.acceleration_g[1];
              calibration.accel_sum_z_g += sample.acceleration_g[2];
              ++calibration.accel_samples;
            }
            if (gyro.valid && !gyro.format_fault)
              last_imu_host_sample_us = sample.host_timestamp_us;
            if (attitude_epoch == current_epoch && current_epoch != 0)
              (void)attitude.update(gyro);
            const bool detected = liftoff_detector.update(
                sample.acceleration_g[0], sample.acceleration_g[1],
                sample.acceleration_g[2], sample.acceleration_valid);
            liftoff_detected = liftoff_detected || detected;
            imu_liftoff_latched = imu_liftoff_latched || detected;
          }
        } else
          imu_data_loss_latched = true;
      } else
        imu_data_loss_latched = true;
    }
    const uint64_t imu_now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (calibration.active && imu_now_us >= calibration.started_at_us &&
        imu_now_us - calibration.started_at_us >= 3'000'000) {
      const bool gyro_samples_ok = calibration.gyro_samples >= 1'500;
      const bool accel_samples_ok = calibration.accel_samples >= 1'500;
      if (gyro_samples_ok) {
        preflight_gyro_bias_rad_s =
            calibration.gyro_sum_rad_s / calibration.gyro_samples;
        preflight_gyro_bias_valid =
            std::isfinite(preflight_gyro_bias_rad_s) &&
            std::abs(preflight_gyro_bias_rad_s) <=
                5.0 * 0.017453292519943295;
      }
      if (accel_samples_ok) {
        const double x = calibration.accel_sum_x_g / calibration.accel_samples;
        const double y = calibration.accel_sum_y_g / calibration.accel_samples;
        const double z = calibration.accel_sum_z_g / calibration.accel_samples;
        const double norm = std::sqrt(x * x + y * y + z * z);
        gravity_reference_valid =
            std::isfinite(norm) && norm >= 0.8 && norm <= 1.2;
      }
      preflight_calibration_active.store(false, std::memory_order_release);
      uint32_t detail = 0;
      if (!preflight_gyro_bias_valid)
        detail |= 1U << 4U;
      if (!gravity_reference_valid)
        detail |= 1U << 5U;
      if (!latest_air_data.ssc_zero_valid)
        detail |= 1U << 6U;
      if (xSemaphoreTake(executor_mutex, 0) == pdTRUE) {
        const auto result = command_executor.finish(
            calibration.transaction_id, protocol::CommandPhase::completed,
            protocol::CommandReason::none, detail);
        xSemaphoreGive(executor_mutex);
        enqueueResult(result, false);
        calibration = {};
      }
    }
    const bool imu_stale = last_imu_host_sample_us == 0 ||
                           imu_now_us < last_imu_host_sample_us ||
                           imu_now_us - last_imu_host_sample_us > 3'000;
    if (imu_data_loss_latched || imu_stale) {
      imu_ready.store(false, std::memory_order_release);
      if (attitude.state().valid)
        attitude.invalidateForReset();
      const uint64_t imu_retry_interval_us =
          imu.initialized() ? 100'000ULL : 1'000'000ULL;
      if (imu_now_us - last_imu_recovery_attempt_us >= imu_retry_interval_us) {
        last_imu_recovery_attempt_us = imu_now_us;
        if (imu.initialized())
          (void)imu.end();
        ++timestamp_epoch;
        if (timestamp_epoch == 0)
          timestamp_epoch = 1;
        timestamp_offset_valid = false;
        const esp_err_t restart = imu.begin(spi, true);
        imu_ready.store(restart == ESP_OK, std::memory_order_release);
        if (restart == ESP_OK) {
          imu_data_loss_latched = false;
          last_imu_host_sample_us = imu_now_us;
        }
      }
    } else {
      imu_ready.store(true, std::memory_order_release);
    }
    const uint64_t encoder_now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (encoder.initialized()) {
      bringup::EncoderSample sample{};
      encoder_timing.capture_requested_timestamp_us = encoder_now_us;
      encoder_timing.spi_transaction_start_us =
          static_cast<uint64_t>(esp_timer_get_time());
      const esp_err_t encoder_read_result = encoder.readPipelined(sample);
      encoder_timing.spi_transaction_complete_us =
          static_cast<uint64_t>(esp_timer_get_time());
      encoder_timing.consumer_timestamp_us = encoder_timing.spi_transaction_complete_us;
      if (encoder_read_result != ESP_OK || !sample.valid) {
        if (encoder_timing.raw_capture_missed_tick_count != 0xFFFFU)
          ++encoder_timing.raw_capture_missed_tick_count;
        encoder_timing.pipeline_state =
            flight_runtime_metadata::EncoderPipelineState::faulted;
        encoder_ready.store(false, std::memory_order_release);
        fin_angle_available = false;
        fin_rate_valid = false;
        fin_velocity.reset();
        (void)encoder.end();
        next_encoder_reconnect_us = encoder_now_us + 1'000'000;
      } else {
        latest_encoder_angle_raw = sample.angle_raw;
        constexpr double kPi = 3.141592653589793;
        constexpr double kTwoPi = 2.0 * kPi;
        const double wrapped = static_cast<double>(sample.angle_radians);
        if (!fin_angle_available) {
          if (fin_zero_available) {
            const double turns = std::round((unwrapped_fin_rad - wrapped) / kTwoPi);
            unwrapped_fin_rad = wrapped + turns * kTwoPi;
          } else {
            unwrapped_fin_rad = wrapped;
          }
          fin_angle_available = true;
        } else {
          double delta = wrapped - previous_wrapped_fin_rad;
          if (delta > kPi)
            delta -= kTwoPi;
          else if (delta < -kPi)
            delta += kTwoPi;
          unwrapped_fin_rad += delta;
        }
        previous_wrapped_fin_rad = wrapped;
        if (fin_zero_available)
          fin_angle_rad = unwrapped_fin_rad - fin_zero_reference_rad;
        fin_rate_valid = fin_velocity.update(sample.host_timestamp_us,
                                             unwrapped_fin_rad,
                                             fin_rate_rad_s);
        encoder_ready.store(true, std::memory_order_release);
        if (encoder_timing.spi_transaction_complete_us >
            encoder_timing.capture_requested_timestamp_us + 1'000) {
          if (encoder_timing.consumer_deadline_miss_count != 0xFFFFU)
            ++encoder_timing.consumer_deadline_miss_count;
        }
        encoder_timing.pipeline_state =
            fin_rate_valid ? flight_runtime_metadata::EncoderPipelineState::ready
                           : flight_runtime_metadata::EncoderPipelineState::warming_up;
      }
    } else {
      encoder_ready.store(false, std::memory_order_release);
      fin_rate_valid = false;
      encoder_timing.pipeline_state =
          flight_runtime_metadata::EncoderPipelineState::faulted;
      if (encoder_now_us >= next_encoder_reconnect_us) {
        esp_err_t reconnect = encoder.begin(spi);
        AS5047D::Status reconnect_status{};
        if (reconnect == ESP_OK)
          reconnect = encoder.getStatus(reconnect_status);
        if (reconnect == ESP_OK)
          reconnect = encoder.startPipelinedRead();
        if (reconnect == ESP_OK) {
          encoder_ready.store(true, std::memory_order_release);
          fin_angle_available = false;
          fin_rate_valid = false;
          fin_velocity.reset();
          encoder_timing.pipeline_ready_timestamp_us = encoder_now_us;
          encoder_timing.pipeline_state =
              flight_runtime_metadata::EncoderPipelineState::warming_up;
          next_encoder_reconnect_us = 0;
          std::printf("AS5047D reconnected\n");
        } else {
          if (encoder.initialized())
            (void)encoder.end();
          next_encoder_reconnect_us = encoder_now_us + 1'000'000;
        }
      }
    }
    flight_runtime_metadata::publishEncoderTiming(encoder_timing);
    const bool fin_observation_valid =
        encoder_ready.load(std::memory_order_acquire) && fin_angle_available &&
        fin_rate_valid && std::isfinite(unwrapped_fin_rad) &&
        std::isfinite(fin_rate_rad_s);
    const bool fin_sample_valid =
        fin_observation_valid && fin_zero_available && std::isfinite(fin_angle_rad);
    if (detector_state == protocol::MissionState::command_receive &&
        command_fin_auto_hold_allowed && command_fin_mode == CommandFinMode::free &&
        fin_observation_valid &&
        !actuator_output_inhibited.load(std::memory_order_acquire)) {
      command_fin_target_rad = unwrapped_fin_rad;
      command_fin_target_is_unwrapped = true;
      command_fin_boot_hold_active = true;
      command_fin_mode = CommandFinMode::position_hold;
    }
    const bool zero_hold_valid = zero_hold_controller.updateValidity(
        fin_angle_rad, fin_rate_rad_s, fin_sample_valid);
    fin_zero_hold_valid.store(zero_hold_valid, std::memory_order_release);

    if (pending_fin_move.active) {
      constexpr double kMoveToleranceRad = 0.008726646259971648;
      constexpr double kMoveRateToleranceRadS = 0.03490658503988659;
      const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
      const bool reached =
          fin_sample_valid &&
          std::abs(fin_angle_rad - pending_fin_move.target_rad) <=
              kMoveToleranceRad &&
          std::abs(fin_rate_rad_s) <= kMoveRateToleranceRadS;
      const bool timed_out = now_us >= pending_fin_move.deadline_us;
      if (reached)
        command_fin_mode = CommandFinMode::position_hold;
      else if (timed_out)
        command_fin_mode = CommandFinMode::free;
      if ((reached || timed_out) &&
          xSemaphoreTake(executor_mutex, 0) == pdTRUE) {
        const auto result = command_executor.finish(
            pending_fin_move.transaction_id,
            reached ? protocol::CommandPhase::completed
                    : protocol::CommandPhase::failed,
            reached ? protocol::CommandReason::none
                    : protocol::CommandReason::timeout);
        xSemaphoreGive(executor_mutex);
        enqueueResult(result, false);
        pending_fin_move = {};
      }
    }

    AirDataSnapshot air_data{};
    if (xQueueReceive(air_data_queue, &air_data, 0) == pdTRUE) {
      latest_air_data = air_data;
      if (air_data.flight.flight_epoch == current_epoch) {
        liftoff_detected =
            liftoff_detected || air_data.flight.lps_liftoff_detected;
        lps_liftoff_latched =
            lps_liftoff_latched || air_data.flight.lps_liftoff_detected;
      }
    }

    mission::MissionTickInput tick{};
    tick.monotonic_us = static_cast<uint64_t>(esp_timer_get_time());
    ++control_tick;
    tick.control_tick = control_tick;
    tick.liftoff_detected = liftoff_detected;
    const bool lps_fresh =
        latest_air_data.lps_monotonic_us != 0 &&
        tick.monotonic_us >= latest_air_data.lps_monotonic_us &&
        tick.monotonic_us - latest_air_data.lps_monotonic_us <=
            sensors::FreshnessThresholds::lps_us;
    const bool ssc_fresh =
        latest_air_data.ssc_monotonic_us != 0 &&
        tick.monotonic_us >= latest_air_data.ssc_monotonic_us &&
        tick.monotonic_us - latest_air_data.ssc_monotonic_us <=
            sensors::FreshnessThresholds::ssc_us;
    tick.deployment_pressure_condition =
        lps_fresh &&
        latest_air_data.flight.flight_epoch == current_epoch &&
        latest_air_data.flight.pressure_apex_detected;
    tick.control.fin_control_available =
        fin_sample_valid && motor_ready.load(std::memory_order_acquire) &&
        !actuator_output_inhibited.load(std::memory_order_acquire) &&
        flight_config::productionFlightConfigurationReady();
    tick.control.fin_zero_hold_valid =
        fin_zero_hold_valid.load(std::memory_order_acquire);
    tick.control.attitude_valid = attitude.state().valid;
    tick.control.roll_estimate_liftoff_relative_unwrapped_rad =
        attitude.state().roll_estimate_liftoff_relative_unwrapped_rad;
    tick.control.roll_estimator_timestamp_us = attitude.state().timestamp_us;
    tick.control.attitude_fresh =
        attitude.state().valid && attitude.state().timestamp_us != 0 &&
        tick.monotonic_us >= attitude.state().timestamp_us &&
        tick.monotonic_us - attitude.state().timestamp_us <= 3'000;
    tick.control.lps_available =
        lps_ready.load(std::memory_order_acquire) && lps_fresh &&
        latest_air_data.lps_valid;
    tick.control.ssc_available =
        ssc_ready.load(std::memory_order_acquire) && ssc_fresh &&
        latest_air_data.ssc_valid;
    tick.control.gyro_bias_valid =
        attitude_epoch == current_epoch && attitude.state().valid;
    tick.control.ssc_zero_valid = latest_air_data.ssc_zero_valid;
    const bool airspeed_available =
        tick.control.lps_available && tick.control.ssc_available &&
        latest_air_data.airspeed_valid;
    tick.control.airspeed_above_60 =
        airspeed_gate.update(airspeed_available, latest_air_data.airspeed_mps);
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      state_machine.tick(tick);
      recovery_boot::storeFlightCheckpoint(state_machine.snapshot());
      xSemaphoreGive(state_mutex);
    }
    liftoff_detected = false;

    RuntimeStatus status{};
    bool power_cutoff = false;
    bool deployment_started = false;
    uint32_t flight_epoch = 0;
    mission::MissionSnapshot mission_snapshot{};
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      mission_snapshot = state_machine.snapshot();
      status.state = mission_snapshot.state;
      status.fin_mode = mission_snapshot.fin == mission::FinDirective::zero_hold
                            ? protocol::FinMode::zero_hold
                            : (mission_snapshot.fin == mission::FinDirective::roll_control
                                   ? protocol::FinMode::roll_control
                                   : protocol::FinMode::brake);
      status.para_mode = para_mode_actual.load(std::memory_order_acquire);
      power_cutoff = mission_snapshot.deployment_power_cutoff_latched;
      deployment_started = mission_snapshot.deployment_started;
      flight_epoch = mission_snapshot.flight_epoch;
      xSemaphoreGive(state_mutex);
    }
    if (mission_snapshot.liftoff_time_valid &&
        attitude_epoch != mission_snapshot.flight_epoch) {
      if (preflight_gyro_bias_valid &&
          attitude.beginFlight(gyro_history, mission_snapshot.liftoff_time_us,
                               preflight_gyro_bias_rad_s))
        attitude_epoch = mission_snapshot.flight_epoch;
      else
        attitude_epoch = 0;
    }
    if (mission_snapshot.reset_invalidated) {
      attitude.invalidateForReset();
      attitude_epoch = 0;
    }

    control::TorqueRequest torque_request{};
    control::MotorCommand motor_command{};
    bool motor_saturated = false;
    protocol::quantization::TorqueError torque_error =
        protocol::quantization::TorqueError::unavailable;
    const bool output_inhibited =
        recovery_requested.load(std::memory_order_acquire) ||
        actuator_output_inhibited.load(std::memory_order_acquire) ||
        mission_snapshot.reset_invalidated;

    auto applyTorque = [&](const control::TorqueRequest &request) {
      torque_request = request;
      if (!request.valid) {
        torque_error =
            protocol::quantization::TorqueError::controller_input_invalid;
        return motor_driver.brake();
      }
      double motor_bus_voltage_v = flight_config::kMotorBusVoltageV;
      if (motor_bus_voltage_valid.load(std::memory_order_acquire)) {
        const uint32_t millivolts =
            motor_bus_millivolts.load(std::memory_order_acquire);
        if (millivolts >= 500U)
          motor_bus_voltage_v = static_cast<double>(millivolts) / 1'000.0;
      }
      motor_command = torque_mapper.map(
          request.output_torque_nm, fin_angle_rad, fin_rate_rad_s,
          motor_bus_voltage_v);
      if (!motor_command.valid) {
        torque_error = protocol::quantization::TorqueError::limit_config_invalid;
        return motor_driver.brake();
      }
      motor_saturated =
          request.saturated || motor_command.saturated ||
          motor_command.pwm_duty >
              flight_config::kProductionMotorMaximumDuty;
      torque_error = protocol::quantization::TorqueError::unknown;
      if (motor_command.brake)
        return motor_driver.brake();
      return motor_driver.apply(motor_command);
    };

    esp_err_t motor_output_result = ESP_OK;
    bool motor_output_coasting = false;
    bool motor_output_braking = false;
    if (output_inhibited) {
      motor_output_result = motor_driver.coast();
      motor_output_coasting = true;
      if (mission_snapshot.reset_invalidated)
        torque_error = protocol::quantization::TorqueError::reset_invalidated;
    } else if (mission_snapshot.state ==
               protocol::MissionState::command_receive) {
      if (command_fin_mode == CommandFinMode::free) {
        motor_output_result = motor_driver.coast();
        motor_output_coasting = true;
      } else {
        const bool use_unwrapped_target =
            command_fin_mode == CommandFinMode::position_hold &&
            command_fin_target_is_unwrapped;
        const bool command_sample_valid =
            use_unwrapped_target ? fin_observation_valid : fin_sample_valid;
        if (!motor_ready.load(std::memory_order_acquire) ||
            !motor_driver.initialized() || !command_sample_valid) {
          motor_output_result = motor_driver.brake();
          motor_output_braking = true;
          torque_error =
              protocol::quantization::TorqueError::controller_input_invalid;
        } else {
          const double target = command_fin_mode == CommandFinMode::zero_hold
                                    ? 0.0
                                    : command_fin_target_rad;
          const double measured =
              use_unwrapped_target ? unwrapped_fin_rad : fin_angle_rad;
          const auto request =
              zero_hold_controller.compute(measured - target, fin_rate_rad_s);
          motor_output_result = applyTorque(request);
          motor_output_braking = !request.valid || !motor_command.valid;
        }
      }
    } else if (!motor_ready.load(std::memory_order_acquire) ||
               !motor_driver.initialized()) {
      motor_output_result = motor_driver.coast();
      motor_output_coasting = true;
      torque_error = protocol::quantization::TorqueError::internal_error;
    } else if (!flight_config::motorProfileValid()) {
      // CommandReceiveの明示試験は許可するが、飛行sequenceでは未認定profileを
      // ZeroHold/Roll出力へ接続しない。ForceStartでもvalidへ偽装しない。
      motor_output_result = motor_driver.brake();
      motor_output_braking = true;
      torque_error = protocol::quantization::TorqueError::limit_config_invalid;
    } else if (mission_snapshot.fin == mission::FinDirective::zero_hold) {
      if (fin_sample_valid) {
        const auto request =
            zero_hold_controller.compute(fin_angle_rad, fin_rate_rad_s);
        motor_output_result = applyTorque(request);
        motor_output_braking = !request.valid || !motor_command.valid;
      } else {
        motor_output_result = motor_driver.brake();
        motor_output_braking = true;
        torque_error =
            protocol::quantization::TorqueError::controller_input_invalid;
      }
    } else if (mission_snapshot.fin == mission::FinDirective::roll_control) {
      const auto &attitude_state = attitude.state();
      const double roll_deviation_rad =
          attitude_state.roll_estimate_liftoff_relative_unwrapped_rad -
          mission_snapshot.control_roll_reference_unwrapped_rad;
      const bool inputs_valid =
          fin_sample_valid && mission_snapshot.control_roll_reference_valid &&
          attitude_state.valid && tick.control.attitude_fresh &&
          std::isfinite(roll_deviation_rad) &&
          attitude_epoch == mission_snapshot.flight_epoch &&
          airspeed_available;
      if (inputs_valid) {
        const control::RollState state{
            roll_deviation_rad, fin_angle_rad,
            attitude_state.roll_rate_rad_s, fin_rate_rad_s};
        const auto request = roll_controller.compute(
            state, latest_air_data.airspeed_mps,
            control::RollControlAuthority::gentle,
            board::kControlAuthorityLimits);
        motor_output_result = applyTorque(request);
        motor_output_braking = !request.valid || !motor_command.valid;
      } else {
        motor_output_result = motor_driver.brake();
        motor_output_braking = true;
        torque_error =
            protocol::quantization::TorqueError::controller_input_invalid;
      }
    } else {
      motor_output_result = motor_driver.brake();
      motor_output_braking = true;
    }
    if (motor_output_result != ESP_OK) {
      motor_ready.store(false, std::memory_order_release);
      (void)motor_driver.coast();
      motor_output_coasting = true;
      motor_output_braking = false;
      torque_error = protocol::quantization::TorqueError::internal_error;
    }
    if (motor_output_coasting)
      status.fin_mode = protocol::FinMode::free;
    else if (motor_output_braking)
      status.fin_mode = protocol::FinMode::brake;
    else if (mission_snapshot.state ==
             protocol::MissionState::command_receive) {
      if (command_fin_mode == CommandFinMode::zero_hold)
        status.fin_mode = protocol::FinMode::zero_hold;
      else if (command_fin_mode == CommandFinMode::position_hold)
        status.fin_mode = protocol::FinMode::position_hold;
      else if (command_fin_mode == CommandFinMode::relative_move)
        status.fin_mode = protocol::FinMode::relative_move;
    }

    const bool encoder_alive = encoder_ready.load(std::memory_order_acquire);
    constexpr double kRadiansToDegrees = 57.29577951308232;
    if (attitude.state().valid) {
      status.roll_raw = protocol::quantization::encodeRoll(
          attitude.state().roll_estimate_liftoff_relative_unwrapped_rad *
              kRadiansToDegrees,
          protocol::quantization::RollError::unknown);
      status.roll_rate_raw = protocol::quantization::encodeRollRate(
          attitude.state().roll_rate_rad_s * kRadiansToDegrees,
          protocol::quantization::RollError::unknown);
    } else {
      const auto reason = mission_snapshot.reset_invalidated
                              ? protocol::quantization::RollError::reset_invalidated
                              : protocol::quantization::RollError::
                                    estimator_invalid;
      status.roll_raw = static_cast<uint16_t>(reason);
      status.roll_rate_raw = static_cast<uint16_t>(reason);
    }
    status.fin_angle_raw =
        encoder_alive && fin_zero_available
            ? protocol::quantization::encodeFinAngle(
                  fin_angle_rad * kRadiansToDegrees,
                  protocol::quantization::FinAngleError::out_of_mechanical_range)
            : static_cast<uint8_t>(
                  encoder_alive
                      ? protocol::quantization::FinAngleError::zero_not_configured
                      : protocol::quantization::FinAngleError::not_initialized);
    status.control_roll_reference_capture_event_sequence =
        mission_snapshot.control_roll_reference_capture_event_sequence;
    status.control_roll_reference_capture_tick =
        mission_snapshot.control_roll_reference_capture_tick;
    status.control_roll_reference_estimator_timestamp_us =
        mission_snapshot.control_roll_reference_estimator_timestamp_us;
    status.roll_estimate_liftoff_relative_unwrapped_rad =
        attitude.state().roll_estimate_liftoff_relative_unwrapped_rad;
    status.control_roll_reference_unwrapped_rad =
        mission_snapshot.control_roll_reference_unwrapped_rad;
    status.control_roll_flags =
        mission_snapshot.control_roll_reference_valid
            ? protocol::ControlRollTelemetryV2::reference_valid
            : 0;
    if (mission_snapshot.state == protocol::MissionState::control)
      status.control_roll_flags |=
          protocol::ControlRollTelemetryV2::control_active;
    if (mission_snapshot.control_roll_reference_valid) {
      status.control_roll_reference_unwrapped_raw =
          protocol::quantization::encodeRoll(
              mission_snapshot.control_roll_reference_unwrapped_rad *
                  kRadiansToDegrees,
              protocol::quantization::RollError::unknown);
      if (attitude.state().valid) {
        status.roll_deviation_unwrapped_rad =
            attitude.state().roll_estimate_liftoff_relative_unwrapped_rad -
            mission_snapshot.control_roll_reference_unwrapped_rad;
        status.roll_deviation_unwrapped_raw =
            protocol::quantization::encodeRoll(
                status.roll_deviation_unwrapped_rad * kRadiansToDegrees,
                protocol::quantization::RollError::unknown);
      }
    } else {
      const auto error = mission_snapshot.reset_invalidated
                             ? protocol::quantization::RollError::reset_invalidated
                             : protocol::quantization::RollError::unavailable;
      status.control_roll_reference_unwrapped_raw =
          static_cast<uint16_t>(error);
      status.roll_deviation_unwrapped_raw = static_cast<uint16_t>(error);
    }
    if (status.control_roll_reference_unwrapped_raw ==
        static_cast<uint16_t>(protocol::quantization::RollError::out_of_range))
      status.control_roll_flags |=
          protocol::ControlRollTelemetryV2::reference_out_of_range;
    if (status.roll_deviation_unwrapped_raw ==
        static_cast<uint16_t>(protocol::quantization::RollError::out_of_range))
      status.control_roll_flags |=
          protocol::ControlRollTelemetryV2::deviation_out_of_range;
    status.fin_rate_raw =
        fin_sample_valid
            ? protocol::quantization::encodeFinRate(
                  fin_rate_rad_s * kRadiansToDegrees,
                  protocol::quantization::FinRateError::estimator_numeric_error)
            : static_cast<uint16_t>(
                  protocol::quantization::FinRateError::estimator_not_ready);
    status.requested_torque_raw =
        torque_request.valid
            ? protocol::quantization::encodeRequestedTorque(
                  torque_request.output_torque_nm,
                  protocol::quantization::TorqueError::controller_numeric_error)
            : static_cast<uint16_t>(torque_error);
    static bool cutoff_sent = false;
    static bool deployment_sent = false;
    if (power_cutoff && !cutoff_sent) {
      const PowerRequest power{false, false, true};
      const ParaRequest para{ParaRequest::Kind::power_off, flight_epoch, true};
      (void)xQueueSend(power_queue, &power, 0);
      (void)xQueueSend(para_queue, &para, 0);
      cutoff_sent = true;
    }
    if (deployment_started && !deployment_sent) {
      const ParaRequest para{ParaRequest::Kind::open, flight_epoch};
      deployment_sent = xQueueSend(para_queue, &para, 0) == pdTRUE;
    }
    if (!deployment_started)
      deployment_sent = false;
    if (flight_epoch == 0)
      cutoff_sent = false;
    status.lps_pressure_raw =
        lps_fresh
            ? latest_air_data.pressure_raw
            : static_cast<uint16_t>(
                  lps_ready.load(std::memory_order_acquire)
                      ? protocol::quantization::LpsPressureError::stale
                      : protocol::quantization::LpsPressureError::not_initialized);
    status.lps_temperature_raw =
        lps_fresh
            ? latest_air_data.temperature_raw
            : static_cast<uint8_t>(
                  lps_ready.load(std::memory_order_acquire)
                      ? protocol::quantization::LpsTemperatureError::stale
                      : protocol::quantization::LpsTemperatureError::not_initialized);
    status.airspeed_raw =
        ssc_fresh
            ? latest_air_data.airspeed_raw
            : static_cast<uint8_t>(
                  ssc_ready.load(std::memory_order_acquire)
                      ? protocol::quantization::AirspeedError::ssc_stale
                      : protocol::quantization::AirspeedError::
                            ssc_not_initialized);
    status.lps_sample_valid = lps_fresh && latest_air_data.lps_valid;
    status.airspeed_sample_valid =
        ssc_fresh && latest_air_data.ssc_valid &&
        latest_air_data.airspeed_valid && status.airspeed_raw <= 245;
    status.deployment_power_cutoff = power_cutoff;
    status.flight_elapsed_us = mission_snapshot.elapsed_us;
    status.static_pressure_pa =
        static_cast<float>(latest_air_data.static_pressure_pa);
    status.ssc_temperature_celsius =
        static_cast<float>(latest_air_data.ssc_temperature_celsius);
    status.airspeed_mps = static_cast<float>(latest_air_data.airspeed_mps);
    const bool air_data_error = !status.lps_sample_valid ||
                                !status.airspeed_sample_valid;
    status.flight_status =
        (lps_liftoff_latched ? (1U << 0U) : 0U) |
        (imu_liftoff_latched ? (1U << 1U) : 0U) |
        (imu_ready.load(std::memory_order_acquire) ? (1U << 2U) : 0U) |
        (sts_ready.load(std::memory_order_acquire) ? (1U << 3U) : 0U) |
        (status.state == protocol::MissionState::control
             ? (1U << 4U)
             : 0U) |
        (logic_power_present.load(std::memory_order_acquire) ? (1U << 5U) : 0U) |
        (motor_power_present.load(std::memory_order_acquire) ? (1U << 6U) : 0U) |
        (can_healthy.load(std::memory_order_acquire) ? (1U << 8U) : 0U) |
        (imu_data_loss_latched ? (1U << 9U) : 0U) |
        (!encoder_alive ? (1U << 10U) : 0U) |
        (air_data_error ? (1U << 11U) : 0U) |
        (motor_saturated ? (1U << 12U) : 0U) |
        (status.fin_mode == protocol::FinMode::brake ? (1U << 13U) : 0U) |
        (mission_snapshot.reset_invalidated ? (1U << 14U) : 0U) |
        (mission_snapshot.control_reentry_inhibited ? (1U << 15U) : 0U);
    status.config_flags =
        (flight_config::motorProfileValid() ? (1U << 0U) : 0U) |
        (fin_zero_configured.load(std::memory_order_acquire) ? (1U << 1U)
                                                             : 0U) |
        (parachute_open_configured.load(std::memory_order_acquire) &&
                 parachute_close_configured.load(std::memory_order_acquire)
             ? (1U << 2U)
             : 0U) |
        (latest_air_data.ssc_zero_valid ? (1U << 3U) : 0U) |
        (1U << 7U);
    (void)xQueueOverwrite(status_queue, &status);

    static uint8_t flash_decimation = 0;
    if (status.state != protocol::MissionState::command_receive &&
        !recovery_requested.load(std::memory_order_acquire)) {
      uint8_t gain_clamp_flags = 0;
      if (status.airspeed_sample_valid) {
        if (status.airspeed_mps <=
            flight_config::kRollGainSchedule.points.front().airspeed_mps)
          gain_clamp_flags |= 1U << 0U;
        if (status.airspeed_mps >=
            flight_config::kRollGainSchedule.points.back().airspeed_mps)
          gain_clamp_flags |= 1U << 1U;
      }
      const flight_log::Sample log_sample{
          static_cast<uint64_t>(esp_timer_get_time()),
          status.flight_elapsed_us,
          mission_snapshot.flight_epoch,
          sd_log_drop_count.load(std::memory_order_relaxed),
          flash_log_drop_count.load(std::memory_order_relaxed),
          status.flight_status,
          static_cast<uint8_t>(status.state),
          status.config_flags,
          static_cast<uint8_t>(status.fin_mode),
          static_cast<uint8_t>(status.para_mode),
          status.lps_temperature_raw,
          status.airspeed_raw,
          status.fin_angle_raw,
          status.lps_pressure_raw,
          status.roll_raw,
          status.roll_rate_raw,
          status.fin_rate_raw,
          status.requested_torque_raw,
          status.control_roll_reference_unwrapped_raw,
          status.roll_deviation_unwrapped_raw,
          status.control_roll_flags,
          status.control_roll_reference_capture_event_sequence,
          gain_clamp_flags,
          status.lps_sample_valid,
          status.airspeed_sample_valid,
          status.deployment_power_cutoff,
          (status.control_roll_flags &
           protocol::ControlRollTelemetryV2::reference_valid) != 0,
          fin_zero_configured.load(std::memory_order_acquire),
          flight_log::kUnknownEncoderZeroCount,
          status.control_roll_reference_capture_tick,
          status.control_roll_reference_estimator_timestamp_us,
          static_cast<float>(status.roll_estimate_liftoff_relative_unwrapped_rad),
          static_cast<float>(status.control_roll_reference_unwrapped_rad),
          static_cast<float>(status.roll_deviation_unwrapped_rad),
          status.static_pressure_pa,
          status.ssc_temperature_celsius,
          status.airspeed_mps,
          static_cast<float>(flight_config::kAirData.pitot_coefficient_assumed),
          static_cast<float>(flight_config::kAirData.pitot_coefficient_true_min),
          static_cast<float>(flight_config::kAirData.pitot_coefficient_true_max),
          NAN};
      const auto serialized = flight_log::serialize(log_sample);
      if (xQueueSend(sd_log_queue, &serialized, 0) != pdTRUE)
        sd_log_drop_count.fetch_add(1, std::memory_order_relaxed);
      if (++flash_decimation >= 20U) {
        flash_decimation = 0;
        if (xQueueSend(flash_log_queue, &serialized, 0) != pdTRUE)
          flash_log_drop_count.fetch_add(1, std::memory_order_relaxed);
      }
    } else {
      flash_decimation = 0;
    }

    uint16_t event_flags = 0;
    if (status.state != previous_state)
      event_flags |=
          protocol::eventFlag(protocol::MissionEventFlag::state_changed);
    if (mission_snapshot.liftoff_time_valid && !previous_liftoff)
      event_flags |=
          protocol::eventFlag(protocol::MissionEventFlag::liftoff_detected);
    if (deployment_started && !previous_deployment)
      event_flags |= protocol::eventFlag(
          protocol::MissionEventFlag::parachute_open_started);
    if (power_cutoff && !previous_cutoff)
      event_flags |= protocol::eventFlag(
          protocol::MissionEventFlag::deployment_power_cutoff);
    if (imu_data_loss_latched && !previous_imu_error)
      event_flags |= protocol::eventFlag(
          protocol::MissionEventFlag::icm_data_loss_or_error);
    if (!encoder_alive && !previous_encoder_error)
      event_flags |=
          protocol::eventFlag(protocol::MissionEventFlag::encoder_error);
    if (air_data_error && !previous_air_data_error)
      event_flags |=
          protocol::eventFlag(protocol::MissionEventFlag::air_data_error);
    if (motor_saturated && !previous_motor_saturation)
      event_flags |= protocol::eventFlag(
          protocol::MissionEventFlag::fin_motor_saturation);
    if (mission_snapshot.reset_invalidated && !previous_reset_invalidated)
      event_flags |= protocol::eventFlag(
          protocol::MissionEventFlag::reset_or_recovery);
    if (event_flags != 0) {
      const uint16_t elapsed_raw = protocol::quantization::encodeDescentElapsed(
          static_cast<double>(mission_snapshot.elapsed_us) / 1.0e6,
          protocol::quantization::TimeError::unavailable);
      enqueueEvent(event_flags, status.state, elapsed_raw);
    }
    previous_state = status.state;
    previous_liftoff = mission_snapshot.liftoff_time_valid;
    previous_deployment = deployment_started;
    previous_cutoff = power_cutoff;
    previous_imu_error = imu_data_loss_latched;
    previous_encoder_error = !encoder_alive;
    previous_air_data_error = air_data_error;
    previous_motor_saturation = motor_saturated;
    previous_reset_invalidated = mission_snapshot.reset_invalidated;
    resetWatchdog();
  }
}

void airDataTask(void *) {
  addWatchdog();
  PowerRequest power{true, false, false, true, false};
  (void)xQueueSend(power_queue, &power, 0);
  vTaskDelay(pdMS_TO_TICKS(100));

  I2CCREATE bus;
  I2CCREATE::Config config{};
  config.port = board::kAirDataI2cPort;
  config.sda = board::kAirDataSda;
  config.scl = board::kAirDataScl;
  config.frequency_hz = board::kAirDataI2cFrequencyHz;
  config.enable_internal_pullups = false;
  config.lock_timeout = avi::Timeout::noWait();
  config.operation_timeout =
      avi::Timeout::milliseconds(board::kAirDataOperationTimeoutMs);
  const esp_err_t bus_result = bus.begin(config);
  LPS25HB lps;
  SSCDRRN005PD2A5 ssc;
  esp_err_t lps_result = ESP_ERR_NOT_FOUND;
  esp_err_t ssc_result = ESP_ERR_NOT_FOUND;
  if (bus_result == ESP_OK) {
    LPS25HB::Config lps_config{};
    lps_config.odr = LPS25HB::Odr::hz25;
    lps_config.pressure_average = LPS25HB::PressureAverage::samples8;
    lps_config.temperature_average = LPS25HB::TemperatureAverage::samples8;
    if (bus.probe(0x5C) == ESP_OK)
      lps_result = lps.begin(bus, LPS25HB::Address::low, lps_config);
    else if (bus.probe(0x5D) == ESP_OK)
      lps_result = lps.begin(bus, LPS25HB::Address::high, lps_config);
    if (bus.probe(0x28) == ESP_OK)
      ssc_result = ssc.begin(bus);
  }
  lps_ready.store(lps_result == ESP_OK, std::memory_order_release);
  ssc_ready.store(ssc_result == ESP_OK, std::memory_order_release);
  std::printf("AirDataTask bus=%s lps=%s ssc=%s timeout_ms=%lu%s\n",
              esp_err_to_name(bus_result), esp_err_to_name(lps_result),
              esp_err_to_name(ssc_result),
              static_cast<unsigned long>(board::kAirDataOperationTimeoutMs),
              ssc_result == ESP_ERR_NOT_FOUND ? " (未接続は継続可能)" : "");

  sensors::AirDataFlightLogic flight_logic;
  sensors::DifferentialPressureConditioner pressure_conditioner{
      flight_config::kAirData.zero_calibration_samples,
      flight_config::kAirData.moving_average_samples,
      flight_config::kAirData.negative_pressure_tolerance_pa};
  AirDataSnapshot snapshot{};
  mission::MissionSnapshot mission_snapshot{};
  uint64_t last_ssc_us = 0;
  uint64_t last_lps_us = 0;
  uint32_t calibration_generation = 0;
  struct SscDiagnostics {
    uint32_t success{};
    uint32_t stale{};
    uint32_t command_mode{};
    uint32_t diagnostic_fault{};
    uint32_t timeout{};
    uint32_t i2c_error{};
    uint32_t reconnect_attempts{};
    uint32_t reconnect_success{};
    uint32_t max_read_us{};
    uint32_t consecutive_restartable_faults{};
  } ssc_diagnostics;
  uint64_t next_ssc_reconnect_us =
      ssc_result == ESP_OK
          ? 0
          : static_cast<uint64_t>(esp_timer_get_time()) +
                static_cast<uint64_t>(board::kSscReconnectIntervalMs) *
                    1'000ULL;
  esp_err_t last_ssc_reconnect_error = ssc_result;
  bool ssc_recovery_pending = ssc_result != ESP_OK;

  auto updateMissionSnapshot = [&]() {
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      const auto updated = state_machine.snapshot();
      if (updated.state == protocol::MissionState::command_receive &&
          mission_snapshot.state != protocol::MissionState::command_receive) {
        pressure_conditioner.reset();
        snapshot.ssc_zero_valid = false;
        snapshot.airspeed_valid = false;
      }
      mission_snapshot = updated;
      xSemaphoreGive(state_mutex);
    }
  };
  auto setInitialSscError = [&](esp_err_t result) {
    if (snapshot.ssc_monotonic_us != 0)
      return;
    protocol::quantization::AirspeedError error =
        protocol::quantization::AirspeedError::ssc_i2c_error;
    if (result == ESP_ERR_TIMEOUT)
      error = protocol::quantization::AirspeedError::ssc_i2c_timeout;
    else if (result == ESP_ERR_NOT_FINISHED)
      error = protocol::quantization::AirspeedError::ssc_stale;
    else if (result == ESP_ERR_INVALID_STATE)
      error = protocol::quantization::AirspeedError::ssc_command_mode;
    else if (result == ESP_ERR_INVALID_RESPONSE)
      error = protocol::quantization::AirspeedError::ssc_diagnostic_fault;
    snapshot.airspeed_raw = static_cast<uint8_t>(error);
  };
  auto setInitialLpsError = [&](esp_err_t result) {
    if (snapshot.lps_monotonic_us != 0)
      return;
    snapshot.pressure_raw = static_cast<uint16_t>(
        result == ESP_ERR_TIMEOUT
            ? protocol::quantization::LpsPressureError::i2c_timeout
            : protocol::quantization::LpsPressureError::i2c_bus_error);
    snapshot.temperature_raw = static_cast<uint8_t>(
        result == ESP_ERR_TIMEOUT
            ? protocol::quantization::LpsTemperatureError::i2c_timeout
            : protocol::quantization::LpsTemperatureError::i2c_bus_error);
  };
  auto incrementSaturated = [](uint32_t &value) {
    if (value != std::numeric_limits<uint32_t>::max())
      ++value;
  };
  auto printSscDiagnostics = [&](const char *prefix, esp_err_t result) {
    std::printf(
        "SSC %s result=%s success=%lu stale=%lu command=%lu diagnostic=%lu "
        "timeout=%lu i2c=%lu reconnect=%lu/%lu consecutive=%lu "
        "max_read_us=%lu\n",
        prefix, esp_err_to_name(result),
        static_cast<unsigned long>(ssc_diagnostics.success),
        static_cast<unsigned long>(ssc_diagnostics.stale),
        static_cast<unsigned long>(ssc_diagnostics.command_mode),
        static_cast<unsigned long>(ssc_diagnostics.diagnostic_fault),
        static_cast<unsigned long>(ssc_diagnostics.timeout),
        static_cast<unsigned long>(ssc_diagnostics.i2c_error),
        static_cast<unsigned long>(ssc_diagnostics.reconnect_success),
        static_cast<unsigned long>(ssc_diagnostics.reconnect_attempts),
        static_cast<unsigned long>(
            ssc_diagnostics.consecutive_restartable_faults),
        static_cast<unsigned long>(ssc_diagnostics.max_read_us));
  };
  auto recordSscRead = [&](esp_err_t result, uint32_t elapsed_us) {
    if (elapsed_us > ssc_diagnostics.max_read_us)
      ssc_diagnostics.max_read_us = elapsed_us;
    bool restartable = false;
    if (result == ESP_OK) {
      incrementSaturated(ssc_diagnostics.success);
      ssc_diagnostics.consecutive_restartable_faults = 0;
    } else if (result == ESP_ERR_NOT_FINISHED) {
      incrementSaturated(ssc_diagnostics.stale);
      // staleはI2C transportが応答した結果なのでdevice再生成の根拠にしない。
      ssc_diagnostics.consecutive_restartable_faults = 0;
    } else if (result == ESP_ERR_INVALID_STATE) {
      incrementSaturated(ssc_diagnostics.command_mode);
      restartable = true;
    } else if (result == ESP_ERR_INVALID_RESPONSE) {
      incrementSaturated(ssc_diagnostics.diagnostic_fault);
      restartable = true;
    } else if (result == ESP_ERR_TIMEOUT) {
      incrementSaturated(ssc_diagnostics.timeout);
      restartable = true;
    } else {
      incrementSaturated(ssc_diagnostics.i2c_error);
      restartable = true;
    }
    if (restartable)
      incrementSaturated(ssc_diagnostics.consecutive_restartable_faults);
    return restartable &&
           ssc_diagnostics.consecutive_restartable_faults >=
               board::kSscRestartConsecutiveFaults;
  };
  auto attemptSscReconnect = [&](uint64_t now_us, bool restart_initialized) {
    if (bus_result != ESP_OK || now_us < next_ssc_reconnect_us)
      return false;
    if (ssc.initialized() && !restart_initialized)
      return false;
    if (mission_snapshot.deployment_power_cutoff_latched ||
        recovery_requested.load(std::memory_order_acquire))
      return false;

    // SSCだけをbusからdetachする。共有I2C busとLPS25HBは維持する。
    ssc_ready.store(false, std::memory_order_release);
    ssc_recovery_pending = true;
    incrementSaturated(ssc_diagnostics.reconnect_attempts);
    esp_err_t cleanup = ssc.end();
    if (cleanup == ESP_ERR_INVALID_STATE)
      cleanup = ESP_OK;
    esp_err_t reconnect = cleanup;
    if (reconnect == ESP_OK)
      reconnect = ssc.begin(bus);
    next_ssc_reconnect_us =
        now_us + static_cast<uint64_t>(board::kSscReconnectIntervalMs) *
                     1'000ULL;
    if (reconnect == ESP_OK) {
      incrementSaturated(ssc_diagnostics.reconnect_success);
      ssc_diagnostics.consecutive_restartable_faults = 0;
      last_ssc_reconnect_error = ESP_OK;
      // begin/probe成功だけではdataのnormal statusを保証しない。次の正常readまで
      // ssc_ready=falseを維持し、古いairspeedをControlへ再接続しない。
      return true;
    }
    if (reconnect != last_ssc_reconnect_error ||
        ssc_diagnostics.reconnect_attempts == 1)
      printSscDiagnostics("reconnect failed", reconnect);
    last_ssc_reconnect_error = reconnect;
    return false;
  };

  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const bool calibration_active =
        preflight_calibration_active.load(std::memory_order_acquire);
    const uint32_t requested_generation =
        preflight_calibration_generation.load(std::memory_order_acquire);
    if (calibration_active && requested_generation != 0 &&
        requested_generation != calibration_generation) {
      pressure_conditioner.reset();
      snapshot.ssc_zero_valid = false;
      snapshot.airspeed_valid = false;
      calibration_generation = requested_generation;
    }
    if (now_us - last_ssc_us >= 2'500) {
      last_ssc_us = now_us;
      updateMissionSnapshot();
      if (!ssc.initialized())
        (void)attemptSscReconnect(now_us, false);
      if (ssc.initialized()) {
        SSCDRRN005PD2A5::Data data{};
        const int64_t read_started_us = esp_timer_get_time();
        const esp_err_t result = ssc.read(data);
        const int64_t read_elapsed_us =
            esp_timer_get_time() - read_started_us;
        const uint32_t read_duration_us = static_cast<uint32_t>(
            read_elapsed_us > 0 ? read_elapsed_us : 0);
        const bool restart_requested =
            recordSscRead(result, read_duration_us);
        if (result == ESP_OK) {
          const bool recovered = ssc_recovery_pending;
          ssc_ready.store(true, std::memory_order_release);
          ssc_recovery_pending = false;
          next_ssc_reconnect_us = 0;
          last_ssc_reconnect_error = ESP_OK;
          if (recovered)
            printSscDiagnostics("recovered", ESP_OK);
          snapshot.ssc_monotonic_us = now_us;
          snapshot.ssc_valid = true;
          snapshot.ssc_temperature_celsius = data.temperature_celsius;
          const bool capture_zero =
              preflight_calibration_active.load(std::memory_order_acquire) &&
              calibration_generation != 0 &&
              preflight_calibration_generation.load(
                  std::memory_order_acquire) == calibration_generation;
          (void)pressure_conditioner.updateZero(
              data.differential_pressure_pa, capture_zero);
          snapshot.ssc_zero_valid = pressure_conditioner.zeroValid();
          snapshot.airspeed_valid = false;

          const auto conditioned =
              pressure_conditioner.update(data.differential_pressure_pa);
          const bool static_pressure_fresh =
              snapshot.lps_valid && snapshot.lps_monotonic_us != 0 &&
              now_us >= snapshot.lps_monotonic_us &&
              now_us - snapshot.lps_monotonic_us <=
                  sensors::FreshnessThresholds::lps_us;
          if (!conditioned.zero_valid) {
            snapshot.airspeed_raw = static_cast<uint8_t>(
                protocol::quantization::AirspeedError::internal_invalid);
          } else if (conditioned.negative_beyond_tolerance) {
            snapshot.airspeed_raw = static_cast<uint8_t>(
                protocol::quantization::AirspeedError::
                    negative_differential_pressure);
          } else if (!conditioned.valid) {
            snapshot.airspeed_raw = static_cast<uint8_t>(
                protocol::quantization::AirspeedError::internal_invalid);
          } else if (!static_pressure_fresh) {
            snapshot.airspeed_raw = static_cast<uint8_t>(
                protocol::quantization::AirspeedError::
                    static_pressure_invalid);
          } else {
            const auto airspeed = sensors::computeSaintVenantAirspeed(
                snapshot.static_pressure_pa, conditioned.pressure_pa,
                data.temperature_celsius,
                flight_config::kAirData.pitot_coefficient_assumed);
            if (airspeed.valid) {
              snapshot.airspeed_mps = airspeed.airspeed_mps;
              snapshot.airspeed_raw = protocol::quantization::encodeAirspeed(
                  airspeed.airspeed_mps,
                  protocol::quantization::AirspeedError::internal_invalid);
              snapshot.airspeed_valid = snapshot.airspeed_raw <= 245;
            } else if (airspeed.error ==
                       sensors::AirspeedModelError::
                           negative_differential_pressure) {
              snapshot.airspeed_raw = static_cast<uint8_t>(
                  protocol::quantization::AirspeedError::
                      negative_differential_pressure);
            } else if (airspeed.error ==
                       sensors::AirspeedModelError::static_pressure_invalid) {
              snapshot.airspeed_raw = static_cast<uint8_t>(
                  protocol::quantization::AirspeedError::
                      static_pressure_invalid);
            } else {
              snapshot.airspeed_raw = static_cast<uint8_t>(
                  protocol::quantization::AirspeedError::internal_invalid);
            }
          }
        } else {
          setInitialSscError(result);
          if (restart_requested) {
            ssc_ready.store(false, std::memory_order_release);
            ssc_recovery_pending = true;
            const uint64_t reconnect_now_us =
                static_cast<uint64_t>(esp_timer_get_time());
            if (reconnect_now_us >= next_ssc_reconnect_us) {
              printSscDiagnostics("restart requested", result);
              (void)attemptSscReconnect(reconnect_now_us, true);
            }
          }
        }
      } else {
        ssc_ready.store(false, std::memory_order_release);
        ssc_recovery_pending = true;
        setInitialSscError(ESP_ERR_INVALID_STATE);
      }
      (void)xQueueOverwrite(air_data_queue, &snapshot);
    }

    if (now_us - last_lps_us >= 40'000) {
      last_lps_us = now_us;
      updateMissionSnapshot();
      double pressure_hpa = 0.0;
      esp_err_t read_result = ESP_ERR_INVALID_STATE;
      LPS25HB::Data data{};
      if (lps.initialized())
        read_result = lps.read(data);
      if (read_result == ESP_OK) {
        snapshot.lps_monotonic_us = now_us;
        snapshot.lps_valid = true;
        snapshot.static_pressure_pa = data.pressure_pa;
        pressure_hpa = static_cast<double>(data.pressure_pa) / 100.0;
        snapshot.pressure_raw = protocol::quantization::encodeLpsPressure(
            pressure_hpa, protocol::quantization::LpsPressureError::unknown);
        snapshot.temperature_raw =
            protocol::quantization::encodeLpsTemperature(
                data.temperature_celsius,
                protocol::quantization::LpsTemperatureError::unknown);
      } else {
        setInitialLpsError(read_result);
      }
      snapshot.flight = flight_logic.update(
          mission_snapshot.flight_epoch, mission_snapshot.state,
          mission_snapshot.elapsed_us, pressure_hpa, read_result == ESP_OK);
      if (snapshot.flight.pressure_apex_detected)
        pressure_deployment_epoch.store(snapshot.flight.flight_epoch,
                                        std::memory_order_release);
      (void)xQueueOverwrite(air_data_queue, &snapshot);
    }
    resetWatchdog();
    vTaskDelay(1);
  }
}

void canTask(void *) {
  addWatchdog();
  CANCREATE can;
  const esp_err_t begin_result = can.begin(canConfig());
  can_healthy.store(begin_result == ESP_OK, std::memory_order_release);
  std::printf("CanTask begin=%s bitrate=125000\n",
              esp_err_to_name(begin_result));
  protocol::TelemetrySequences sequences;
  TickType_t last_status = xTaskGetTickCount();
  TickType_t last_fast = xTaskGetTickCount();
  TickType_t last_control_roll = xTaskGetTickCount();
  TickType_t last_lps = xTaskGetTickCount();
  TickType_t last_time_request = xTaskGetTickCount();
  RuntimeStatus latest{};
  EventRequest pending_event{};
  bool event_pending = false;
  uint8_t event_sequence = 0;
  bool was_bus_off = false;
  uint8_t time_request_id = 1;
  uint8_t recovery_mode_sequence = 0;
  protocol::RecoveryStatusMessage pending_recovery_status{};
  bool recovery_status_pending = false;
  protocol::RecoveryLogData pending_recovery_log_data{};
  bool recovery_log_data_pending = false;
  protocol::CommandResult pending_command_result{};
  bool command_result_pending = false;
  protocol::MissionState last_status_state = protocol::MissionState::unknown;
  uint8_t last_control_roll_capture_event_sequence = 0;
  uint32_t last_control_roll_status_signature =
      protocol::controlRollStatusSignature(
          latest.control_roll_reference_unwrapped_raw,
          latest.roll_deviation_unwrapped_raw, latest.control_roll_flags);
  for (;;) {
    if (begin_result == ESP_OK) {
      CANCREATE::Frame frame{};
      while (can.read(frame, avi::Timeout::noWait()) == ESP_OK) {
        const auto input = toProtocol(frame);
        if (input.identifier ==
            static_cast<uint16_t>(protocol::CanId::recovery_control)) {
          protocol::RecoveryControl control{};
          if (protocol::decode(input, control) == protocol::CodecError::none) {
            RecoveryRequest request{};
            request.kind = RecoveryRequest::Kind::control;
            request.control = control;
            if (xQueueSend(recovery_queue, &request, 0) != pdTRUE) {
              const protocol::RecoveryStatusMessage busy{
                  control.opcode, control.transfer_id,
                  protocol::RecoveryStatusCode::busy, control.source, 0};
              (void)xQueueSend(recovery_status_queue, &busy, 0);
            }
          }
        } else if (input.identifier ==
            static_cast<uint16_t>(protocol::CanId::generic_command_request)) {
          protocol::GenericCommandRequest request{};
          if (protocol::decode(input, request) == protocol::CodecError::none) {
            if (recovery_only_mode.load(std::memory_order_acquire)) {
              const protocol::CommandResult rejected{
                  request.transaction_id, request.command,
                  protocol::CommandPhase::rejected,
                  protocol::CommandReason::invalid_state, 0};
              enqueueResult(rejected, false);
              continue;
            }
            const MissionCommandEnvelope envelope{request};
            if (xQueueSend(command_queue, &envelope, 0) != pdTRUE) {
              const protocol::CommandResult busy{
                  request.transaction_id, request.command,
                  protocol::CommandPhase::rejected,
                  protocol::CommandReason::busy, 0};
              enqueueResult(busy, false);
            }
          }
        } else if (input.identifier == static_cast<uint16_t>(
                                           protocol::CanId::actuator_emergency_stop)) {
          protocol::EmergencyStop emergency{};
          if (protocol::decode(input, protocol::CanId::actuator_emergency_stop,
                               emergency) == protocol::CodecError::none) {
            if (emergency.transaction_id != 0) {
              latchPhysicalEmergency(emergency.transaction_id, false);
            } else {
              mission::EmergencyDecision result{};
              if (xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                result = command_executor.actuatorEmergency(
                    emergency.transaction_id,
                    protocol::MissionState::unknown);
                xSemaphoreGive(executor_mutex);
              } else {
                result.result = {emergency.transaction_id, 0xF0,
                                 protocol::CommandPhase::rejected,
                                 protocol::CommandReason::busy, 0};
              }
              enqueueResult(result.result, true);
            }
          }
        } else if (input.identifier == static_cast<uint16_t>(
                                           protocol::CanId::liftoff_detection_emergency_stop)) {
          protocol::EmergencyStop emergency{};
          if (protocol::decode(
                  input,
                  protocol::CanId::liftoff_detection_emergency_stop,
                  emergency) == protocol::CodecError::none) {
            if (emergency.transaction_id != 0) {
              latchPhysicalEmergency(emergency.transaction_id, true);
            } else {
              enqueueResult(command_executor.liftoffEmergencyResult(
                                emergency.transaction_id, false),
                            true);
            }
          }
        } else if (input.identifier ==
                   static_cast<uint16_t>(protocol::CanId::time_response)) {
          protocol::TimeResponse response{};
          if (protocol::decode(input, response) == protocol::CodecError::none &&
              response.source != protocol::TimeSource::invalid &&
              response.unix_seconds != 0 && response.milliseconds < 1000 &&
              xSemaphoreTake(time_mutex, 0) == pdTRUE) {
            time_state = {response.source, response.unix_seconds,
                          response.milliseconds,
                          static_cast<uint64_t>(esp_timer_get_time()),
                          response.request_id, true};
            xSemaphoreGive(time_mutex);
          }
        }
      }
      if (!command_result_pending &&
          xQueueReceive(result_queue, &pending_command_result, 0) == pdTRUE)
        command_result_pending = true;
      if (command_result_pending &&
          writeFrame(can, protocol::encode(pending_command_result)) ==
              ESP_OK)
        command_result_pending = false;
      if (!recovery_status_pending &&
          xQueueReceive(recovery_status_queue, &pending_recovery_status, 0) ==
              pdTRUE)
        recovery_status_pending = true;
      if (recovery_status_pending &&
          writeFrame(can, protocol::encode(pending_recovery_status)) ==
              ESP_OK)
        recovery_status_pending = false;
      if (!recovery_log_data_pending &&
          xQueueReceive(recovery_log_data_queue, &pending_recovery_log_data, 0) ==
              pdTRUE)
        recovery_log_data_pending = true;
      if (recovery_log_data_pending &&
          writeFrame(can, protocol::encode(pending_recovery_log_data)) == ESP_OK)
        recovery_log_data_pending = false;
      if (recovery_mode_command_pending.load(std::memory_order_acquire)) {
        const auto reason = static_cast<protocol::RecoveryModeReason>(
            recovery_mode_reason.load(std::memory_order_acquire));
        const protocol::RecoveryModeCommand recovery_mode{
            recovery_mode_sequence,
            protocol::RecoveryMode::enter_recovery_beacon, reason};
        if (writeFrame(can, protocol::encode(recovery_mode)) == ESP_OK) {
          ++recovery_mode_sequence;
          recovery_mode_command_pending.store(false, std::memory_order_release);
          recovery_mode_command_sent.store(true, std::memory_order_release);
        }
      }
      EventRequest event{};
      const uint16_t parachute_failure_flag = protocol::eventFlag(
          protocol::MissionEventFlag::parachute_deployment_failure);
      while (xQueueReceive(event_queue, &event, 0) == pdTRUE) {
        const bool failure_already_pending =
            (pending_event.flags & parachute_failure_flag) != 0;
        pending_event.flags |= event.flags;
        pending_event.state = event.state;
        pending_event.elapsed_raw = event.elapsed_raw;
        if (!failure_already_pending ||
            (event.flags & parachute_failure_flag) != 0)
          pending_event.detail = event.detail;
        event_pending = true;
      }
      const uint16_t overflow_flags =
          event_overflow_latch.exchange(0, std::memory_order_acq_rel);
      if (overflow_flags != 0) {
        pending_event.flags |= overflow_flags;
        if ((overflow_flags & parachute_failure_flag) != 0)
          pending_event.detail = parachute_failure_overflow_detail.exchange(
              0, std::memory_order_acq_rel);
        event_pending = true;
      }
      if (event_pending) {
        const protocol::MissionEvent mission_event{
            event_sequence, pending_event.flags, pending_event.state,
            pending_event.elapsed_raw, pending_event.detail};
        if (writeFrame(can, protocol::encode(mission_event)) == ESP_OK) {
          ++event_sequence;
          pending_event = {};
          event_pending = false;
        }
      }
      (void)xQueuePeek(status_queue, &latest, 0);
      const TickType_t now = xTaskGetTickCount();
      const bool recovery_only =
          recovery_only_mode.load(std::memory_order_acquire);
      bool time_valid = false;
      if (xSemaphoreTake(time_mutex, 0) == pdTRUE) {
        time_valid = time_state.valid;
        xSemaphoreGive(time_mutex);
      }
      if (!recovery_only && !time_valid &&
          now - last_time_request >= pdMS_TO_TICKS(1'000)) {
        last_time_request = now;
        if (writeFrame(can, protocol::encode(
                                protocol::TimeRequest{time_request_id})) ==
            ESP_OK) {
          ++time_request_id;
          if (time_request_id == 0)
            time_request_id = 1;
        }
      }
      if (!recovery_only && now - last_fast >= pdMS_TO_TICKS(10)) {
        last_fast = now;
        const protocol::KinematicsTelemetry kinematics{
            sequences.next(protocol::CanId::kinematics_telemetry),
            latest.roll_raw, latest.roll_rate_raw, latest.fin_angle_raw,
            latest.fin_rate_raw};
        (void)writeFrame(can, protocol::encode(kinematics));
        if (latest.state != protocol::MissionState::command_receive &&
            latest.state != protocol::MissionState::descent) {
          const protocol::ControlTelemetry control{
              sequences.next(protocol::CanId::control_telemetry),
              latest.requested_torque_raw,
              protocol::quantization::encodeFlightElapsed(
                  static_cast<double>(latest.flight_elapsed_us) / 1.0e6,
                  protocol::quantization::TimeError::unavailable)};
          (void)writeFrame(can, protocol::encode(control));
        }
        if (latest.state == protocol::MissionState::descent) {
          const uint16_t failure = static_cast<uint16_t>(
              parachute_deployment_failure.load(std::memory_order_acquire) &
              0x0FU);
          const uint16_t persistence_corrupt =
              parachute_persistence_corrupt.load(std::memory_order_acquire)
                  ? uint16_t{1U << 4U}
                  : uint16_t{0};
          const protocol::DescentCoreTelemetry descent{
              sequences.next(protocol::CanId::descent_core_telemetry),
              static_cast<uint16_t>(failure | persistence_corrupt),
              parachute_angle_raw.load(std::memory_order_acquire)};
          (void)writeFrame(can, protocol::encode(descent));
        }
        if (!latest.deployment_power_cutoff) {
          const protocol::AirspeedTelemetry airspeed{
              sequences.next(protocol::CanId::airspeed_telemetry),
              latest.airspeed_raw};
          (void)writeFrame(can, protocol::encode(airspeed));
        }
      }
      if (!recovery_only && now - last_lps >= pdMS_TO_TICKS(40)) {
        last_lps = now;
        if (!latest.deployment_power_cutoff) {
          const protocol::LpsTelemetry lps{
              sequences.next(protocol::CanId::lps_telemetry),
              latest.lps_pressure_raw, latest.lps_temperature_raw};
          (void)writeFrame(can, protocol::encode(lps));
        }
      }
      const bool new_reference_capture =
          latest.control_roll_reference_capture_event_sequence !=
          last_control_roll_capture_event_sequence;
      const uint32_t control_roll_status_signature =
          protocol::controlRollStatusSignature(
              latest.control_roll_reference_unwrapped_raw,
              latest.roll_deviation_unwrapped_raw, latest.control_roll_flags);
      const bool control_roll_status_changed =
          control_roll_status_signature != last_control_roll_status_signature;
      if (!recovery_only &&
          (new_reference_capture || control_roll_status_changed ||
           now - last_control_roll >= pdMS_TO_TICKS(100))) {
        last_control_roll = now;
        uint8_t flags = latest.control_roll_flags;
        if (new_reference_capture)
          flags |= protocol::ControlRollTelemetryV2::
              reference_captured_since_previous_frame;
        const protocol::ControlRollTelemetryV2 telemetry{
            sequences.next(protocol::CanId::control_roll_telemetry_v2),
            latest.control_roll_reference_unwrapped_raw,
            latest.roll_deviation_unwrapped_raw, flags,
            latest.control_roll_reference_capture_event_sequence};
        if (writeFrame(can, protocol::encode(telemetry)) == ESP_OK) {
          last_control_roll_capture_event_sequence =
              latest.control_roll_reference_capture_event_sequence;
          last_control_roll_status_signature = control_roll_status_signature;
        }
      }
      if (latest.state != last_status_state ||
          now - last_status >= pdMS_TO_TICKS(100)) {
        last_status = now;
        last_status_state = latest.state;
        protocol::MissionStatusTelemetry telemetry{};
        telemetry.sequence =
            sequences.next(protocol::CanId::mission_status_telemetry);
        telemetry.state = latest.state;
        telemetry.flight_status = latest.flight_status;
        telemetry.config_flags = latest.config_flags;
        telemetry.fin_mode = latest.fin_mode;
        telemetry.para_mode = latest.para_mode;
        telemetry.parachute_angle_raw =
            parachute_angle_raw.load(std::memory_order_acquire);
        (void)writeFrame(can, protocol::encode(telemetry));
        uint8_t persistence_flags = 0;
        if (parachute_config_load_complete.load(std::memory_order_acquire))
          persistence_flags |= 1U << 0U;
        if (parachute_persistence_ready.load(std::memory_order_acquire))
          persistence_flags |= 1U << 1U;
        if (parachute_persistence_corrupt.load(std::memory_order_acquire))
          persistence_flags |= 1U << 2U;
        if (result_queue_overflow.load(std::memory_order_relaxed) != 0)
          persistence_flags |= 1U << 7U;
        const bool flight_elapsed_valid =
            latest.state == protocol::MissionState::engine_burn ||
            latest.state == protocol::MissionState::control ||
            latest.state == protocol::MissionState::descent;
        const protocol::PowerTimeTelemetry power{
            sequences.next(protocol::CanId::power_time_telemetry),
            logic_voltage_raw.load(std::memory_order_acquire),
            motor_voltage_raw.load(std::memory_order_acquire),
            flight_elapsed_valid
                ? protocol::quantization::encodeDescentElapsed(
                      static_cast<double>(latest.flight_elapsed_us) / 1.0e6,
                      protocol::quantization::TimeError::unavailable)
                : static_cast<uint16_t>(
                      0xFFF0U |
                      static_cast<uint8_t>(
                          protocol::quantization::TimeError::pre_liftoff)),
            recovery_boot::recoveryElapsedRaw(
                recovery_requested.load(std::memory_order_acquire),
                recovery_only),
            persistence_flags};
        (void)writeFrame(can, protocol::encode(power));
        if (!recovery_only) {
          const protocol::AttitudeTiltTelemetry tilt{
              sequences.next(protocol::CanId::attitude_tilt_telemetry), 121,
              511};
          (void)writeFrame(can, protocol::encode(tilt));
        }
      }
      CANCREATE::Status status{};
      if (can.getStatus(status) == ESP_OK) {
        const bool bus_off = status.state == CANCREATE::State::bus_off;
        can_healthy.store(status.state == CANCREATE::State::running,
                          std::memory_order_release);
        if (bus_off) {
          (void)can.recover(avi::Timeout::milliseconds(100));
        } else if (was_bus_off) {
          enqueueEvent(protocol::eventFlag(
                           protocol::MissionEventFlag::can_recovery),
                       latest.state);
        }
        was_bus_off = bus_off;
      } else {
        can_healthy.store(false, std::memory_order_release);
      }
    }
    resetWatchdog();
    vTaskDelay(1);
  }
}

protocol::CommandReason transitionReason(mission::TransitionResult result) {
  switch (result) {
  case mission::TransitionResult::completed:
    return protocol::CommandReason::none;
  case mission::TransitionResult::invalid_state:
    return protocol::CommandReason::invalid_state;
  case mission::TransitionResult::not_configured:
    return protocol::CommandReason::not_configured;
  case mission::TransitionResult::runtime_unavailable:
    return protocol::CommandReason::internal_error;
  }
  return protocol::CommandReason::internal_error;
}

void commandWorkerTask(void *) {
  addWatchdog();
  for (;;) {
    MissionCommandEnvelope envelope{};
    if (xQueueReceive(command_queue, &envelope, pdMS_TO_TICKS(10)) != pdTRUE) {
      resetWatchdog();
      continue;
    }
    {
      mission::CommandContext context{};
      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        const auto snapshot = state_machine.snapshot();
        context.state = snapshot.state;
        context.deployment_power_cutoff_done =
            snapshot.deployment_power_cutoff_latched;
        xSemaphoreGive(state_mutex);
      } else {
        context.state = protocol::MissionState::unknown;
        context.deployment_power_cutoff_done = false;
      }
      context.resources_preallocated =
          flight_config::nonBypassFlightConfigurationReady() &&
          flash_log_ready.load(std::memory_order_acquire) &&
          sd_log_ready.load(std::memory_order_acquire);
      context.persistence_load_complete =
          parachute_config_load_complete.load(std::memory_order_acquire);
      context.persistence_runtime_available =
          parachute_persistence_ready.load(std::memory_order_acquire);
      context.fin_available = true;
      context.parachute_available = parachute_command_queue != nullptr;
      context.fin_safe_commands_supported = true;
      context.calibration_supported = true;
      context.storage_export_supported = storage_export_queue != nullptr;
      if (xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        const protocol::CommandResult busy{
            envelope.request.transaction_id, envelope.request.command,
            protocol::CommandPhase::rejected, protocol::CommandReason::busy,
            0};
        enqueueResult(busy, false);
        resetWatchdog();
        continue;
      }
      envelope.decision = command_executor.begin(envelope.request, context);
      xSemaphoreGive(executor_mutex);
      enqueueResult(envelope.decision.result, false);
      ParachuteCommandRequest parachute_command{};
      const bool parachute_domain =
          envelope.decision.domain == mission::CommandDomain::parachute;
      const bool storage_domain =
          envelope.decision.domain == mission::CommandDomain::storage;
      if (parachute_domain)
        parachute_command = {ParachuteCommandRequest::Kind::generic,
                             envelope.request, {}};
      const StorageExportRequest storage_request{
          envelope.request.transaction_id};
      BaseType_t queued = pdTRUE;
      if (envelope.decision.execute) {
        if (parachute_domain)
          queued = xQueueSend(parachute_command_queue, &parachute_command, 0);
        else if (storage_domain)
          queued = xQueueSend(storage_export_queue, &storage_request, 0);
        else
          queued = xQueueSend(transition_queue, &envelope, 0);
      }
      if (envelope.decision.execute && queued != pdTRUE) {
        if (xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          const auto failed = command_executor.finish(
              envelope.request.transaction_id, protocol::CommandPhase::failed,
              protocol::CommandReason::busy);
          xSemaphoreGive(executor_mutex);
          enqueueResult(failed, false);
        }
      }
    }
    resetWatchdog();
  }
}

void housekeepingTask(void *) {
  addWatchdog();
  bool logic_numeric_valid = false;
  bool motor_numeric_valid = false;

  auto publishVoltage = [](const bringup::power::AdcReading &reading,
                           std::atomic<uint8_t> &raw,
                           bool &numeric_valid) {
    if (!reading.calibrated_valid) {
      if (!numeric_valid) {
        raw.store(static_cast<uint8_t>(
                      protocol::quantization::BatteryError::adc_error),
                  std::memory_order_release);
      }
      return;
    }

    const uint8_t encoded = protocol::quantization::encodeBatteryVoltage(
        reading.source_voltage_v,
        protocol::quantization::BatteryError::adc_error);
    raw.store(encoded, std::memory_order_release);
    numeric_valid = encoded <= 240U;
  };

  for (;;) {
    if (!bringup::power::initialized())
      (void)bringup::power::initialize();

    if (bringup::power::initialized()) {
      bringup::power::PowerSample sample{};
      (void)bringup::power::read(sample);
      publishVoltage(sample.logic, logic_voltage_raw, logic_numeric_valid);
      publishVoltage(sample.motor, motor_voltage_raw, motor_numeric_valid);
      const uint64_t power_now_us =
          static_cast<uint64_t>(esp_timer_get_time());
      if (sample.motor.calibrated_valid &&
          std::isfinite(sample.motor.source_voltage_v) &&
          sample.motor.source_voltage_v >= 0.0F) {
        const double millivolts =
            static_cast<double>(sample.motor.source_voltage_v) * 1'000.0;
        if (millivolts <=
            static_cast<double>(std::numeric_limits<uint32_t>::max())) {
          motor_bus_millivolts.store(
              static_cast<uint32_t>(std::lround(millivolts)),
              std::memory_order_release);
          motor_bus_voltage_valid.store(true, std::memory_order_release);
        }
      } else {
        motor_bus_voltage_valid.store(false, std::memory_order_release);
      }
      sensors::power_presence_runtime::observeRaw(
          logic_voltage_raw.load(std::memory_order_acquire),
          motor_voltage_raw.load(std::memory_order_acquire), power_now_us);
      logic_power_present.store(
          sensors::power_presence_runtime::logicPresent(power_now_us),
          std::memory_order_release);
      motor_power_present.store(
          sensors::power_presence_runtime::motorPresent(power_now_us),
          std::memory_order_release);
    }
    resetWatchdog();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

constexpr char kParachuteNvsNamespace[] = "para_cfg";
constexpr char kParachuteOpenNvsKey[] = "open_v1";
constexpr char kParachuteCloseNvsKey[] = "close_v1";

const char *parachuteNvsKey(actuators::ParachuteEndpoint endpoint) {
  return endpoint == actuators::ParachuteEndpoint::open
             ? kParachuteOpenNvsKey
             : kParachuteCloseNvsKey;
}

uint16_t parachuteCorruptionDetail(
    actuators::ParachuteEndpoint endpoint,
    actuators::ParachuteBlobError reason) {
  const uint16_t endpoint_bit =
      endpoint == actuators::ParachuteEndpoint::close ? uint16_t{1U << 8U}
                                                       : uint16_t{0};
  return static_cast<uint16_t>(endpoint_bit |
                               static_cast<uint8_t>(reason));
}

uint16_t persistenceRuntimeDetail(esp_err_t error) {
  return static_cast<uint16_t>(
      0x8000U | (static_cast<uint32_t>(error) & 0x7FFFU));
}

esp_err_t loadParachuteEndpoint(
    nvs_handle_t handle, actuators::ParachuteEndpoint endpoint, bool &valid,
    uint16_t &count, actuators::ParachuteBlobError &corruption_reason) {
  valid = false;
  corruption_reason = actuators::ParachuteBlobError::none;
  std::size_t size = 0;
  esp_err_t result =
      nvs_get_blob(handle, parachuteNvsKey(endpoint), nullptr, &size);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    return ESP_OK;
  if (result != ESP_OK)
    return result;
  if (size != actuators::kParachuteEndpointBlobSize) {
    corruption_reason = actuators::ParachuteBlobError::wrong_size;
    return ESP_OK;
  }
  actuators::ParachuteEndpointBlob blob{};
  result = nvs_get_blob(handle, parachuteNvsKey(endpoint), blob.data(), &size);
  if (result != ESP_OK)
    return result;
  const auto decoded =
      actuators::decodeParachuteEndpoint(blob.data(), size, endpoint);
  if (!decoded.valid()) {
    corruption_reason = decoded.error;
    return ESP_OK;
  }
  valid = true;
  count = decoded.angle->count();
  return ESP_OK;
}

bool verifyParachuteEndpoint(nvs_handle_t handle,
                             actuators::ParachuteEndpoint endpoint,
                             uint16_t expected_count) {
  bool valid = false;
  auto corruption_reason = actuators::ParachuteBlobError::none;
  uint16_t count = 0;
  return loadParachuteEndpoint(handle, endpoint, valid, count,
                               corruption_reason) == ESP_OK &&
         valid && corruption_reason == actuators::ParachuteBlobError::none &&
         count == expected_count;
}

bool rollbackParachuteEndpoint(
    nvs_handle_t handle, const ParachutePersistenceRequest &request) {
  esp_err_t result = ESP_OK;
  if (request.previous_valid) {
    const auto previous =
        actuators::AbsoluteParachuteAngle::fromCount(request.previous_count);
    if (!previous.has_value())
      return false;
    const auto blob =
        actuators::encodeParachuteEndpoint(request.endpoint, *previous);
    result = nvs_set_blob(handle, parachuteNvsKey(request.endpoint),
                          blob.data(), blob.size());
    if (result == ESP_OK)
      result = nvs_commit(handle);
    return result == ESP_OK && verifyParachuteEndpoint(
                                   handle, request.endpoint,
                                   request.previous_count);
  }

  result = nvs_erase_key(handle, parachuteNvsKey(request.endpoint));
  if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND)
    return false;
  if (nvs_commit(handle) != ESP_OK)
    return false;
  std::size_t size = 0;
  return nvs_get_blob(handle, parachuteNvsKey(request.endpoint), nullptr,
                      &size) == ESP_ERR_NVS_NOT_FOUND;
}

bool saveParachuteEndpoint(const ParachutePersistenceRequest &request,
                           bool &rollback_failed) {
  rollback_failed = false;
  const auto angle =
      actuators::AbsoluteParachuteAngle::fromCount(request.count);
  if (!angle.has_value())
    return false;
  nvs_handle_t handle{};
  if (nvs_open(kParachuteNvsNamespace, NVS_READWRITE, &handle) != ESP_OK)
    return false;
  const auto blob =
      actuators::encodeParachuteEndpoint(request.endpoint, *angle);
  esp_err_t result = nvs_set_blob(handle, parachuteNvsKey(request.endpoint),
                                  blob.data(), blob.size());
  bool candidate_may_be_committed = result == ESP_OK;
  if (result == ESP_OK)
    result = nvs_commit(handle);
  if (result == ESP_OK &&
      verifyParachuteEndpoint(handle, request.endpoint, request.count)) {
    nvs_close(handle);
    return true;
  }
  if (candidate_may_be_committed)
    rollback_failed = !rollbackParachuteEndpoint(handle, request);
  nvs_close(handle);
  return false;
}

void internalFlashTask(void *) {
  addWatchdog();
  ParachutePersistenceResponse load_response{};
  load_response.kind = ParachutePersistenceResponse::Kind::load;
  const esp_err_t flash_log_result = internal_flash_log.openExisting();
  flash_log_ready.store(
      flash_log_result == ESP_OK && !internal_flash_log.hasData(),
      std::memory_order_release);
  if (flash_log_result != ESP_OK) {
    flash_log_failed.store(true, std::memory_order_release);
    enqueueEvent(protocol::eventFlag(
                     protocol::MissionEventFlag::persistence_error),
                 protocol::MissionState::command_receive, 0,
                 static_cast<uint16_t>(0x9000U |
                                       (static_cast<uint32_t>(flash_log_result) &
                                        0x0FFFU)));
  }

  const esp_err_t nvs_result = nvs_flash_init();
  esp_err_t persistence_runtime_error = nvs_result;
  load_response.persistence_ready = nvs_result == ESP_OK;
  if (nvs_result == ESP_OK) {
    nvs_handle_t handle{};
    const esp_err_t open_result =
        nvs_open(kParachuteNvsNamespace, NVS_READONLY, &handle);
    if (open_result == ESP_OK) {
      auto open_corruption = actuators::ParachuteBlobError::none;
      auto close_corruption = actuators::ParachuteBlobError::none;
      const esp_err_t open_load = loadParachuteEndpoint(
          handle, actuators::ParachuteEndpoint::open,
          load_response.open_valid, load_response.open_count,
          open_corruption);
      const esp_err_t close_load = loadParachuteEndpoint(
          handle, actuators::ParachuteEndpoint::close,
          load_response.close_valid, load_response.close_count,
          close_corruption);
      nvs_close(handle);
      load_response.corruption_detected =
          open_corruption != actuators::ParachuteBlobError::none ||
          close_corruption != actuators::ParachuteBlobError::none;
      if (open_corruption != actuators::ParachuteBlobError::none) {
        std::printf("parachute NVS open_v1 corrupted: reason=%u\n",
                    static_cast<unsigned>(open_corruption));
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::persistence_error),
                     protocol::MissionState::command_receive, 0,
                     parachuteCorruptionDetail(
                         actuators::ParachuteEndpoint::open,
                         open_corruption));
      }
      if (close_corruption != actuators::ParachuteBlobError::none) {
        std::printf("parachute NVS close_v1 corrupted: reason=%u\n",
                    static_cast<unsigned>(close_corruption));
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::persistence_error),
                     protocol::MissionState::command_receive, 0,
                     parachuteCorruptionDetail(
                         actuators::ParachuteEndpoint::close,
                         close_corruption));
      }
      if (open_load != ESP_OK) {
        std::printf("parachute NVS open_v1 load failed: %s\n",
                    esp_err_to_name(open_load));
        persistence_runtime_error = open_load;
      }
      if (close_load != ESP_OK) {
        std::printf("parachute NVS close_v1 load failed: %s\n",
                    esp_err_to_name(close_load));
        if (persistence_runtime_error == ESP_OK)
          persistence_runtime_error = close_load;
      }
      load_response.persistence_ready =
          open_load == ESP_OK && close_load == ESP_OK;
    } else if (open_result != ESP_ERR_NVS_NOT_FOUND) {
      load_response.persistence_ready = false;
      persistence_runtime_error = open_result;
    }
  }
  load_response.success = load_response.persistence_ready;
  if (!load_response.persistence_ready) {
    enqueueEvent(protocol::eventFlag(
                     protocol::MissionEventFlag::persistence_error),
                 protocol::MissionState::command_receive, 0,
                 persistenceRuntimeDetail(persistence_runtime_error));
  }
  (void)xQueueSend(parachute_persistence_response_queue, &load_response,
                   pdMS_TO_TICKS(100));
  mission::RecoveryRuntime recovery;
  bool enter_waiting = false;
  protocol::RecoveryModeReason enter_reason =
      protocol::RecoveryModeReason::auto_elapsed_120;
  uint64_t recovery_entry_deadline_us = 0;
  uint64_t recovery_wake_deadline_us = 0;
  bool flash_flush_pending = false;
  RecoveryDumpCursor flash_dump{};
  if (recovery_only_mode.load(std::memory_order_acquire)) {
    recovery_requested.store(true, std::memory_order_release);
    recovery_motor_safe.store(true, std::memory_order_release);
    const bool valid_wake =
        recovery_wake_valid.load(std::memory_order_acquire);
    if (valid_wake)
      (void)recovery.wake(true, true);
    const protocol::RecoveryStatusMessage wake_status{
        protocol::RecoveryOpcode::wake, 0,
        valid_wake ? protocol::RecoveryStatusCode::ready
                   : protocol::RecoveryStatusCode::internal_error,
        protocol::RecoverySource::internal_flash, 0};
    (void)xQueueSend(recovery_status_queue, &wake_status, 0);
    recovery_mode_reason.store(
        static_cast<uint8_t>(protocol::RecoveryModeReason::recovery_wake_retry),
        std::memory_order_release);
    recovery_mode_command_sent.store(false, std::memory_order_release);
    recovery_mode_command_pending.store(true, std::memory_order_release);
    recovery_wake_deadline_us =
        static_cast<uint64_t>(esp_timer_get_time()) + 2'000'000;
  }
  for (;;) {
    flight_log::SerializedRecord flash_record{};
    while (xQueueReceive(flash_log_queue, &flash_record, 0) == pdTRUE) {
      if (internal_flash_log.append(flash_record) != ESP_OK) {
        flash_log_drop_count.fetch_add(1, std::memory_order_relaxed);
        if (!flash_log_failed.exchange(true, std::memory_order_acq_rel))
          enqueueEvent(protocol::eventFlag(
                           protocol::MissionEventFlag::persistence_error),
                       protocol::MissionState::unknown, 0, 0x9001);
      }
    }

    ParachutePersistenceRequest persistence_request{};
    while (xQueueReceive(parachute_persistence_request_queue,
                         &persistence_request, 0) == pdTRUE) {
      bool rollback_failed = false;
      const bool saved = load_response.persistence_ready &&
                         saveParachuteEndpoint(persistence_request,
                                               rollback_failed);
      const ParachutePersistenceResponse response{
          ParachutePersistenceResponse::Kind::save,
          persistence_request.transaction_id, persistence_request.endpoint,
          saved, load_response.persistence_ready, false};
      if (!saved) {
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::persistence_error),
                     protocol::MissionState::command_receive, 0,
                     rollback_failed ? uint16_t{0x8002}
                                     : uint16_t{0x8001});
      }
      (void)xQueueSend(parachute_persistence_response_queue, &response,
                       pdMS_TO_TICKS(100));
    }
    RecoveryRequest request{};
    while (xQueueReceive(recovery_queue, &request, 0) == pdTRUE) {
      if (request.kind == RecoveryRequest::Kind::enter) {
        if (recovery_requested.load(std::memory_order_acquire))
          continue;
        bool eligible = false;
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          const auto snapshot = state_machine.snapshot();
          eligible = snapshot.state == protocol::MissionState::descent &&
                     snapshot.deployment_power_cutoff_latched;
          xSemaphoreGive(state_mutex);
        }
        if (!eligible || !recovery.requestEntry())
          continue;
        enter_reason = request.reason;
        recovery_requested.store(true, std::memory_order_release);
        recovery_sd_flushed.store(false, std::memory_order_release);
        persistence_flushed.store(false, std::memory_order_release);
        recovery_mode_command_pending.store(false, std::memory_order_release);
        recovery_mode_command_sent.store(false, std::memory_order_release);
        const PersistenceSignal signal = PersistenceSignal::flush_and_safe;
        if (xQueueOverwrite(persistence_queue, &signal) == pdTRUE) {
          flash_flush_pending = true;
          enter_waiting = true;
        }
        continue;
      }

      if (request.control.source ==
          protocol::RecoverySource::mission_sd_latest_flight) {
        if (xQueueSend(sd_recovery_queue, &request.control, 0) != pdTRUE) {
          const protocol::RecoveryStatusMessage busy{
              request.control.opcode, request.control.transfer_id,
              protocol::RecoveryStatusCode::busy, request.control.source, 0};
          (void)xQueueSend(recovery_status_queue, &busy, 0);
        }
        continue;
      }

      protocol::RecoveryStatusCode status =
          protocol::RecoveryStatusCode::invalid_state;
      if (request.control.opcode == protocol::RecoveryOpcode::wake) {
        status = recovery_only_mode.load(std::memory_order_acquire)
                     ? protocol::RecoveryStatusCode::ready
                     : protocol::RecoveryStatusCode::invalid_state;
      } else if (!recovery_only_mode.load(std::memory_order_acquire)) {
        status = protocol::RecoveryStatusCode::invalid_state;
      } else if (request.control.opcode ==
                 protocol::RecoveryOpcode::start_log_dump) {
        const uint32_t total_size = internal_flash_log.size();
        if (!internal_flash_log.ready()) {
          status = protocol::RecoveryStatusCode::source_unavailable;
        } else if (flash_dump.active) {
          status = protocol::RecoveryStatusCode::busy;
        } else if (request.control.offset > total_size) {
          status = protocol::RecoveryStatusCode::invalid_argument;
        } else {
          const uint32_t available = total_size - request.control.offset;
          const uint32_t requested =
              request.control.length == 0
                  ? available
                  : std::min(request.control.length, available);
          flash_dump = {requested != 0, request.control,
                        request.control.offset, requested, 0};
          status = requested == 0 ? protocol::RecoveryStatusCode::complete
                                  : protocol::RecoveryStatusCode::dumping;
        }
      } else if (request.control.opcode ==
                 protocol::RecoveryOpcode::stop_log_dump) {
        if (flash_dump.active &&
            flash_dump.control.transfer_id == request.control.transfer_id) {
          flash_dump.active = false;
          status = protocol::RecoveryStatusCode::aborted;
        } else {
          status = protocol::RecoveryStatusCode::invalid_state;
        }
      }
      const protocol::RecoveryStatusMessage response{
          request.control.opcode, request.control.transfer_id, status,
          request.control.source, internal_flash_log.size()};
      (void)xQueueSend(recovery_status_queue, &response, 0);
    }

    if (flash_dump.active) {
      protocol::RecoveryLogData data{};
      data.transfer_id = flash_dump.control.transfer_id;
      data.sequence = flash_dump.sequence;
      const std::size_t requested = std::min<std::size_t>(
          data.data.size(), flash_dump.remaining);
      std::size_t read_size = 0;
      const esp_err_t read = internal_flash_log.read(
          flash_dump.next_offset, data.data.data(), requested, read_size);
      if (read != ESP_OK || read_size == 0) {
        const protocol::RecoveryStatusMessage failed{
            protocol::RecoveryOpcode::start_log_dump,
            flash_dump.control.transfer_id,
            protocol::RecoveryStatusCode::io_error,
            protocol::RecoverySource::internal_flash,
            internal_flash_log.size()};
        (void)xQueueSend(recovery_status_queue, &failed, 0);
        flash_dump.active = false;
      } else if (xQueueSend(recovery_log_data_queue, &data, 0) == pdTRUE) {
        flash_dump.next_offset += static_cast<uint32_t>(read_size);
        flash_dump.remaining -= static_cast<uint32_t>(read_size);
        ++flash_dump.sequence;
        if (flash_dump.remaining == 0) {
          const protocol::RecoveryStatusMessage complete{
              protocol::RecoveryOpcode::start_log_dump,
              flash_dump.control.transfer_id,
              protocol::RecoveryStatusCode::complete,
              protocol::RecoverySource::internal_flash,
              internal_flash_log.size()};
          (void)xQueueSend(recovery_status_queue, &complete, 0);
          flash_dump.active = false;
        }
      }
    }

    if (flash_flush_pending &&
        uxQueueMessagesWaiting(flash_log_queue) == 0) {
      persistence_flushed.store(true, std::memory_order_release);
      flash_flush_pending = false;
    }
    if (enter_waiting &&
        recovery_power_safe.load(std::memory_order_acquire) &&
        recovery_motor_safe.load(std::memory_order_acquire) &&
        recovery_sd_flushed.load(std::memory_order_acquire) &&
        persistence_flushed.load(std::memory_order_acquire)) {
      if (recovery_boot::commitRecoveryEntryMarker() &&
          recovery.markResourcesSafeAndFlushed()) {
        recovery_mode_reason.store(static_cast<uint8_t>(enter_reason),
                                   std::memory_order_release);
        recovery_mode_command_sent.store(false, std::memory_order_release);
        recovery_mode_command_pending.store(true, std::memory_order_release);
        recovery_entry_deadline_us =
            static_cast<uint64_t>(esp_timer_get_time()) + 1'000'000;
        enter_waiting = false;
      }
    }
    const bool recovery_only =
        recovery_only_mode.load(std::memory_order_acquire);
    const bool wake_window_elapsed =
        recovery_only && static_cast<uint64_t>(esp_timer_get_time()) >=
                             recovery_wake_deadline_us;
    const bool entry_window_elapsed =
        !recovery_only && recovery_entry_deadline_us != 0 &&
        static_cast<uint64_t>(esp_timer_get_time()) >=
            recovery_entry_deadline_us;
    if (recovery.mayEnterDeepSleep() &&
        (wake_window_elapsed ||
         (!recovery_only &&
          (recovery_mode_command_sent.load(std::memory_order_acquire) ||
           entry_window_elapsed)))) {
      resetWatchdog();
      recovery_boot::enterPeriodicDeepSleep();
    }
    resetWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void sdLogTask(void *) {
  addWatchdog();
  const bool recovery_boot_mode =
      recovery_only_mode.load(std::memory_order_acquire);
  const esp_err_t prepare = recovery_boot_mode ? sd_flight_log.openExisting()
                                                : sd_flight_log.prepareForFlight();
  sd_log_ready.store(prepare == ESP_OK, std::memory_order_release);
  if (prepare != ESP_OK) {
    sd_log_failed.store(true, std::memory_order_release);
    enqueueEvent(protocol::eventFlag(protocol::MissionEventFlag::mission_sd_error),
                 protocol::MissionState::command_receive, 0,
                 static_cast<uint16_t>(static_cast<uint32_t>(prepare) & 0xFFFFU));
  }

  RecoveryDumpCursor sd_dump{};
  struct StorageExportCompletion {
    bool pending{};
    uint8_t transaction_id{};
    protocol::CommandReason reason{protocol::CommandReason::none};
    uint32_t detail{};
  } export_completion;
  for (;;) {
    if (export_completion.pending &&
        xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      const auto result = command_executor.finish(
          export_completion.transaction_id,
          export_completion.reason == protocol::CommandReason::none
              ? protocol::CommandPhase::completed
              : protocol::CommandPhase::failed,
          export_completion.reason, export_completion.detail);
      xSemaphoreGive(executor_mutex);
      enqueueResult(result, false);
      export_completion = {};
    }

    if (!export_completion.pending) {
      StorageExportRequest export_request{};
      if (xQueueReceive(storage_export_queue, &export_request, 0) == pdTRUE) {
        protocol::CommandReason reason = protocol::CommandReason::none;
        esp_err_t export_result = ESP_ERR_INVALID_STATE;
        if (!sd_flight_log.ready() || !internal_flash_log.ready()) {
          reason = protocol::CommandReason::device_unavailable;
        } else {
          export_result =
              sd_flight_log.exportRawFlashAndErase(internal_flash_log);
          if (export_result != ESP_OK)
            reason = protocol::CommandReason::persistence_error;
        }
        if (reason == protocol::CommandReason::none)
          flash_log_ready.store(true, std::memory_order_release);
        export_completion =
            {true, export_request.transaction_id, reason,
             static_cast<uint32_t>(export_result)};
      }
    }

    flight_log::SerializedRecord record{};
    while (xQueueReceive(sd_log_queue, &record, 0) == pdTRUE) {
      if (sd_flight_log.append(record) != ESP_OK) {
        sd_log_drop_count.fetch_add(1, std::memory_order_relaxed);
        if (!sd_log_failed.exchange(true, std::memory_order_acq_rel))
          enqueueEvent(protocol::eventFlag(
                           protocol::MissionEventFlag::mission_sd_error),
                       protocol::MissionState::unknown, 0, 1);
      }
    }

    PersistenceSignal signal{};
    if (xQueueReceive(persistence_queue, &signal, 0) == pdTRUE) {
      const esp_err_t flushed = sd_flight_log.ready() ? sd_flight_log.flush()
                                                       : ESP_ERR_INVALID_STATE;
      if (flushed != ESP_OK &&
          !sd_log_failed.exchange(true, std::memory_order_acq_rel))
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::mission_sd_error),
                     protocol::MissionState::unknown, 0, 2);
      recovery_sd_flushed.store(true, std::memory_order_release);
    }

    protocol::RecoveryControl control{};
    while (xQueueReceive(sd_recovery_queue, &control, 0) == pdTRUE) {
      protocol::RecoveryStatusCode status =
          protocol::RecoveryStatusCode::invalid_state;
      if (!recovery_only_mode.load(std::memory_order_acquire)) {
        status = protocol::RecoveryStatusCode::invalid_state;
      } else if (control.opcode == protocol::RecoveryOpcode::start_log_dump) {
        const uint32_t total_size = sd_flight_log.size();
        if (!sd_flight_log.ready()) {
          status = protocol::RecoveryStatusCode::source_unavailable;
        } else if (sd_dump.active) {
          status = protocol::RecoveryStatusCode::busy;
        } else if (control.offset > total_size) {
          status = protocol::RecoveryStatusCode::invalid_argument;
        } else {
          const uint32_t available = total_size - control.offset;
          const uint32_t requested =
              control.length == 0 ? available
                                  : std::min(control.length, available);
          sd_dump = {requested != 0, control, control.offset, requested, 0};
          status = requested == 0 ? protocol::RecoveryStatusCode::complete
                                  : protocol::RecoveryStatusCode::dumping;
        }
      } else if (control.opcode == protocol::RecoveryOpcode::stop_log_dump) {
        if (sd_dump.active && sd_dump.control.transfer_id == control.transfer_id) {
          sd_dump.active = false;
          status = protocol::RecoveryStatusCode::aborted;
        } else {
          status = protocol::RecoveryStatusCode::invalid_state;
        }
      } else if (control.opcode == protocol::RecoveryOpcode::wake) {
        status = protocol::RecoveryStatusCode::ready;
      }
      const protocol::RecoveryStatusMessage response{
          control.opcode, control.transfer_id, status, control.source,
          sd_flight_log.size()};
      (void)xQueueSend(recovery_status_queue, &response, 0);
    }

    if (sd_dump.active) {
      protocol::RecoveryLogData data{};
      data.transfer_id = sd_dump.control.transfer_id;
      data.sequence = sd_dump.sequence;
      const std::size_t requested =
          std::min<std::size_t>(data.data.size(), sd_dump.remaining);
      std::size_t read_size = 0;
      const esp_err_t read = sd_flight_log.read(
          sd_dump.next_offset, data.data.data(), requested, read_size);
      if (read != ESP_OK || read_size == 0) {
        const protocol::RecoveryStatusMessage failed{
            protocol::RecoveryOpcode::start_log_dump,
            sd_dump.control.transfer_id,
            protocol::RecoveryStatusCode::io_error,
            protocol::RecoverySource::mission_sd_latest_flight,
            sd_flight_log.size()};
        (void)xQueueSend(recovery_status_queue, &failed, 0);
        sd_dump.active = false;
      } else if (xQueueSend(recovery_log_data_queue, &data, 0) == pdTRUE) {
        sd_dump.next_offset += static_cast<uint32_t>(read_size);
        sd_dump.remaining -= static_cast<uint32_t>(read_size);
        ++sd_dump.sequence;
        if (sd_dump.remaining == 0) {
          const protocol::RecoveryStatusMessage complete{
              protocol::RecoveryOpcode::start_log_dump,
              sd_dump.control.transfer_id,
              protocol::RecoveryStatusCode::complete,
              protocol::RecoverySource::mission_sd_latest_flight,
              sd_flight_log.size()};
          (void)xQueueSend(recovery_status_queue, &complete, 0);
          sd_dump.active = false;
        }
      }
    }
    resetWatchdog();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

TaskHandle_t createTask(std::size_t index, TaskFunction_t function) {
  const auto &descriptor = kTaskArchitecture[index];
  return xTaskCreateStatic(function, descriptor.name, kTaskStackWords, nullptr,
                           descriptor.priority, task_stacks[index].data(),
                           &task_controls[index]);
}

} // 無名名前空間

esp_err_t ProductionRuntime::start() {
  bool expected = false;
  if (!runtime_started.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;
  status_queue = xQueueCreateStatic(1, sizeof(RuntimeStatus),
                                    status_queue_buffer.data(),
                                    &status_queue_storage);
  power_queue = xQueueCreateStatic(4, sizeof(PowerRequest),
                                   power_queue_buffer.data(),
                                   &power_queue_storage);
  command_queue = xQueueCreateStatic(32, sizeof(MissionCommandEnvelope),
                                     command_queue_buffer.data(),
                                     &command_queue_storage);
  transition_queue = xQueueCreateStatic(8, sizeof(MissionCommandEnvelope),
                                        transition_queue_buffer.data(),
                                        &transition_queue_storage);
  emergency_queue = xQueueCreateStatic(16, sizeof(EmergencyEnvelope),
                                       emergency_queue_buffer.data(),
                                       &emergency_queue_storage);
  result_queue = xQueueCreateStatic(32, sizeof(protocol::CommandResult),
                                    result_queue_buffer.data(),
                                    &result_queue_storage);
  para_queue = xQueueCreateStatic(4, sizeof(ParaRequest),
                                  para_queue_buffer.data(),
                                  &para_queue_storage);
  parachute_command_queue = xQueueCreateStatic(
      8, sizeof(ParachuteCommandRequest),
      parachute_command_queue_buffer.data(),
      &parachute_command_queue_storage);
  parachute_start_response_queue = xQueueCreateStatic(
      2, sizeof(ParachuteStartResponse),
      parachute_start_response_queue_buffer.data(),
      &parachute_start_response_queue_storage);
  parachute_persistence_request_queue = xQueueCreateStatic(
      4, sizeof(ParachutePersistenceRequest),
      parachute_persistence_request_queue_buffer.data(),
      &parachute_persistence_request_queue_storage);
  parachute_persistence_response_queue = xQueueCreateStatic(
      4, sizeof(ParachutePersistenceResponse),
      parachute_persistence_response_queue_buffer.data(),
      &parachute_persistence_response_queue_storage);
  air_data_queue = xQueueCreateStatic(1, sizeof(AirDataSnapshot),
                                      air_data_queue_buffer.data(),
                                      &air_data_queue_storage);
  recovery_queue = xQueueCreateStatic(4, sizeof(RecoveryRequest),
                                      recovery_queue_buffer.data(),
                                      &recovery_queue_storage);
  recovery_status_queue = xQueueCreateStatic(
      4, sizeof(protocol::RecoveryStatusMessage),
      recovery_status_queue_buffer.data(), &recovery_status_queue_storage);
  recovery_log_data_queue = xQueueCreateStatic(
      16, sizeof(protocol::RecoveryLogData), recovery_log_data_queue_buffer.data(),
      &recovery_log_data_queue_storage);
  sd_recovery_queue = xQueueCreateStatic(
      4, sizeof(protocol::RecoveryControl), sd_recovery_queue_buffer.data(),
      &sd_recovery_queue_storage);
  storage_export_queue = xQueueCreateStatic(
      2, sizeof(StorageExportRequest), storage_export_queue_buffer.data(),
      &storage_export_queue_storage);
  flash_log_queue = xQueueCreateStatic(
      32, sizeof(flight_log::SerializedRecord), flash_log_queue_buffer.data(),
      &flash_log_queue_storage);
  sd_log_queue = xQueueCreateStatic(
      64, sizeof(flight_log::SerializedRecord), sd_log_queue_buffer.data(),
      &sd_log_queue_storage);
  event_queue = xQueueCreateStatic(16, sizeof(EventRequest),
                                   event_queue_buffer.data(),
                                   &event_queue_storage);
  persistence_queue = xQueueCreateStatic(
      1, sizeof(PersistenceSignal), persistence_queue_buffer.data(),
      &persistence_queue_storage);
  state_mutex = xSemaphoreCreateMutexStatic(&state_mutex_storage);
  executor_mutex = xSemaphoreCreateMutexStatic(&executor_mutex_storage);
  time_mutex = xSemaphoreCreateMutexStatic(&time_mutex_storage);
  if (status_queue == nullptr || power_queue == nullptr ||
      command_queue == nullptr || emergency_queue == nullptr ||
      transition_queue == nullptr || result_queue == nullptr ||
      para_queue == nullptr || parachute_command_queue == nullptr ||
      parachute_start_response_queue == nullptr ||
      parachute_persistence_request_queue == nullptr ||
      parachute_persistence_response_queue == nullptr ||
      air_data_queue == nullptr || recovery_queue == nullptr ||
      recovery_status_queue == nullptr || recovery_log_data_queue == nullptr ||
      sd_recovery_queue == nullptr || storage_export_queue == nullptr ||
      flash_log_queue == nullptr || sd_log_queue == nullptr ||
      event_queue == nullptr || persistence_queue == nullptr ||
      state_mutex == nullptr || executor_mutex == nullptr ||
      time_mutex == nullptr) {
    failSafeOutputs();
    return ESP_ERR_NO_MEM;
  }

  RuntimeStatus initial{};
  if (recovery_only_) {
    initial.state = protocol::MissionState::descent;
    initial.flight_status = (1U << 14U) | (1U << 13U);
    initial.fin_mode = protocol::FinMode::brake;
    initial.para_mode = protocol::ParaMode::powered_off;
    initial.deployment_power_cutoff = true;
  }
  (void)xQueueOverwrite(status_queue, &initial);
  recovery_only_mode.store(recovery_only_, std::memory_order_release);
  recovery_wake_valid.store(recovery_wake_valid_, std::memory_order_release);
  if (!recovery_only_) {
    recovery_boot::StartReadinessAudit readiness_audit{};
    if (recovery_boot::loadStartReadinessAudit(readiness_audit)) {
      std::printf(
          "RTC preflight audit restored: forced=%u generation=%lu captured_us=%llu ready=0x%02X missing=0x%02X\n",
          readiness_audit.forced ? 1U : 0U,
          static_cast<unsigned long>(readiness_audit.generation),
          static_cast<unsigned long long>(readiness_audit.captured_at_us),
          static_cast<unsigned>(readiness_audit.ready_mask),
          static_cast<unsigned>(readiness_audit.missing_mask));
    }
    mission::ResetCheckpoint checkpoint{};
    if (recovery_boot::loadFlightCheckpoint(checkpoint)) {
      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        (void)state_machine.restoreAfterReset(
            static_cast<uint64_t>(esp_timer_get_time()), checkpoint);
        xSemaphoreGive(state_mutex);
      }
    }
  }
  flight_enabled_ = flight_config::productionFlightConfigurationReady();

  const std::array<TaskFunction_t, 9> functions{
      safetyTask,         parachuteTask, missionRealtimeTask,
      airDataTask,        canTask,       commandWorkerTask,
      housekeepingTask,   internalFlashTask, sdLogTask,
  };
  for (std::size_t index = 0; index < functions.size(); ++index) {
    const bool recovery_task = index == 0 || index == 4 || index == 6 ||
                               index == 7 || index == 8;
    if (recovery_only_ && !recovery_task)
      continue;
    if (createTask(index, functions[index]) == nullptr) {
      failSafeOutputs();
      return ESP_ERR_NO_MEM;
    }
  }
  std::printf("production runtime: flight_enabled=%s\n",
              flight_enabled_ ? "true" : "false");
  if (!flight_enabled_)
    std::printf("飛行設定が不完全なためSequence Startを拒否します。\n");
  else
    std::printf("暫定flight設定を有効化しました。TODO(HW_TEST/SIMULATION)の値を含み、flight-qualifiedではありません。\n");
  return ESP_OK;
}

} // 名前空間 runtime
