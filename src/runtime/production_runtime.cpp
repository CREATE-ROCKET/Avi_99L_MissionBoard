#include "runtime/production_runtime.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "CANCREATE.h"
#include "I2CCREATE.h"
#include "LPS25HB.h"
#include "SSCDRRN005PD2A5.h"
#include "STS3215.h"
#include "STSCREATE.h"
#include "avi_esp_libs/timeout.h"
#include "bringup/encoder_bringup.hpp"
#include "bringup/imu_bringup.hpp"
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
#include "sensors/air_data_flight_logic.hpp"
#include "sensors/airspeed_estimator.hpp"
#include "sensors/as5047d_health.hpp"
#include "sensors/attitude_estimator.hpp"
#include "sensors/flight_detectors.hpp"
#include "sensors/sensor_health.hpp"

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
  protocol::RecoveryControl control{};
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
  persistence_corrupt = 7,
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
QueueHandle_t event_queue{};
QueueHandle_t persistence_queue{};
std::atomic<uint32_t> result_queue_overflow{};
std::atomic<uint32_t> emergency_metadata_overflow{};
std::atomic<uint16_t> event_overflow_latch{};
std::atomic<bool> runtime_started{false};
std::atomic<bool> imu_ready{false};
std::atomic<bool> encoder_ready{false};
std::atomic<bool> motor_ready{false};
std::atomic<bool> fin_zero_configured{false};
std::atomic<bool> lps_ready{false};
std::atomic<bool> ssc_ready{false};
std::atomic<bool> sts_ready{false};
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
std::atomic<bool> recovery_status_sent{false};
std::atomic<uint32_t> pressure_deployment_epoch{};
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

void failSafeOutputs() {
  (void)bringup::safe_outputs::motorCoast();
  (void)bringup::safe_outputs::setAux5v(false);
  (void)bringup::safe_outputs::setParaPower(false);
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
  if (xQueueSend(event_queue, &event, 0) != pdTRUE)
    event_overflow_latch.fetch_or(flags, std::memory_order_relaxed);
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

bool estimatePreflightGyroBias(const sensors::GyroHistoryRing &history,
                               uint64_t liftoff_time_us, double &bias) {
  constexpr uint64_t kBiasWindowUs = 1'000'000;
  const uint64_t first_time = liftoff_time_us > kBiasWindowUs
                                  ? liftoff_time_us - kBiasWindowUs
                                  : 0;
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t index = 0; index < history.size(); ++index) {
    const auto &sample = history.at(index);
    if (sample.timestamp_us < first_time ||
        sample.timestamp_us > liftoff_time_us || !sample.valid ||
        sample.saturated || sample.format_fault || sample.lost_packets != 0)
      continue;
    sum += sample.roll_rate_rad_s;
    ++count;
  }
  // TODO(SIMULATION): 1秒/500 sample条件をpreflight noiseで再評価する。
  if (count < 500)
    return false;
  bias = sum / static_cast<double>(count);
  return true;
}

void safetyTask(void *) {
  addWatchdog();
  bool cutoff_latched = false;
  uint32_t tracked_epoch = 0;
  bool liftoff_valid = false;
  uint64_t liftoff_time_us = 0;
  uint64_t elapsed_offset_us = 0;
  bool deployment_requested = false;
  for (;;) {
    if (emergency_power_safe_requested.exchange(false,
                                                std::memory_order_acq_rel)) {
      // Emergency latchはqueue容量と無関係にSafetyTaskからpowerを落とす。
      (void)bringup::safe_outputs::setAux5v(false);
      (void)bringup::safe_outputs::setParaPower(false);
      sts_ready.store(false, std::memory_order_release);
      para_mode_actual.store(protocol::ParaMode::powered_off,
                             std::memory_order_release);
    }
    if (recovery_requested.load(std::memory_order_acquire)) {
      const bool aux_safe = bringup::safe_outputs::setAux5v(false) == ESP_OK;
      const bool para_safe =
          bringup::safe_outputs::setParaPower(false) == ESP_OK;
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
        (void)bringup::safe_outputs::setParaPower(false);
        sts_ready.store(false, std::memory_order_release);
        para_mode_actual.store(protocol::ParaMode::powered_off,
                               std::memory_order_release);
      } else {
        (void)bringup::safe_outputs::setAux5v(request.auxiliary_5v);
        (void)bringup::safe_outputs::setParaPower(request.parachute_power);
      }
    }
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      const auto snapshot = state_machine.snapshot();
      if (snapshot.flight_epoch != tracked_epoch) {
        tracked_epoch = snapshot.flight_epoch;
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
      (void)bringup::safe_outputs::setParaPower(false);
      sts_ready.store(false, std::memory_order_release);
      para_mode_actual.store(protocol::ParaMode::powered_off,
                             std::memory_order_release);
      const ParaRequest para{ParaRequest::Kind::power_off, tracked_epoch,
                             true};
      (void)xQueueSend(para_queue, &para, 0);
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
  uint64_t open_requested_at_us = 0;
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
  const int16_t target_tolerance_count = static_cast<int16_t>(std::ceil(
      flight_config::kParachute.target_tolerance_deg /
      actuators::kParachuteDegreesPerCount));

  auto queuePower = [](const PowerRequest &request) {
    if (xQueueSend(power_queue, &request, 0) == pdTRUE)
      return true;
    // queue飽和時も電源OFF要求だけはGPIOへ直接反映する。
    if (request.cutoff || !request.parachute_power)
      (void)bringup::safe_outputs::setParaPower(false);
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
    controller.notifyPowerCutoff();
    sts_ready.store(false, std::memory_order_release);
    para_mode_actual.store(final_mode, std::memory_order_release);
    desired = DesiredState::powered_off;
    active_epoch = 0;
    power_enabled_at_us = 0;
    next_initialization_attempt_us = 0;
    open_requested_at_us = 0;
    power_enabled = false;
    mode_prepared = false;
    controller_started = false;
    hold_established = false;
    last_initialization_error = ESP_ERR_INVALID_STATE;
  };

  auto requestPower = [&](uint64_t now_us) {
    if (power_enabled)
      return true;
    const PowerRequest power{true, true, false};
    if (!queuePower(power))
      return false;
    power_enabled = true;
    power_enabled_at_us = now_us;
    next_initialization_attempt_us =
        now_us + static_cast<uint64_t>(
                     flight_config::kParachute.power_stabilization_ms) *
                     1'000ULL;
    return true;
  };

  auto initializeServo = [&](uint64_t now_us) {
    if (!power_enabled ||
        now_us < power_enabled_at_us +
                     static_cast<uint64_t>(
                         flight_config::kParachute.power_stabilization_ms) *
                         1'000ULL ||
        now_us < next_initialization_attempt_us)
      return false;
    next_initialization_attempt_us =
        now_us + static_cast<uint64_t>(
                     flight_config::kParachute.retry_interval_ms) *
                     1'000ULL;

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
    STS3215::RawData data{};
    const esp_err_t result = servo.readRaw(data);
    if (result != ESP_OK)
      return result;
    const auto current =
        actuators::AbsoluteParachuteAngle::fromCount(data.position);
    if (!current.has_value())
      return ESP_ERR_INVALID_RESPONSE;
    angle = *current;
    moving = data.moving;
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
        // Emergency後でもcommit済みならRAMをNVSへ合わせ、Completedは送らない。
        configuration.activatePersistedCandidate(pending.candidate);
        updateConfigurationMirrors();
        if (!pending.interrupted)
          powerOff(false, true, protocol::ParaMode::powered_off);
        requestFinish(protocol::CommandReason::none);
      } else {
        if (!pending.interrupted)
          powerOff(false, true, protocol::ParaMode::powered_off);
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
      // SafetyTaskとFSMからの重複Openでは最初の5秒deadlineを延長しない。
      if (same_epoch && desired == DesiredState::open)
        continue;
      if (!configuration.flightSnapshotValid()) {
        std::printf("parachute open failed: flight snapshot unavailable\n");
        uint8_t expected = 0;
        (void)parachute_deployment_failure.compare_exchange_strong(
            expected, static_cast<uint8_t>(
                          ParachuteDeploymentFailure::open_not_configured));
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
        } else if (!requestPower(pending.started_at_us)) {
          requestFinish(protocol::CommandReason::busy,
                        kDetailQueueUnavailable);
        }
      }
    }

    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (pending.active && pending.stage == OperationStage::initialize) {
      if (initializeServo(now_us)) {
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
            // ForceではSTS失敗をvalidへ偽装せず、snapshotだけはoptionalのままfreezeする。
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
            if (hold == ESP_OK) {
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
                     detail != 0 ? detail : static_cast<uint16_t>(failure));
      }
    };

    // Hold要求中はpower-on/reconnectを継続し、fresh currentを取得できた時点でHoldする。
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
          if (hold_established)
            para_mode_actual.store(protocol::ParaMode::hold,
                                   std::memory_order_release);
          else
            recordParachuteFailure(ParachuteDeploymentFailure::hold_failed);
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

      // 5秒はOpen retryだけを終了する。電源は+25秒cutoffまで維持する。
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
  // FIFO bufferを持つwrapperはtask専有だが、6 KiBのstackには置かない。
  static bringup::ImuBringup imu;
  bringup::EncoderBringup encoder;
  // 1200 sampleのhistoryは約38 KiBあり、6 KiBのtask stackへ置かない。
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
  control::TorqueMapper torque_mapper{board::kFlightMotorA,
                                      board::kFinSoftwareLimits};
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
          (void)motor_driver.coast();
          // ActuatorEmergencyはmotor/ParaをFreeにするcommandであり、
          // 差圧系のGPIO40までpower cycleしない。
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
        // RealtimeTask自身がconsumerなので次tickまでlatchを保持する。
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
          // 最新attemptだけを有効にする。途中失敗時に古い値へrollbackしない。
          preflight_gyro_bias_valid = false;
          gravity_reference_valid = false;
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
        // Cancel後の電源OFFはqueue飽和時も物理出力へ直接反映する。
        (void)bringup::safe_outputs::setParaPower(false);
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
        // FIFO full/data lossをtimestamp epoch破棄と同一扱いせず、読出しを継続する。
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
      // TODO(HW_TEST): sample数・静置判定・gravity norm閾値を実機で確定する。
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
        gravity_reference_valid = std::isfinite(norm) && norm >= 0.8 && norm <= 1.2;
      }
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
      if (imu.initialized() &&
          imu_now_us - last_imu_recovery_attempt_us >= 100'000) {
        last_imu_recovery_attempt_us = imu_now_us;
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
    if (encoder.initialized()) {
      bringup::EncoderSample sample{};
      if (encoder.readPipelined(sample) != ESP_OK || !sample.valid) {
        encoder_ready.store(false, std::memory_order_release);
        fin_angle_available = false;
        fin_rate_valid = false;
        fin_velocity.reset();
      } else {
        constexpr double kPi = 3.141592653589793;
        constexpr double kTwoPi = 6.283185307179586;
        const double wrapped = static_cast<double>(sample.angle_radians);
        if (!fin_angle_available) {
          if (fin_zero_available) {
            // transient後は直前unwrapped角に最も近いturnへ復帰する。
            const double turns =
                std::round((unwrapped_fin_rad - wrapped) / kTwoPi);
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
        if (!fin_zero_available) {
          // TODO(HW_TEST): 起動時の物理直進位置を0 degとみなす暫定zeroを、
          // NVS値または明示calibrationへ置換する。
          fin_zero_reference_rad = unwrapped_fin_rad;
          fin_zero_available = true;
          fin_zero_configured.store(true, std::memory_order_release);
        }
        fin_angle_rad = unwrapped_fin_rad - fin_zero_reference_rad;
        fin_rate_valid = fin_velocity.update(sample.host_timestamp_us,
                                             fin_angle_rad,
                                             fin_rate_rad_s);
        encoder_ready.store(true, std::memory_order_release);
      }
    } else {
      encoder_ready.store(false, std::memory_order_release);
      fin_rate_valid = false;
    }
    const bool fin_sample_valid =
        encoder_ready.load(std::memory_order_acquire) && fin_zero_available &&
        fin_rate_valid && std::isfinite(fin_angle_rad) &&
        std::isfinite(fin_rate_rad_s);
    const bool zero_hold_valid = zero_hold_controller.updateValidity(
        fin_angle_rad, fin_rate_rad_s, fin_sample_valid);
    fin_zero_hold_valid.store(zero_hold_valid, std::memory_order_release);

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
      motor_command = torque_mapper.map(
          request.output_torque_nm, fin_angle_rad, fin_rate_rad_s,
          flight_config::kMotorBusVoltageV);
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
    if (output_inhibited ||
        mission_snapshot.state == protocol::MissionState::command_receive) {
      motor_output_result = motor_driver.coast();
      motor_output_coasting = true;
      if (mission_snapshot.reset_invalidated)
        torque_error = protocol::quantization::TorqueError::reset_invalidated;
    } else if (!motor_ready.load(std::memory_order_acquire) ||
               !motor_driver.initialized()) {
      motor_output_result = motor_driver.coast();
      motor_output_coasting = true;
      torque_error = protocol::quantization::TorqueError::internal_error;
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
        (can_healthy.load(std::memory_order_acquire) ? (1U << 8U) : 0U) |
        (imu_data_loss_latched ? (1U << 9U) : 0U) |
        (!encoder_alive ? (1U << 10U) : 0U) |
        (air_data_error ? (1U << 11U) : 0U) |
        (motor_saturated ? (1U << 12U) : 0U) |
        (status.fin_mode == protocol::FinMode::brake ? (1U << 13U) : 0U) |
        (mission_snapshot.reset_invalidated ? (1U << 14U) : 0U) |
        (mission_snapshot.control_reentry_inhibited ? (1U << 15U) : 0U);
    // config_flagsは暫定flight設定の可視化に使用する。
    // bit0 MotorProfile、bit1 Fin zero、bit2 Para設定、bit3 SSC zero、
    // bit7は未qualificationの暫定値を含むことを示す。
    status.config_flags =
        (board::kFlightMotorA.parameters_valid ? (1U << 0U) : 0U) |
        (fin_zero_configured.load(std::memory_order_acquire) ? (1U << 1U)
                                                             : 0U) |
        (parachute_open_configured.load(std::memory_order_acquire) &&
                 parachute_close_configured.load(std::memory_order_acquire)
             ? (1U << 2U)
             : 0U) |
        (latest_air_data.ssc_zero_valid ? (1U << 3U) : 0U) |
        (1U << 7U);
    (void)xQueueOverwrite(status_queue, &status);

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
  PowerRequest power{true, false, false};
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
    // SSCが未接続でもMission runtimeを停止しない。
    if (bus.probe(0x28) == ESP_OK)
      ssc_result = ssc.begin(bus);
  }
  lps_ready.store(lps_result == ESP_OK, std::memory_order_release);
  ssc_ready.store(ssc_result == ESP_OK, std::memory_order_release);
  std::printf("AirDataTask bus=%s lps=%s ssc=%s%s\n",
              esp_err_to_name(bus_result), esp_err_to_name(lps_result),
              esp_err_to_name(ssc_result),
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

  auto updateMissionSnapshot = [&]() {
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      const auto updated = state_machine.snapshot();
      if (updated.state == protocol::MissionState::command_receive &&
          mission_snapshot.state != protocol::MissionState::command_receive) {
        // benchでsequenceをやり直す場合は、前flightのzeroを流用しない。
        // TODO(HW_TEST): 明示PreflightCalibrationへ移行後はcommand側でresetする。
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
    snapshot.airspeed_raw = static_cast<uint8_t>(
        result == ESP_ERR_TIMEOUT
            ? protocol::quantization::AirspeedError::ssc_i2c_timeout
            : protocol::quantization::AirspeedError::ssc_i2c_error);
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

  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (now_us - last_ssc_us >= 2'500) {
      last_ssc_us = now_us;
      updateMissionSnapshot();
      if (ssc.initialized()) {
        SSCDRRN005PD2A5::Data data{};
        const esp_err_t result = ssc.read(data);
        if (result == ESP_OK) {
          snapshot.ssc_monotonic_us = now_us;
          snapshot.ssc_valid = true;
          snapshot.ssc_temperature_celsius = data.temperature_celsius;
          // TODO(HW_TEST): 静止・無風のCommandReceive起動約1秒をzero取得に
          // 使用する暫定実装を、明示PreflightCalibrationへ置換する。
          (void)pressure_conditioner.updateZero(
              data.differential_pressure_pa,
              mission_snapshot.state ==
                  protocol::MissionState::command_receive);
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
        }
      } else {
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
  protocol::RecoveryStatusMessage pending_recovery_status{};
  bool recovery_status_pending = false;
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
            const RecoveryRequest request{control};
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
              // state mutexの一時競合で安全側commandを取りこぼさない。
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
              // 実state判定はmutex取得をretryするRealtimeTaskへ一元化する。
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
              ESP_OK) {
        if ((pending_recovery_status.opcode ==
                 protocol::RecoveryOpcode::enter_recovery ||
             pending_recovery_status.opcode ==
                 protocol::RecoveryOpcode::wake) &&
            pending_recovery_status.status ==
                protocol::RecoveryStatusCode::ready)
          recovery_status_sent.store(true, std::memory_order_release);
        recovery_status_pending = false;
      }
      EventRequest event{};
      while (xQueueReceive(event_queue, &event, 0) == pdTRUE) {
        pending_event.flags |= event.flags;
        pending_event.state = event.state;
        pending_event.elapsed_raw = event.elapsed_raw;
        pending_event.detail = event.detail;
        event_pending = true;
      }
      const uint16_t overflow_flags =
          event_overflow_latch.exchange(0, std::memory_order_acq_rel);
      if (overflow_flags != 0) {
        pending_event.flags |= overflow_flags;
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
              static_cast<uint8_t>(
                  protocol::quantization::ParachuteAngleError::unavailable)};
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
        telemetry.parachute_angle_raw = 0xFF;
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
        const protocol::PowerTimeTelemetry power{
            sequences.next(protocol::CanId::power_time_telemetry), 0xFF, 0xFF,
            latest.state == protocol::MissionState::descent
                ? protocol::quantization::encodeDescentElapsed(
                      static_cast<double>(latest.flight_elapsed_us) / 1.0e6,
                      protocol::quantization::TimeError::unavailable)
                : static_cast<uint16_t>(
                      0xFFF0U |
                      static_cast<uint8_t>(
                          protocol::quantization::TimeError::pre_liftoff)),
            0xFFF1, persistence_flags};
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
    // CommandWorkerはpolicy/cacheだけを所有し、FSM更新はRealtimeTaskへ委譲する。
    {
      mission::CommandContext context{};
      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        context.state = state_machine.snapshot().state;
        xSemaphoreGive(state_mutex);
      } else {
        context.state = protocol::MissionState::unknown;
      }
      context.resources_preallocated =
          flight_config::nonBypassFlightConfigurationReady();
      context.persistence_load_complete =
          parachute_config_load_complete.load(std::memory_order_acquire);
      context.persistence_runtime_available =
          parachute_persistence_ready.load(std::memory_order_acquire);
      context.fin_available =
          encoder_ready.load(std::memory_order_acquire) &&
          motor_ready.load(std::memory_order_acquire) &&
          fin_zero_configured.load(std::memory_order_acquire);
      // 入口ではSTS接続状態ではなく、owner taskへの経路だけを判定する。
      context.parachute_available = parachute_command_queue != nullptr;
      context.fin_safe_commands_supported = false;
      context.calibration_supported = true;
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
      QueueHandle_t destination = transition_queue;
      ParachuteCommandRequest parachute_command{};
      const bool parachute_domain =
          envelope.decision.domain == mission::CommandDomain::parachute;
      if (parachute_domain) {
        parachute_command = {ParachuteCommandRequest::Kind::generic,
                             envelope.request, {}};
        destination = parachute_command_queue;
      }
      const BaseType_t queued =
          !envelope.decision.execute
              ? pdTRUE
              : (parachute_domain
                     ? xQueueSend(destination, &parachute_command, 0)
                     : xQueueSend(destination, &envelope, 0));
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
  for (;;) {
    // ADC/LEDはownerを確保し、flight設定未確定中は駆動せずhealth周期のみ維持する。
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
  protocol::RecoveryControl enter_request{};
  uint64_t recovery_wake_deadline_us = 0;
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
    // TODO(HW_TEST): CAN command windowを実測後に確定する。
    recovery_wake_deadline_us =
        static_cast<uint64_t>(esp_timer_get_time()) + 2'000'000;
  }
  for (;;) {
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
        // bit15=1はblob corruptionではなくpersistence runtime failureを表す。
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
      protocol::RecoveryStatusCode status =
          protocol::RecoveryStatusCode::invalid_state;
      switch (request.control.opcode) {
      case protocol::RecoveryOpcode::enter_recovery: {
        bool descent = false;
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          descent = state_machine.snapshot().state ==
                    protocol::MissionState::descent;
          xSemaphoreGive(state_mutex);
        }
        if (descent && recovery.requestEntry()) {
          enter_request = request.control;
          recovery_requested.store(true, std::memory_order_release);
          recovery_sd_flushed.store(false, std::memory_order_release);
          persistence_flushed.store(false, std::memory_order_release);
          recovery_status_sent.store(false, std::memory_order_release);
          const PersistenceSignal signal =
              PersistenceSignal::flush_and_safe;
          if (xQueueOverwrite(persistence_queue, &signal) == pdTRUE) {
            enter_waiting = true;
            status = protocol::RecoveryStatusCode::busy;
          } else {
            status = protocol::RecoveryStatusCode::internal_error;
          }
        }
        break;
      }
      case protocol::RecoveryOpcode::wake:
        status = recovery_only_mode.load(std::memory_order_acquire)
                     ? protocol::RecoveryStatusCode::ready
                     : protocol::RecoveryStatusCode::invalid_state;
        break;
      case protocol::RecoveryOpcode::start_log_dump:
        // TEMPORARY_IMPLEMENTATION: backup reader未接続時は明示的に拒否する。
        status = protocol::RecoveryStatusCode::source_unavailable;
        break;
      case protocol::RecoveryOpcode::stop_log_dump:
        status = protocol::RecoveryStatusCode::invalid_state;
        break;
      }
      if (!(request.control.opcode ==
                protocol::RecoveryOpcode::enter_recovery &&
            enter_waiting)) {
        const protocol::RecoveryStatusMessage response{
            request.control.opcode, request.control.transfer_id, status,
            request.control.source, 0};
        (void)xQueueSend(recovery_status_queue, &response, 0);
      }
    }

    if (enter_waiting &&
        recovery_power_safe.load(std::memory_order_acquire) &&
        recovery_motor_safe.load(std::memory_order_acquire) &&
        recovery_sd_flushed.load(std::memory_order_acquire)) {
      // Internal Flash writer未接続のため未flush recordは存在しない。
      persistence_flushed.store(true, std::memory_order_release);
      if (recovery.markResourcesSafeAndFlushed()) {
        const protocol::RecoveryStatusMessage ready{
            protocol::RecoveryOpcode::enter_recovery,
            enter_request.transfer_id,
            protocol::RecoveryStatusCode::ready,
            enter_request.source, 0};
        (void)xQueueSend(recovery_status_queue, &ready, 0);
      }
      enter_waiting = false;
    }
    const bool recovery_only =
        recovery_only_mode.load(std::memory_order_acquire);
    const bool wake_window_elapsed =
        recovery_only && static_cast<uint64_t>(esp_timer_get_time()) >=
                             recovery_wake_deadline_us;
    if (recovery.mayEnterDeepSleep() &&
        (wake_window_elapsed ||
         (!recovery_only &&
          recovery_status_sent.load(std::memory_order_acquire)))) {
      resetWatchdog();
      recovery_boot::enterPeriodicDeepSleep();
    }
    resetWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void sdLogTask(void *) {
  addWatchdog();
  for (;;) {
    PersistenceSignal signal{};
    if (xQueueReceive(persistence_queue, &signal, pdMS_TO_TICKS(20)) ==
        pdTRUE) {
      // TEMPORARY_IMPLEMENTATION: writer未接続のためflush対象recordはない。
      recovery_sd_flushed.store(true, std::memory_order_release);
    }
    resetWatchdog();
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
      air_data_queue == nullptr ||
      recovery_queue == nullptr || recovery_status_queue == nullptr ||
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
