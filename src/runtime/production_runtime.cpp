#include "runtime/production_runtime.hpp"

#include <algorithm>
#include <array>
#include <atomic>
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
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mission/command_executor.hpp"
#include "mission/mission_state.hpp"
#include "mission/recovery.hpp"
#include "actuators/safety_core.hpp"
#include "control/control_pipeline.hpp"
#include "protocol/can_protocol.hpp"
#include "protocol/quantization.hpp"
#include "runtime/task_architecture.hpp"
#include "runtime/recovery_boot.hpp"
#include "runtime/emergency_latch.hpp"
#include "sensors/air_data_flight_logic.hpp"
#include "sensors/attitude_estimator.hpp"
#include "sensors/flight_detectors.hpp"

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
  bool lps_valid{};
  bool ssc_valid{};
};

struct PowerRequest {
  bool auxiliary_5v{};
  bool parachute_power{};
  bool cutoff{};
};

struct MissionCommandEnvelope {
  protocol::GenericCommandRequest request{};
  mission::CommandDecision decision{};
};

struct EmergencyEnvelope {
  uint8_t transaction_id{};
  bool liftoff_detection{};
};

struct ParaRequest {
  enum class Kind : uint8_t { hold, open, power_off } kind{Kind::hold};
  uint32_t flight_epoch{};
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

protocol::CommandReason transitionReason(mission::TransitionResult result);

StaticQueue_t status_queue_storage;
StaticQueue_t power_queue_storage;
StaticQueue_t command_queue_storage;
StaticQueue_t transition_queue_storage;
StaticQueue_t emergency_queue_storage;
StaticQueue_t result_queue_storage;
StaticQueue_t para_queue_storage;
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
std::atomic<bool> lps_ready{false};
std::atomic<bool> ssc_ready{false};
std::atomic<bool> sts_ready{false};
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
  return can.write(output, avi::Timeout::noWait());
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
    }
    if (recovery_requested.load(std::memory_order_acquire)) {
      const bool aux_safe = bringup::safe_outputs::setAux5v(false) == ESP_OK;
      const bool para_safe =
          bringup::safe_outputs::setParaPower(false) == ESP_OK;
      recovery_power_safe.store(aux_safe && para_safe,
                                std::memory_order_release);
    }
    PowerRequest request{};
    while (xQueueReceive(power_queue, &request, 0) == pdTRUE) {
      cutoff_latched = cutoff_latched || request.cutoff;
      if (cutoff_latched) {
        (void)bringup::safe_outputs::setAux5v(false);
        (void)bringup::safe_outputs::setParaPower(false);
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
      const ParaRequest para{ParaRequest::Kind::open, tracked_epoch};
      deployment_requested =
          xQueueSendToFront(para_queue, &para, 0) == pdTRUE;
    }
    if (!cutoff_latched && liftoff_valid && now >= liftoff_time_us &&
        elapsed_us >= 25'000'000) {
      cutoff_latched = true;
      (void)bringup::safe_outputs::setAux5v(false);
      (void)bringup::safe_outputs::setParaPower(false);
      const ParaRequest para{ParaRequest::Kind::power_off, tracked_epoch};
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
  bool power_enabled = false;
  auto powerOff = [&]() {
    if (servo.initialized()) {
      (void)servo.disableTorque();
      (void)servo.end();
    }
    if (bus.initialized())
      (void)bus.end();
    PowerRequest power{false, false, true};
    (void)xQueueSend(power_queue, &power, 0);
    controller.notifyPowerCutoff();
    power_enabled = false;
  };
  for (;;) {
    ParaRequest request{};
    while (xQueueReceive(para_queue, &request, 0) == pdTRUE) {
      if (request.kind == ParaRequest::Kind::power_off) {
        powerOff();
        continue;
      }
      bool current_request = false;
      if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
        const auto snapshot = state_machine.snapshot();
        current_request = request.flight_epoch != 0 &&
                          request.flight_epoch == snapshot.flight_epoch &&
                          (request.kind != ParaRequest::Kind::open ||
                           snapshot.state == protocol::MissionState::descent);
        xSemaphoreGive(state_mutex);
      }
      if (!current_request) {
        powerOff();
      } else if (request.kind == ParaRequest::Kind::hold) {
        // Open/Close設定未復元時はUARTへ駆動commandを送らない。
        if (servo.initialized())
          (void)servo.holdCurrentPosition({STS3215::TorqueLimit::percent(10)});
      } else {
        // TEMPORARY_IMPLEMENTATION: 永続Open位置未復元のためOpenを拒否し電源を遮断する。
        powerOff();
      }
    }
    if (power_enabled && servo.initialized()) {
      STS3215::Data data{};
      const esp_err_t read = servo.read(data);
      sts_ready.store(read == ESP_OK, std::memory_order_release);
      const auto action = controller.tick(
          {static_cast<uint64_t>(esp_timer_get_time()), read == ESP_OK,
           data.position_deg, !data.moving});
      if (action == actuators::ParachuteAction::cut_power)
        powerOff();
    }
    if (!servo.initialized())
      sts_ready.store(false, std::memory_order_release);
    resetWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void missionRealtimeTask(void *) {
  addWatchdog();
  bringup::SpiBringup spi;
  bringup::ImuBringup imu;
  bringup::EncoderBringup encoder;
  sensors::GyroHistoryRing gyro_history;
  sensors::ImuLiftoffDetector liftoff_detector;
  sensors::AttitudeEstimator attitude;
  control::QuadraticN3FinVelocityEstimator fin_velocity;
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
  bool previous_reset_invalidated = false;
  uint64_t last_imu_sample_us = 0;
  uint64_t last_imu_recovery_attempt_us = 0;
  bool timestamp_offset_valid = false;
  uint64_t sensor_to_host_offset_us = 0;
  uint32_t attitude_epoch = 0;
  bool fin_angle_available = false;
  double previous_wrapped_fin_rad = 0.0;
  double unwrapped_fin_rad = 0.0;
  double fin_rate_rad_s = 0.0;
  bool fin_rate_valid = false;
  const esp_err_t spi_result = spi.begin();
  const esp_err_t imu_result =
      spi_result == ESP_OK ? imu.begin(spi, true) : ESP_ERR_INVALID_STATE;
  const esp_err_t encoder_result =
      spi_result == ESP_OK ? encoder.begin(spi) : ESP_ERR_INVALID_STATE;
  const esp_err_t pipeline_result =
      encoder_result == ESP_OK ? encoder.startPipelinedRead()
                               : ESP_ERR_INVALID_STATE;
  imu_ready.store(imu_result == ESP_OK, std::memory_order_release);
  encoder_ready.store(pipeline_result == ESP_OK, std::memory_order_release);
  std::printf("MissionRealtimeTask spi=%s imu=%s encoder=%s pipeline=%s\n",
              esp_err_to_name(spi_result), esp_err_to_name(imu_result),
              esp_err_to_name(encoder_result), esp_err_to_name(pipeline_result));

  uint32_t timestamp_epoch = 1;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, 1);
    if (recovery_requested.load(std::memory_order_acquire)) {
      recovery_motor_safe.store(
          bringup::safe_outputs::motorCoast() == ESP_OK,
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
          (void)bringup::safe_outputs::motorCoast();
          const PowerRequest power{false, false, false};
          (void)xQueueSend(power_queue, &power, 0);
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
      (void)bringup::safe_outputs::motorCoast();
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
      if (code == mission::CommandCode::cancel_sequence)
        transition = state_machine.cancelSequence();
      else if (code == mission::CommandCode::disable_fin_control)
        transition = state_machine.disableFinControl();
      else if (code == mission::CommandCode::start_sequence)
        transition = state_machine.startSequence(
            static_cast<uint64_t>(esp_timer_get_time()), {});
      const auto transition_state = state_machine.snapshot().state;
      xSemaphoreGive(state_mutex);
      const auto reason = transitionReason(transition);
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
    if (current_epoch != detector_epoch ||
        detector_state == protocol::MissionState::command_receive) {
      detector_epoch = current_epoch;
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
            if (gyro.valid && !gyro.format_fault)
              last_imu_sample_us = gyro.timestamp_us;
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
    const bool imu_stale = last_imu_sample_us == 0 ||
                           imu_now_us < last_imu_sample_us ||
                           imu_now_us - last_imu_sample_us > 3'000;
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
          last_imu_sample_us = imu_now_us;
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
        const double wrapped = static_cast<double>(sample.angle_radians);
        if (!fin_angle_available) {
          unwrapped_fin_rad = wrapped;
          fin_angle_available = true;
        } else {
          double delta = wrapped - previous_wrapped_fin_rad;
          if (delta > 3.141592653589793)
            delta -= 6.283185307179586;
          else if (delta < -3.141592653589793)
            delta += 6.283185307179586;
          unwrapped_fin_rad += delta;
        }
        previous_wrapped_fin_rad = wrapped;
        fin_rate_valid = fin_velocity.update(sample.host_timestamp_us,
                                             unwrapped_fin_rad,
                                             fin_rate_rad_s);
        encoder_ready.store(true, std::memory_order_release);
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
    tick.liftoff_detected = liftoff_detected;
    const bool lps_fresh =
        latest_air_data.lps_monotonic_us != 0 &&
        tick.monotonic_us >= latest_air_data.lps_monotonic_us &&
        tick.monotonic_us - latest_air_data.lps_monotonic_us <= 120'000;
    const bool ssc_fresh =
        latest_air_data.ssc_monotonic_us != 0 &&
        tick.monotonic_us >= latest_air_data.ssc_monotonic_us &&
        tick.monotonic_us - latest_air_data.ssc_monotonic_us <= 30'000;
    tick.deployment_pressure_condition =
        lps_fresh &&
        latest_air_data.flight.flight_epoch == current_epoch &&
        latest_air_data.flight.pressure_apex_detected;
    tick.control.fin_control_available =
        encoder_ready.load(std::memory_order_acquire);
    tick.control.fin_zero_hold_valid =
        fin_zero_hold_valid.load(std::memory_order_acquire);
    tick.control.attitude_valid = attitude.state().valid;
    tick.control.lps_available =
        lps_ready.load(std::memory_order_acquire) && lps_fresh &&
        latest_air_data.lps_valid;
    tick.control.ssc_available =
        ssc_ready.load(std::memory_order_acquire) && ssc_fresh &&
        latest_air_data.ssc_valid;
    tick.control.gyro_bias_valid =
        attitude_epoch == current_epoch && attitude.state().valid;
    // TODO(HW_TEST): SSC zero確定まではControl gateをfail-safeで閉じる。
    tick.control.ssc_zero_valid = false;
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
      status.para_mode =
          mission_snapshot.parachute == mission::ParaDirective::powered_off
              ? protocol::ParaMode::powered_off
              : (mission_snapshot.parachute == mission::ParaDirective::open
                     ? protocol::ParaMode::opening_or_retrying
                     : protocol::ParaMode::hold);
      power_cutoff = mission_snapshot.deployment_power_cutoff_latched;
      deployment_started = mission_snapshot.deployment_started;
      flight_epoch = mission_snapshot.flight_epoch;
      xSemaphoreGive(state_mutex);
    }
    if (mission_snapshot.liftoff_time_valid &&
        attitude_epoch != mission_snapshot.flight_epoch) {
      double gyro_bias = 0.0;
      if (estimatePreflightGyroBias(gyro_history,
                                   mission_snapshot.liftoff_time_us,
                                   gyro_bias) &&
          attitude.beginFlight(gyro_history, mission_snapshot.liftoff_time_us,
                               gyro_bias))
        attitude_epoch = mission_snapshot.flight_epoch;
      else
        attitude_epoch = 0;
    }
    if (mission_snapshot.reset_invalidated) {
      attitude.invalidateForReset();
      attitude_epoch = 0;
    }
    const bool encoder_alive = encoder_ready.load(std::memory_order_acquire);
    constexpr double kRadiansToDegrees = 57.29577951308232;
    if (attitude.state().valid) {
      status.roll_raw = protocol::quantization::encodeRoll(
          attitude.state().roll_rad * kRadiansToDegrees,
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
    // Fin zero永続値未復元中はabsolute angleをvalid値として公開しない。
    status.fin_angle_raw = static_cast<uint8_t>(
        encoder_alive
            ? protocol::quantization::FinAngleError::zero_not_configured
            : protocol::quantization::FinAngleError::not_initialized);
    status.fin_rate_raw =
        fin_rate_valid
            ? protocol::quantization::encodeFinRate(
                  fin_rate_rad_s * kRadiansToDegrees,
                  protocol::quantization::FinRateError::estimator_numeric_error)
            : static_cast<uint16_t>(
                  protocol::quantization::FinRateError::estimator_not_ready);
    static bool cutoff_sent = false;
    static bool deployment_sent = false;
    if (power_cutoff && !cutoff_sent) {
      const PowerRequest power{false, false, true};
      const ParaRequest para{ParaRequest::Kind::power_off, flight_epoch};
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
        ssc_fresh && latest_air_data.ssc_valid && status.airspeed_raw <= 245;
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
        (status.fin_mode == protocol::FinMode::brake ? (1U << 13U) : 0U) |
        (mission_snapshot.reset_invalidated ? (1U << 14U) : 0U) |
        (mission_snapshot.control_reentry_inhibited ? (1U << 15U) : 0U);
    status.config_flags = 0;
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
    previous_reset_invalidated = mission_snapshot.reset_invalidated;
    (void)bringup::safe_outputs::motorCoast();
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
  AirDataSnapshot snapshot{};
  mission::MissionSnapshot mission_snapshot{};
  uint64_t last_ssc_us = 0;
  uint64_t last_lps_us = 0;
  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (now_us - last_ssc_us >= 2'500) {
      last_ssc_us = now_us;
      if (ssc.initialized()) {
        SSCDRRN005PD2A5::Data data{};
        const esp_err_t result = ssc.read(data);
        if (result == ESP_OK) {
          snapshot.ssc_monotonic_us = now_us;
          snapshot.ssc_valid = true;
          // TODO(SIMULATION): Saint-Venant係数K=0.92の最終検証前は速度へ変換しない。
          snapshot.airspeed_raw = static_cast<uint8_t>(
              protocol::quantization::AirspeedError::internal_invalid);
        }
      }
      (void)xQueueOverwrite(air_data_queue, &snapshot);
    }

    if (now_us - last_lps_us >= 40'000) {
      last_lps_us = now_us;
      double pressure_hpa = 0.0;
      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        mission_snapshot = state_machine.snapshot();
        xSemaphoreGive(state_mutex);
      }
      esp_err_t read_result = ESP_ERR_INVALID_STATE;
      LPS25HB::Data data{};
      if (lps.initialized())
        read_result = lps.read(data);
      if (read_result == ESP_OK) {
        snapshot.lps_monotonic_us = now_us;
        snapshot.lps_valid = true;
        pressure_hpa = static_cast<double>(data.pressure_pa) / 100.0;
        snapshot.pressure_raw = protocol::quantization::encodeLpsPressure(
            pressure_hpa, protocol::quantization::LpsPressureError::unknown);
        snapshot.temperature_raw =
            protocol::quantization::encodeLpsTemperature(
                data.temperature_celsius,
                protocol::quantization::LpsTemperatureError::unknown);
      }
      if (read_result == ESP_OK) {
        snapshot.flight = flight_logic.update(
            mission_snapshot.flight_epoch, mission_snapshot.state,
            mission_snapshot.elapsed_us, pressure_hpa, true);
        if (snapshot.flight.pressure_apex_detected)
          pressure_deployment_epoch.store(snapshot.flight.flight_epoch,
                                          std::memory_order_release);
      }
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
            protocol::MissionState state = protocol::MissionState::unknown;
            if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
              state = state_machine.snapshot().state;
              xSemaphoreGive(state_mutex);
            }
            if (emergency.transaction_id != 0 &&
                state == protocol::MissionState::command_receive)
              latchPhysicalEmergency(emergency.transaction_id, false);
            else {
              mission::EmergencyDecision result{};
              if (xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                result = command_executor.actuatorEmergency(
                    emergency.transaction_id, state);
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
            protocol::MissionState state = protocol::MissionState::unknown;
            if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
              state = state_machine.snapshot().state;
              xSemaphoreGive(state_mutex);
            }
            if (emergency.transaction_id != 0 &&
                state == protocol::MissionState::engine_burn) {
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
              static_cast<uint16_t>(
                  protocol::quantization::TorqueError::unavailable),
              protocol::quantization::encodeFlightElapsed(
                  static_cast<double>(latest.flight_elapsed_us) / 1.0e6,
                  protocol::quantization::TimeError::unavailable)};
          (void)writeFrame(can, protocol::encode(control));
        }
        if (latest.state == protocol::MissionState::descent) {
          const protocol::DescentCoreTelemetry descent{
              sequences.next(protocol::CanId::descent_core_telemetry),
              0x1FFF, static_cast<uint8_t>(
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
            0xFFF1,
            static_cast<uint8_t>(result_queue_overflow.load(
                                     std::memory_order_relaxed) != 0
                                     ? 0x80
                                     : 0)};
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
      context.sequence_configured = false;
      context.resources_preallocated = true;
      context.fin_available = encoder_ready.load(std::memory_order_acquire);
      context.parachute_available = false;
      context.fin_safe_commands_supported = false;
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
      if (envelope.decision.execute &&
          xQueueSend(transition_queue, &envelope, 0) != pdTRUE) {
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

void internalFlashTask(void *) {
  addWatchdog();
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
      para_queue == nullptr || air_data_queue == nullptr ||
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
    mission::ResetCheckpoint checkpoint{};
    if (recovery_boot::loadFlightCheckpoint(checkpoint)) {
      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        (void)state_machine.restoreAfterReset(
            static_cast<uint64_t>(esp_timer_get_time()), checkpoint);
        xSemaphoreGive(state_mutex);
      }
    }
  }
  flight_enabled_ = board::kFlightMotorA.parameters_valid &&
                    board::kFlightMotorA.polarity !=
                        board::MotorPolarity::unconfigured &&
                    board::kFinSoftwareLimits.configured;
  // Open/Close永続設定のproduction loader未接続なので必ずflight-disabledとする。
  flight_enabled_ = false;

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
    std::printf("飛行設定未確定のためSequence Startを拒否し、actuatorを安全状態に維持します。\n");
  return ESP_OK;
}

} // 名前空間 runtime
