from pathlib import Path
import re

path = Path("src/runtime/production_runtime.cpp")
s = path.read_text()

def sub_once(pattern, repl, text, label, flags=0):
    out, n = re.subn(pattern, repl, text, count=1, flags=flags)
    if n != 1:
        raise RuntimeError(f"{label}: expected 1 replacement, got {n}")
    return out

s = s.replace('#include "nvs.h"\n', '')
s = s.replace('#include "nvs_flash.h"\n', '')

s = sub_once(
    r"struct ParaRequest \{.*?\n\};\n\nstruct ParachuteCommandRequest",
    '''struct ParaRequest {
  enum class Kind : uint8_t {
    open,
    hold,
    power_off,
  } kind{Kind::power_off};
  uint32_t flight_epoch{};
  bool safety_authorized{};
};

struct ParachuteCommandRequest''',
    s, "ParaRequest", re.S)

s = sub_once(
    r"struct ParachutePersistenceRequest \{.*?\n\};\n\nstruct RecoveryRequest",
    "struct RecoveryRequest",
    s, "persistence structs", re.S)

s = sub_once(
    r"enum class ParachuteDeploymentFailure : uint8_t \{.*?\n\};",
    '''enum class ParachuteDeploymentFailure : uint8_t {
  none = 0,
  move_command_failed = 4,
  motion_timeout = 5,
  hold_failed = 6,
};''',
    s, "deployment failure", re.S)

s = re.sub(r"^StaticQueue_t parachute_persistence_[^\n]*\n", "", s, flags=re.M)
s = re.sub(r"^std::array<uint8_t, sizeof\(ParachutePersistenceRequest\) \* 4>\n\s+parachute_persistence_request_queue_buffer\{\};\n", "", s, flags=re.M)
s = re.sub(r"^std::array<uint8_t, sizeof\(ParachutePersistenceResponse\) \* 4>\n\s+parachute_persistence_response_queue_buffer\{\};\n", "", s, flags=re.M)
s = re.sub(r"^QueueHandle_t parachute_persistence_[^\n]*\n", "", s, flags=re.M)

for token in [
    "parachute_config_load_complete",
    "parachute_persistence_ready",
    "parachute_persistence_corrupt",
    "parachute_open_configured",
    "parachute_close_configured",
]:
    s = re.sub(rf"^std::atomic<[^>]+> {token}[^;]*;\n", "", s, flags=re.M)

new_task = r'''void parachuteTask(void *) {
  addWatchdog();
  STSCREATE bus;
  STS3215 servo;

  enum class DesiredState : uint8_t { holding, moving, powered_off };
  DesiredState desired = DesiredState::holding;

  struct MotionOperation {
    bool active{};
    bool automatic{};
    bool pending_start{};
    uint8_t transaction_id{};
    uint32_t flight_epoch{};
    uint64_t started_at_us{};
    float delta_degrees{};
  } operation;

  uint32_t automatic_open_epoch = 0;
  uint32_t opened_epoch = 0;
  uint64_t power_enabled_at_us = 0;
  uint64_t next_power_request_us = 0;
  uint64_t next_initialization_attempt_us = 0;
  uint64_t last_telemetry_read_us = 0;
  bool power_enabled = false;
  bool step_mode_ready = false;
  bool hold_established = false;

  constexpr uint64_t kTelemetryIntervalUs = 500'000;
  constexpr uint64_t kMotionTimeoutUs = 3'000'000;

  auto queuePower = [](const PowerRequest &request) {
    if (xQueueSend(power_queue, &request, 0) == pdTRUE)
      return true;
    if (request.cutoff || !request.parachute_power)
      (void)setTrackedParaPower(false);
    if (request.cutoff || !request.auxiliary_5v)
      (void)bringup::safe_outputs::setAux5v(false);
    return false;
  };

  auto resetTransport = [&]() {
    if (servo.initialized())
      (void)servo.end();
    if (bus.initialized())
      (void)bus.end();
    sts_ready.store(false, std::memory_order_release);
    step_mode_ready = false;
    hold_established = false;
    power_enabled = false;
    power_enabled_at_us = 0;
    next_initialization_attempt_us = 0;
  };

  auto requestPower = [&](uint64_t now_us) {
    if (parachute_power_applied.load(std::memory_order_acquire)) {
      if (!power_enabled) {
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

    if (power_enabled)
      resetTransport();

    if (now_us < next_power_request_us)
      return true;

    const PowerRequest power{true, true, false};
    if (!queuePower(power)) {
      next_power_request_us =
          now_us + static_cast<uint64_t>(
                       flight_config::kParachuteCommandReceiveReconnectMs) *
                       1'000ULL;
      return false;
    }
    next_power_request_us =
        now_us + static_cast<uint64_t>(
                     flight_config::kParachuteCommandReceiveReconnectMs) *
                     1'000ULL;
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
        now_us + static_cast<uint64_t>(
                     flight_config::kParachuteCommandReceiveReconnectMs) *
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
      if (bus.begin(config) != ESP_OK)
        return false;
    }

    if (!servo.initialized() &&
        servo.begin(bus, board::kParaServoId) != ESP_OK)
      return false;
    if (!servo.configurationValid()) {
      (void)servo.end();
      return false;
    }
    if (!step_mode_ready) {
      const esp_err_t configured = servo.configureStepMode(
          STS3215::Persistence::volatile_only);
      if (configured != ESP_OK)
        return false;
      step_mode_ready =
          servo.verifyOperatingMode(STS3215::OperatingMode::step) == ESP_OK;
    }
    sts_ready.store(step_mode_ready, std::memory_order_release);
    return step_mode_ready;
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

  auto establishHold = [&]() {
    if (!servo.initialized() || !step_mode_ready)
      return ESP_ERR_INVALID_STATE;
    STS3215::HoldConfig hold{};
    hold.torque_limit = STS3215::TorqueLimit::percent(
        flight_config::kParachute.torque_limit_percent);
    const esp_err_t result = servo.holdCurrentPosition(hold);
    if (result == ESP_OK) {
      hold_established = true;
      desired = DesiredState::holding;
      para_mode_actual.store(protocol::ParaMode::hold,
                             std::memory_order_release);
    }
    return result;
  };

  auto updateTelemetry = [&]() {
    STS3215::RawData data{};
    const esp_err_t result = servo.readRaw(data);
    if (result != ESP_OK)
      return result;
    const auto current =
        actuators::AbsoluteParachuteAngle::fromCount(data.position);
    if (!current.has_value()) {
      parachute_angle_raw.store(
          static_cast<uint8_t>(
              protocol::quantization::ParachuteAngleError::position_out_of_range),
          std::memory_order_release);
      return ESP_ERR_INVALID_RESPONSE;
    }
    const double degrees =
        static_cast<double>(current->count()) *
        actuators::kParachuteDegreesPerCount;
    parachute_angle_raw.store(
        protocol::quantization::encodeParachuteAngle(
            degrees,
            protocol::quantization::ParachuteAngleError::position_invalid),
        std::memory_order_release);
    return ESP_OK;
  };

  auto finishGeneric = [&](uint8_t transaction_id,
                           protocol::CommandReason reason,
                           uint32_t detail = 0) {
    if (transaction_id == 0)
      return;
    if (xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(2)) != pdTRUE)
      return;
    const auto result = command_executor.finish(
        transaction_id,
        reason == protocol::CommandReason::none
            ? protocol::CommandPhase::completed
            : protocol::CommandPhase::failed,
        reason, detail);
    xSemaphoreGive(executor_mutex);
    if (result.command != 0)
      enqueueResult(result, false);
  };

  auto latchDeploymentFailure = [&](ParachuteDeploymentFailure failure) {
    uint8_t expected = 0;
    if (parachute_deployment_failure.compare_exchange_strong(
            expected, static_cast<uint8_t>(failure))) {
      enqueueEvent(
          protocol::eventFlag(
              protocol::MissionEventFlag::parachute_deployment_failure),
          protocol::MissionState::descent, 0,
          static_cast<uint16_t>(failure));
    }
  };

  auto startMove = [&](float delta_degrees, bool automatic,
                       uint8_t transaction_id, uint32_t flight_epoch,
                       uint64_t now_us) {
    // 相対移動は1 operationにつき1回だけ発行する。retryで再送しない。
    const esp_err_t result =
        servo.moveRelativeDegrees(delta_degrees, motion());
    if (result != ESP_OK) {
      if (automatic)
        latchDeploymentFailure(
            ParachuteDeploymentFailure::move_command_failed);
      else
        finishGeneric(transaction_id,
                      result == ESP_ERR_TIMEOUT
                          ? protocol::CommandReason::timeout
                          : protocol::CommandReason::device_unavailable);
      (void)establishHold();
      return false;
    }
    operation = {true, automatic, false, transaction_id, flight_epoch,
                 now_us, delta_degrees};
    desired = DesiredState::moving;
    hold_established = false;
    para_mode_actual.store(protocol::ParaMode::opening_or_retrying,
                           std::memory_order_release);
    return true;
  };

  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

    ParaRequest request{};
    while (xQueueReceive(para_queue, &request, 0) == pdTRUE) {
      if (request.kind == ParaRequest::Kind::power_off) {
        operation = {};
        desired = DesiredState::powered_off;
        if (servo.initialized())
          (void)servo.end();
        if (bus.initialized())
          (void)bus.end();
        const PowerRequest power{false, false, request.safety_authorized};
        (void)queuePower(power);
        sts_ready.store(false, std::memory_order_release);
        para_mode_actual.store(protocol::ParaMode::powered_off,
                               std::memory_order_release);
        power_enabled = false;
        step_mode_ready = false;
        hold_established = false;
        continue;
      }

      if (request.kind == ParaRequest::Kind::hold) {
        operation = {};
        desired = DesiredState::holding;
        hold_established = false;
        continue;
      }

      bool current_request = request.safety_authorized;
      if (!current_request && xSemaphoreTake(state_mutex, 0) == pdTRUE) {
        const auto snapshot = state_machine.snapshot();
        current_request =
            request.flight_epoch != 0 &&
            request.flight_epoch == snapshot.flight_epoch &&
            snapshot.state == protocol::MissionState::descent;
        xSemaphoreGive(state_mutex);
      }
      if (!current_request)
        continue;
      if (opened_epoch == request.flight_epoch ||
          automatic_open_epoch == request.flight_epoch)
        continue;
      automatic_open_epoch = request.flight_epoch;
      desired = DesiredState::holding;
      hold_established = false;
      (void)requestPower(now_us);
    }

    if (!operation.active && !operation.pending_start) {
      ParachuteCommandRequest command_request{};
      if (xQueueReceive(parachute_command_queue, &command_request, 0) ==
          pdTRUE) {
        if (command_request.kind ==
            ParachuteCommandRequest::Kind::start_preparation) {
          const ParachuteStartResponse response{
              command_request.command.transaction_id,
              protocol::CommandReason::none, 0};
          (void)xQueueSend(parachute_start_response_queue, &response, 0);
        } else {
          const auto code = static_cast<mission::CommandCode>(
              command_request.command.command);
          if (code != mission::CommandCode::para_open &&
              code != mission::CommandCode::para_close) {
            finishGeneric(command_request.command.transaction_id,
                          protocol::CommandReason::not_supported);
          } else {
            const float delta =
                code == mission::CommandCode::para_open
                    ? actuators::kParachuteOpenRelativeDegrees
                    : actuators::kParachuteCloseRelativeDegrees;
            operation = {false, false, true,
                         command_request.command.transaction_id, 0, 0, delta};
            desired = DesiredState::holding;
            hold_established = false;
            (void)requestPower(now_us);
          }
        }
      }
    }

    if (desired != DesiredState::powered_off) {
      (void)requestPower(now_us);
      if (initializeServo(now_us)) {
        if (operation.pending_start) {
          const uint8_t transaction_id = operation.transaction_id;
          const float delta = operation.delta_degrees;
          operation = {};
          (void)startMove(delta, false, transaction_id, 0, now_us);
        } else if (automatic_open_epoch != 0 &&
                   opened_epoch != automatic_open_epoch &&
                   !operation.active) {
          bool should_open = false;
          if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
            const auto snapshot = state_machine.snapshot();
            should_open =
                snapshot.flight_epoch == automatic_open_epoch &&
                snapshot.state == protocol::MissionState::descent &&
                !snapshot.deployment_power_cutoff_latched;
            xSemaphoreGive(state_mutex);
          }
          if (should_open) {
            const uint32_t epoch = automatic_open_epoch;
            // 発行前にepochを消費し、response timeout等でも再発行させない。
            opened_epoch = epoch;
            (void)startMove(
                actuators::kParachuteOpenRelativeDegrees, true, 0, epoch,
                now_us);
          }
        } else if (!operation.active && !hold_established) {
          (void)establishHold();
        }

        if (operation.active) {
          STS3215::RawData data{};
          const esp_err_t read = servo.readRaw(data);
          if (read == ESP_OK && !data.moving) {
            const bool automatic = operation.automatic;
            const uint8_t transaction_id = operation.transaction_id;
            operation = {};
            const esp_err_t held = establishHold();
            if (held != ESP_OK) {
              if (automatic)
                latchDeploymentFailure(
                    ParachuteDeploymentFailure::hold_failed);
              else
                finishGeneric(transaction_id,
                              protocol::CommandReason::device_unavailable);
            } else if (!automatic) {
              finishGeneric(transaction_id,
                            protocol::CommandReason::none);
            }
          } else if (now_us - operation.started_at_us >=
                     kMotionTimeoutUs) {
            const bool automatic = operation.automatic;
            const uint8_t transaction_id = operation.transaction_id;
            operation = {};
            const esp_err_t held = establishHold();
            if (automatic)
              latchDeploymentFailure(
                  held == ESP_OK
                      ? ParachuteDeploymentFailure::motion_timeout
                      : ParachuteDeploymentFailure::hold_failed);
            else
              finishGeneric(transaction_id,
                            protocol::CommandReason::timeout);
          }
        }

        if (now_us - last_telemetry_read_us >= kTelemetryIntervalUs) {
          last_telemetry_read_us = now_us;
          (void)updateTelemetry();
        }
      }
    }

    resetWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void missionRealtimeTask(void *) {'''

s = sub_once(
    r"void parachuteTask\(void \*\) \{.*?\n\}\n\nvoid missionRealtimeTask\(void \*\) \{",
    new_task,
    s, "parachute task", re.S)

s = s.replace("const PowerRequest power{true, false, false};",
              "const PowerRequest power{true, true, false};")
s = s.replace("ParaRequest::Kind::free", "ParaRequest::Kind::hold")
s = s.replace("ParaRequest::Kind::discard_snapshot", "ParaRequest::Kind::hold")

s = re.sub(r"^\s*context\.persistence_load_complete =.*\n", "", s, flags=re.M)
s = re.sub(r"^\s*context\.persistence_runtime_available =.*\n", "", s, flags=re.M)
s = re.sub(r"^\s*readiness\.parachute_open_configured =.*\n", "", s, flags=re.M)
s = re.sub(r"^\s*readiness\.parachute_close_configured =.*\n", "", s, flags=re.M)

m = re.search(r"\nconstexpr char kParachuteNvsNamespace\[\].*?\nvoid internalFlashTask\(void \*\) \{", s, flags=re.S)
if m:
    s = s[:m.start()] + "\nvoid internalFlashTask(void *) {" + s[m.end():]

s = re.sub(r"\n\s*nvs_handle_t parachute_nvs.*?\n\s*for \(;;\) \{",
           "\n  for (;;) {", s, count=1, flags=re.S)
s = re.sub(
    r"\n\s*ParachutePersistenceRequest persistence_request\{\};.*?(?=\n\s*PersistenceSignal)",
    "\n", s, count=1, flags=re.S)

s = re.sub(
    r"\n\s*parachute_persistence_request_queue = xQueueCreateStatic\(.*?;\n",
    "\n", s, count=1, flags=re.S)
s = re.sub(
    r"\n\s*parachute_persistence_response_queue = xQueueCreateStatic\(.*?;\n",
    "\n", s, count=1, flags=re.S)
s = s.replace("      parachute_persistence_request_queue == nullptr ||\n", "")
s = s.replace("      parachute_persistence_response_queue == nullptr ||\n", "")

for token in [
    "parachute_config_load_complete",
    "parachute_persistence_ready",
    "parachute_persistence_corrupt",
    "parachute_open_configured",
    "parachute_close_configured",
]:
    s = re.sub(rf"^\s*if \({token}\.load\(.*?\n(?:\s+.*\n){{0,3}}", "", s,
               flags=re.M)

for forbidden in [
    "ParachutePersistenceRequest",
    "ParachutePersistenceResponse",
    "parachute_persistence_",
    "parachute_config_load_complete",
    "parachute_open_configured",
    "parachute_close_configured",
    "kParachuteNvsNamespace",
    "nvs_open(",
    "nvs_flash_init(",
    "configureMultiTurnPositionMode",
    "servo.disableTorque()",
]:
    if forbidden in s:
        raise RuntimeError(f"forbidden token remains: {forbidden}")

path.write_text(s)
