from pathlib import Path

path = Path("src/runtime/production_runtime.cpp")
s = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global s
    count = s.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected 1 occurrence, got {count}")
    s = s.replace(old, new, 1)


# command終端通知とStart preparation応答はmutex/queue競合で捨てず、次loopで再試行する。
replace_once(
    """  struct MotionOperation {
    bool active{};
    bool automatic{};
    bool pending_start{};
    uint8_t transaction_id{};
    uint32_t flight_epoch{};
    uint64_t started_at_us{};
    float delta_degrees{};
  } operation;
""",
    """  struct MotionOperation {
    bool active{};
    bool automatic{};
    bool pending_start{};
    uint8_t transaction_id{};
    uint32_t flight_epoch{};
    uint64_t started_at_us{};
    float delta_degrees{};
  } operation;

  struct CommandCompletion {
    bool pending{};
    uint8_t transaction_id{};
    protocol::CommandReason reason{protocol::CommandReason::none};
    uint32_t detail{};
  } completion;
  bool start_response_pending = false;
  ParachuteStartResponse pending_start_response{};
""",
    "completion state",
)

# persisted phase設定に依存しないようDirectionをnormalへ固定する。
replace_once(
    """      const esp_err_t configured = servo.configureStepMode(
          STS3215::Persistence::volatile_only);
      if (configured != ESP_OK)
        return false;
      step_mode_ready =
          servo.verifyOperatingMode(STS3215::OperatingMode::step) == ESP_OK;
""",
    """      const esp_err_t configured = servo.configureStepMode(
          STS3215::Persistence::volatile_only);
      if (configured != ESP_OK)
        return false;
      // 過去のEEPROM phase設定で相対moveの符号が反転しないよう、
      // flight中だけDirection::normalへ固定する。
      const esp_err_t direction = servo.setDirection(
          STS3215::Direction::normal,
          STS3215::Persistence::volatile_only);
      if (direction != ESP_OK)
        return false;
      step_mode_ready =
          servo.verifyOperatingMode(STS3215::OperatingMode::step) == ESP_OK;
""",
    "step mode direction",
)

# transport fault時は物理電源を落とさずbus/servoだけ再接続する。
replace_once(
    """  auto resetTransport = [&]() {
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
""",
    """  auto resetTransport = [&](uint64_t now_us, bool power_lost) {
    if (servo.initialized())
      (void)servo.end();
    if (bus.initialized())
      (void)bus.end();
    sts_ready.store(false, std::memory_order_release);
    step_mode_ready = false;
    hold_established = false;
    if (power_lost) {
      power_enabled = false;
      power_enabled_at_us = 0;
      next_initialization_attempt_us = 0;
      return;
    }
    power_enabled =
        parachute_power_applied.load(std::memory_order_acquire);
    if (power_enabled) {
      // 電源rail自体は落としていないためstabilization待ちはやり直さず、
      // state別reconnect intervalだけ待つ。
      power_enabled_at_us = now_us;
      next_initialization_attempt_us =
          now_us + static_cast<uint64_t>(reconnectIntervalMs()) * 1'000ULL;
    } else {
      power_enabled_at_us = 0;
      next_initialization_attempt_us = 0;
    }
  };
""",
    "reset transport",
)
s = s.replace("      resetTransport();\n", "      resetTransport(now_us, true);\n")

# terminal resultはexecutor mutexが取れない1回だけで消失させない。
replace_once(
    """  auto finishGeneric = [&](uint8_t transaction_id,
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
""",
    """  auto queueGenericCompletion = [&](uint8_t transaction_id,
                                    protocol::CommandReason reason,
                                    uint32_t detail = 0) {
    if (transaction_id == 0)
      return;
    completion = {true, transaction_id, reason, detail};
  };

  auto flushGenericCompletion = [&]() {
    if (!completion.pending)
      return;
    if (xSemaphoreTake(executor_mutex, 0) != pdTRUE)
      return;
    const auto result = command_executor.finish(
        completion.transaction_id,
        completion.reason == protocol::CommandReason::none
            ? protocol::CommandPhase::completed
            : protocol::CommandPhase::failed,
        completion.reason, completion.detail);
    xSemaphoreGive(executor_mutex);
    completion = {};
    // Emergencyが先にtransactionを終端した場合は二重resultを送らない。
    if (result.command != 0)
      enqueueResult(result, false);
  };
""",
    "generic completion",
)
s = s.replace("finishGeneric(", "queueGenericCompletion(")

# move command失敗後は同じ相対moveを再送せずtransportだけ再接続する。
replace_once(
    """      (void)establishHold();
      return false;
    }
    operation = {true, automatic, false, transaction_id, flight_epoch,
""",
    """      resetTransport(now_us, false);
      return false;
    }
    operation = {true, automatic, false, transaction_id, flight_epoch,
""",
    "move failure reconnect",
)

# Closeは既存wire ParaMode::closingを使用し、Openだけopening_or_retryingとする。
replace_once(
    """    para_mode_actual.store(protocol::ParaMode::opening_or_retrying,
                           std::memory_order_release);
    return true;
  };

  for (;;) {
""",
    """    const auto movement_mode =
        delta_degrees == actuators::kParachuteCloseRelativeDegrees
            ? protocol::ParaMode::closing
            : protocol::ParaMode::opening_or_retrying;
    para_mode_actual.store(movement_mode, std::memory_order_release);
    return true;
  };

  for (;;) {
""",
    "para movement mode",
)

# loop先頭で保留中のterminal result / start responseを再送する。
replace_once(
    """  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

    ParaRequest request{};
""",
    """  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    flushGenericCompletion();
    if (start_response_pending &&
        xQueueSend(parachute_start_response_queue, &pending_start_response, 0) ==
            pdTRUE)
      start_response_pending = false;

    ParaRequest request{};
""",
    "loop completion flush",
)

# +25sでまだOpenを発行できていなかった/動作中だった場合もfailureだけ記録し、cutoffを止めない。
replace_once(
    """      if (request.kind == ParaRequest::Kind::power_off) {
        operation = {};
        desired = DesiredState::powered_off;
""",
    """      if (request.kind == ParaRequest::Kind::power_off) {
        if (request.safety_authorized) {
          if (operation.active && operation.automatic)
            latchDeploymentFailure(
                ParachuteDeploymentFailure::motion_timeout);
          else if (automatic_open_epoch != 0 &&
                   opened_epoch != automatic_open_epoch)
            latchDeploymentFailure(
                ParachuteDeploymentFailure::move_command_failed);
        }
        operation = {};
        completion = {};
        desired = DesiredState::powered_off;
""",
    "cutoff failure latch",
)

# Emergency Holdは実行中operationだけでなく未送信terminal completionも破棄する。
replace_once(
    """      if (request.kind == ParaRequest::Kind::hold) {
        operation = {};
        desired = DesiredState::holding;
""",
    """      if (request.kind == ParaRequest::Kind::hold) {
        operation = {};
        completion = {};
        desired = DesiredState::holding;
""",
    "emergency hold completion",
)

# pending terminal/start responseがある間は次のparachute generic requestを消費しない。
replace_once(
    """    if (!operation.active && !operation.pending_start) {
      ParachuteCommandRequest command_request{};
""",
    """    if (!operation.active && !operation.pending_start &&
        !completion.pending && !start_response_pending) {
      ParachuteCommandRequest command_request{};
""",
    "command receive gating",
)

# Start preparation responseもqueue競合時に再試行する。
replace_once(
    """        if (command_request.kind ==
            ParachuteCommandRequest::Kind::start_preparation) {
          const ParachuteStartResponse response{
              command_request.command.transaction_id,
              protocol::CommandReason::none, 0};
          (void)xQueueSend(parachute_start_response_queue, &response, 0);
        } else {
""",
    """        if (command_request.kind ==
            ParachuteCommandRequest::Kind::start_preparation) {
          pending_start_response = {
              command_request.command.transaction_id,
              protocol::CommandReason::none, 0};
          start_response_pending = true;
        } else {
""",
    "start response persistence",
)

# Hold確立失敗またはtelemetry transport faultは相対moveを再送せず再接続へ回す。
replace_once(
    """        } else if (!operation.active && !hold_established) {
          (void)establishHold();
        }
""",
    """        } else if (!operation.active && !hold_established) {
          if (establishHold() != ESP_OK)
            resetTransport(now_us, false);
        }
""",
    "steady hold reconnect",
)

# move完了/timeout時にHoldできなければtransportをinvalid化する。
s = s.replace(
    """            const esp_err_t held = establishHold();
            if (held != ESP_OK) {
""",
    """            const esp_err_t held = establishHold();
            if (held != ESP_OK) {
              resetTransport(now_us, false);
""",
    1,
)
s = s.replace(
    """            const esp_err_t held = establishHold();
            if (automatic)
""",
    """            const esp_err_t held = establishHold();
            if (held != ESP_OK)
              resetTransport(now_us, false);
            if (automatic)
""",
    1,
)

replace_once(
    """        if (now_us - last_telemetry_read_us >= kTelemetryIntervalUs) {
          last_telemetry_read_us = now_us;
          (void)updateTelemetry();
        }
""",
    """        if (now_us - last_telemetry_read_us >= kTelemetryIntervalUs) {
          last_telemetry_read_us = now_us;
          if (updateTelemetry() != ESP_OK && !operation.active)
            resetTransport(now_us, false);
        }
""",
    "telemetry reconnect",
)

for required in [
    "servo.setDirection(",
    "servo.moveRelativeDegrees(delta_degrees, motion())",
    "queueGenericCompletion",
    "flushGenericCompletion",
    "start_response_pending",
    "resetTransport(now_us, false)",
    "protocol::ParaMode::closing",
]:
    if required not in s:
        raise RuntimeError(f"required hardening missing: {required}")

path.write_text(s)
