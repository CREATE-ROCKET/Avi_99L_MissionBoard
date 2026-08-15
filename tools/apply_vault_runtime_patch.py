from pathlib import Path

path = Path("src/runtime/production_runtime.cpp")
text = path.read_text(encoding="utf-8")


def replace_once(old: str, new: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"置換対象が一意ではありません: count={count}\n{old[:160]}")
    text = text.replace(old, new, 1)


replace_once(
    """struct RecoveryRequest {
  protocol::RecoveryControl control{};
};
""",
    """struct RecoveryRequest {
  enum class Kind : uint8_t { control, enter } kind{Kind::control};
  protocol::RecoveryControl control{};
  protocol::RecoveryModeReason reason{
      protocol::RecoveryModeReason::auto_elapsed_120};
};
""",
)

replace_once(
    """  retry_exhausted = 5,
  hold_failed = 6,
  persistence_corrupt = 7,
};
""",
    """  retry_exhausted = 5,
  hold_failed = 6,
};
""",
)

replace_once(
    """std::atomic<uint16_t> event_overflow_latch{};
std::atomic<bool> runtime_started{false};
""",
    """std::atomic<uint16_t> event_overflow_latch{};
std::atomic<uint16_t> parachute_failure_overflow_detail{};
std::atomic<bool> runtime_started{false};
""",
)

replace_once(
    """std::atomic<bool> recovery_sd_flushed{false};
std::atomic<bool> recovery_status_sent{false};
std::atomic<uint32_t> pressure_deployment_epoch{};
""",
    """std::atomic<bool> recovery_sd_flushed{false};
std::atomic<bool> recovery_mode_command_pending{false};
std::atomic<bool> recovery_mode_command_sent{false};
std::atomic<uint8_t> recovery_mode_reason{static_cast<uint8_t>(
    protocol::RecoveryModeReason::auto_elapsed_120)};
std::atomic<uint32_t> pressure_deployment_epoch{};
""",
)

replace_once(
    """void enqueueEvent(uint16_t flags, protocol::MissionState state,
                  uint16_t elapsed_raw = 0, uint16_t detail = 0) {
  const EventRequest event{flags, state, elapsed_raw, detail};
  if (xQueueSend(event_queue, &event, 0) != pdTRUE)
    event_overflow_latch.fetch_or(flags, std::memory_order_relaxed);
}
""",
    """void enqueueEvent(uint16_t flags, protocol::MissionState state,
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
""",
)

replace_once(
    """  uint64_t elapsed_offset_us = 0;
  bool deployment_requested = false;
  for (;;) {
""",
    """  uint64_t elapsed_offset_us = 0;
  bool deployment_requested = false;
  bool recovery_entry_queued = false;
  protocol::MissionState tracked_state = protocol::MissionState::command_receive;
  for (;;) {
""",
)

replace_once(
    """      const auto snapshot = state_machine.snapshot();
      if (snapshot.flight_epoch != tracked_epoch) {
        tracked_epoch = snapshot.flight_epoch;
""",
    """      const auto snapshot = state_machine.snapshot();
      tracked_state = snapshot.state;
      if (snapshot.flight_epoch != tracked_epoch) {
        tracked_epoch = snapshot.flight_epoch;
        recovery_entry_queued = false;
""",
)

replace_once(
    """      const ParaRequest para{ParaRequest::Kind::power_off, tracked_epoch,
                             true};
      (void)xQueueSend(para_queue, &para, 0);
    }
    resetWatchdog();
""",
    """      const ParaRequest para{ParaRequest::Kind::power_off, tracked_epoch,
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
""",
)

replace_once(
    """      if (!configuration.flightSnapshotValid()) {
        std::printf("parachute open failed: flight snapshot unavailable\\n");
        uint8_t expected = 0;
        (void)parachute_deployment_failure.compare_exchange_strong(
            expected, static_cast<uint8_t>(
                          ParachuteDeploymentFailure::open_not_configured));
        desired = DesiredState::holding;
        hold_established = false;
        continue;
      }
""",
    """      if (!configuration.flightSnapshotValid()) {
        std::printf("parachute open failed: flight snapshot unavailable\\n");
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
""",
)

replace_once(
    """                     protocol::MissionState::descent, 0,
                     detail != 0 ? detail : static_cast<uint16_t>(failure));
""",
    """                     protocol::MissionState::descent, 0,
                     static_cast<uint16_t>(failure));
""",
)

replace_once(
    """  control::TorqueMapper torque_mapper{board::kFlightMotorA,
                                      board::kFinSoftwareLimits};
""",
    """  control::TorqueMapper torque_mapper{
      flight_config::kActiveFlightMotorProfile, board::kFinSoftwareLimits};
""",
)

replace_once(
    """    if (current_epoch != detector_epoch) {
      detector_epoch = current_epoch;
      liftoff_detector.reset();
""",
    """    if (current_epoch != detector_epoch) {
      detector_epoch = current_epoch;
      if (current_epoch != 0) {
        parachute_deployment_failure.store(0, std::memory_order_release);
        parachute_failure_overflow_detail.store(0, std::memory_order_release);
      }
      liftoff_detector.reset();
""",
)

replace_once(
    """    status.config_flags =
        (board::kFlightMotorA.parameters_valid ? (1U << 0U) : 0U) |
""",
    """    status.config_flags =
        (flight_config::motorProfileValid() ? (1U << 0U) : 0U) |
""",
)

replace_once(
    """      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        context.state = state_machine.snapshot().state;
        xSemaphoreGive(state_mutex);
      } else {
        context.state = protocol::MissionState::unknown;
      }
""",
    """      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        const auto snapshot = state_machine.snapshot();
        context.state = snapshot.state;
        context.deployment_power_cutoff_done =
            snapshot.deployment_power_cutoff_latched;
        xSemaphoreGive(state_mutex);
      } else {
        context.state = protocol::MissionState::unknown;
        context.deployment_power_cutoff_done = false;
      }
""",
)

replace_once(
    """      } else if (code == mission::CommandCode::disable_fin_control) {
        transition = state_machine.disableFinControl();
      } else if (code == mission::CommandCode::start_sequence ||
""",
    """      } else if (code == mission::CommandCode::disable_fin_control) {
        transition = state_machine.disableFinControl();
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
""",
)

replace_once(
    """            const RecoveryRequest request{control};
            if (xQueueSend(recovery_queue, &request, 0) != pdTRUE) {
""",
    """            RecoveryRequest request{};
            request.kind = RecoveryRequest::Kind::control;
            request.control = control;
            if (xQueueSend(recovery_queue, &request, 0) != pdTRUE) {
""",
)

replace_once(
    """  uint8_t time_request_id = 1;
  protocol::RecoveryStatusMessage pending_recovery_status{};
""",
    """  uint8_t time_request_id = 1;
  uint8_t recovery_mode_sequence = 0;
  protocol::RecoveryStatusMessage pending_recovery_status{};
""",
)

replace_once(
    """      if (recovery_status_pending &&
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
""",
    """      if (recovery_status_pending &&
          writeFrame(can, protocol::encode(pending_recovery_status)) ==
              ESP_OK)
        recovery_status_pending = false;
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
""",
)

replace_once(
    """      EventRequest event{};
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
""",
    """      EventRequest event{};
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
""",
)

replace_once(
    """  mission::RecoveryRuntime recovery;
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
""",
    """  mission::RecoveryRuntime recovery;
  bool enter_waiting = false;
  protocol::RecoveryModeReason enter_reason =
      protocol::RecoveryModeReason::auto_elapsed_120;
  uint64_t recovery_entry_deadline_us = 0;
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
    recovery_mode_reason.store(
        static_cast<uint8_t>(protocol::RecoveryModeReason::recovery_wake_retry),
        std::memory_order_release);
    recovery_mode_command_sent.store(false, std::memory_order_release);
    recovery_mode_command_pending.store(true, std::memory_order_release);
    // TODO(HW_TEST): CAN command windowを実測後に確定する。
    recovery_wake_deadline_us =
        static_cast<uint64_t>(esp_timer_get_time()) + 2'000'000;
  }
""",
)

old_recovery_loop = """    RecoveryRequest request{};
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
"""
new_recovery_loop = """    RecoveryRequest request{};
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
        if (xQueueOverwrite(persistence_queue, &signal) == pdTRUE)
          enter_waiting = true;
        continue;
      }

      protocol::RecoveryStatusCode status =
          protocol::RecoveryStatusCode::invalid_state;
      switch (request.control.opcode) {
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
      const protocol::RecoveryStatusMessage response{
          request.control.opcode, request.control.transfer_id, status,
          request.control.source, 0};
      (void)xQueueSend(recovery_status_queue, &response, 0);
    }
"""
replace_once(old_recovery_loop, new_recovery_loop)

replace_once(
    """      if (recovery.markResourcesSafeAndFlushed()) {
        const protocol::RecoveryStatusMessage ready{
            protocol::RecoveryOpcode::enter_recovery,
            enter_request.transfer_id,
            protocol::RecoveryStatusCode::ready,
            enter_request.source, 0};
        (void)xQueueSend(recovery_status_queue, &ready, 0);
      }
      enter_waiting = false;
""",
    """      if (recovery.markResourcesSafeAndFlushed()) {
        recovery_mode_reason.store(static_cast<uint8_t>(enter_reason),
                                   std::memory_order_release);
        recovery_mode_command_sent.store(false, std::memory_order_release);
        recovery_mode_command_pending.store(true, std::memory_order_release);
        recovery_entry_deadline_us =
            static_cast<uint64_t>(esp_timer_get_time()) + 2'000'000;
      }
      enter_waiting = false;
""",
)

replace_once(
    """    if (recovery.mayEnterDeepSleep() &&
        (wake_window_elapsed ||
         (!recovery_only &&
          recovery_status_sent.load(std::memory_order_acquire)))) {
""",
    """    const bool entry_window_elapsed =
        !recovery_only && recovery_entry_deadline_us != 0 &&
        static_cast<uint64_t>(esp_timer_get_time()) >=
            recovery_entry_deadline_us;
    if (recovery.mayEnterDeepSleep() &&
        (wake_window_elapsed ||
         (!recovery_only &&
          (recovery_mode_command_sent.load(std::memory_order_acquire) ||
           entry_window_elapsed)))) {
""",
)

path.write_text(text, encoding="utf-8")
