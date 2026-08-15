from pathlib import Path

path = Path("src/runtime/production_runtime.cpp")
text = path.read_text(encoding="utf-8")


def replace_once(old: str, new: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"置換対象が一意ではありません: count={count}\n{old[:240]}")
    text = text.replace(old, new, 1)


replace_once(
    '#include "runtime/recovery_boot.hpp"\n#include "runtime/emergency_latch.hpp"\n',
    '#include "runtime/recovery_boot.hpp"\n#include "runtime/emergency_latch.hpp"\n#include "runtime/flight_log.hpp"\n#include "runtime/flight_storage.hpp"\n',
)

replace_once(
    """  uint64_t flight_elapsed_us{};
  uint16_t roll_raw{
""",
    """  uint64_t flight_elapsed_us{};
  float static_pressure_pa{};
  float ssc_temperature_celsius{};
  float airspeed_mps{};
  uint16_t roll_raw{
""",
)

replace_once(
    """struct RecoveryRequest {
  enum class Kind : uint8_t { control, enter } kind{Kind::control};
  protocol::RecoveryControl control{};
  protocol::RecoveryModeReason reason{
      protocol::RecoveryModeReason::auto_elapsed_120};
};
""",
    """struct RecoveryRequest {
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
""",
)

replace_once(
    """StaticQueue_t recovery_status_queue_storage;
StaticQueue_t event_queue_storage;
StaticQueue_t persistence_queue_storage;
""",
    """StaticQueue_t recovery_status_queue_storage;
StaticQueue_t recovery_log_data_queue_storage;
StaticQueue_t sd_recovery_queue_storage;
StaticQueue_t flash_log_queue_storage;
StaticQueue_t sd_log_queue_storage;
StaticQueue_t event_queue_storage;
StaticQueue_t persistence_queue_storage;
""",
)

replace_once(
    """std::array<uint8_t, sizeof(protocol::RecoveryStatusMessage) * 4>
    recovery_status_queue_buffer{};
std::array<uint8_t, sizeof(EventRequest) * 16> event_queue_buffer{};
""",
    """std::array<uint8_t, sizeof(protocol::RecoveryStatusMessage) * 4>
    recovery_status_queue_buffer{};
std::array<uint8_t, sizeof(protocol::RecoveryLogData) * 16>
    recovery_log_data_queue_buffer{};
std::array<uint8_t, sizeof(protocol::RecoveryControl) * 4>
    sd_recovery_queue_buffer{};
std::array<uint8_t, sizeof(flight_log::SerializedRecord) * 32>
    flash_log_queue_buffer{};
std::array<uint8_t, sizeof(flight_log::SerializedRecord) * 64>
    sd_log_queue_buffer{};
std::array<uint8_t, sizeof(EventRequest) * 16> event_queue_buffer{};
""",
)

replace_once(
    """QueueHandle_t recovery_queue{};
QueueHandle_t recovery_status_queue{};
QueueHandle_t event_queue{};
""",
    """QueueHandle_t recovery_queue{};
QueueHandle_t recovery_status_queue{};
QueueHandle_t recovery_log_data_queue{};
QueueHandle_t sd_recovery_queue{};
QueueHandle_t flash_log_queue{};
QueueHandle_t sd_log_queue{};
QueueHandle_t event_queue{};
""",
)

replace_once(
    """std::atomic<uint32_t> result_queue_overflow{};
std::atomic<uint32_t> emergency_metadata_overflow{};
""",
    """std::atomic<uint32_t> result_queue_overflow{};
std::atomic<uint32_t> emergency_metadata_overflow{};
std::atomic<uint32_t> flash_log_drop_count{};
std::atomic<uint32_t> sd_log_drop_count{};
std::atomic<bool> flash_log_ready{false};
std::atomic<bool> sd_log_ready{false};
std::atomic<bool> flash_log_failed{false};
std::atomic<bool> sd_log_failed{false};
""",
)

replace_once(
    """StaticSemaphore_t executor_mutex_storage;

constexpr std::size_t kTaskCount = kTaskArchitecture.size();
""",
    """StaticSemaphore_t executor_mutex_storage;
flight_storage::InternalFlashLog internal_flash_log;
flight_storage::SdFlightLog sd_flight_log;

constexpr std::size_t kTaskCount = kTaskArchitecture.size();
""",
)

replace_once(
    """      status.flight_elapsed_us = mission_snapshot.elapsed_us;
      if (latest_air_data.lps_valid) {
""",
    """      status.flight_elapsed_us = mission_snapshot.elapsed_us;
      status.static_pressure_pa =
          static_cast<float>(latest_air_data.static_pressure_pa);
      status.ssc_temperature_celsius =
          static_cast<float>(latest_air_data.ssc_temperature_celsius);
      status.airspeed_mps = static_cast<float>(latest_air_data.airspeed_mps);
      if (latest_air_data.lps_valid) {
""",
)

replace_once(
    """      (void)xQueueOverwrite(status_queue, &status);
    }

    resetWatchdog();
""",
    """      (void)xQueueOverwrite(status_queue, &status);

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
        if (++flash_decimation >= 10U) {
          flash_decimation = 0;
          if (xQueueSend(flash_log_queue, &serialized, 0) != pdTRUE)
            flash_log_drop_count.fetch_add(1, std::memory_order_relaxed);
        }
      } else {
        flash_decimation = 0;
      }
    }

    resetWatchdog();
""",
)

replace_once(
    """      context.resources_preallocated =
          flight_config::nonBypassFlightConfigurationReady();
""",
    """      context.resources_preallocated =
          flight_config::nonBypassFlightConfigurationReady() &&
          flash_log_ready.load(std::memory_order_acquire) &&
          sd_log_ready.load(std::memory_order_acquire);
""",
)

replace_once(
    """  const esp_err_t nvs_result = nvs_flash_init();
  esp_err_t persistence_runtime_error = nvs_result;
""",
    """  const bool recovery_boot_mode =
      recovery_only_mode.load(std::memory_order_acquire);
  const esp_err_t flash_log_result =
      recovery_boot_mode ? internal_flash_log.openExisting()
                         : internal_flash_log.prepareForFlight();
  flash_log_ready.store(flash_log_result == ESP_OK, std::memory_order_release);
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
""",
)

replace_once(
    """  uint64_t recovery_entry_deadline_us = 0;
  uint64_t recovery_wake_deadline_us = 0;
""",
    """  uint64_t recovery_entry_deadline_us = 0;
  uint64_t recovery_wake_deadline_us = 0;
  bool flash_flush_pending = false;
  RecoveryDumpCursor flash_dump{};
""",
)

replace_once(
    """  for (;;) {
    ParachutePersistenceRequest persistence_request{};
""",
    """  for (;;) {
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
""",
)

replace_once(
    """        const PersistenceSignal signal = PersistenceSignal::flush_and_safe;
        if (xQueueOverwrite(persistence_queue, &signal) == pdTRUE)
          enter_waiting = true;
""",
    """        const PersistenceSignal signal = PersistenceSignal::flush_and_safe;
        if (xQueueOverwrite(persistence_queue, &signal) == pdTRUE) {
          flash_flush_pending = true;
          enter_waiting = true;
        }
""",
)

old_switch = """      protocol::RecoveryStatusCode status =
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
"""
new_switch = """      if (request.control.source ==
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
"""
replace_once(old_switch, new_switch)

replace_once(
    """    if (enter_waiting &&
        recovery_power_safe.load(std::memory_order_acquire) &&
        recovery_motor_safe.load(std::memory_order_acquire) &&
        recovery_sd_flushed.load(std::memory_order_acquire)) {
      // Internal Flash writer未接続のため未flush recordは存在しない。
      persistence_flushed.store(true, std::memory_order_release);
""",
    """    if (flash_dump.active) {
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
""",
)

old_sd = """void sdLogTask(void *) {
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
"""
new_sd = """void sdLogTask(void *) {
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
  for (;;) {
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
      // SD failureでRecovery entry自体を永久停止させない。flush試行完了を表す。
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
"""
replace_once(old_sd, new_sd)

replace_once(
    """  protocol::RecoveryStatusMessage pending_recovery_status{};
  bool recovery_status_pending = false;
  protocol::CommandResult pending_command_result{};
""",
    """  protocol::RecoveryStatusMessage pending_recovery_status{};
  bool recovery_status_pending = false;
  protocol::RecoveryLogData pending_recovery_log_data{};
  bool recovery_log_data_pending = false;
  protocol::CommandResult pending_command_result{};
""",
)

replace_once(
    """      if (recovery_status_pending &&
          writeFrame(can, protocol::encode(pending_recovery_status)) ==
              ESP_OK)
        recovery_status_pending = false;
      if (recovery_mode_command_pending.load(std::memory_order_acquire)) {
""",
    """      if (recovery_status_pending &&
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
""",
)

replace_once(
    """  recovery_status_queue = xQueueCreateStatic(
      4, sizeof(protocol::RecoveryStatusMessage),
      recovery_status_queue_buffer.data(), &recovery_status_queue_storage);
  event_queue = xQueueCreateStatic(16, sizeof(EventRequest),
""",
    """  recovery_status_queue = xQueueCreateStatic(
      4, sizeof(protocol::RecoveryStatusMessage),
      recovery_status_queue_buffer.data(), &recovery_status_queue_storage);
  recovery_log_data_queue = xQueueCreateStatic(
      16, sizeof(protocol::RecoveryLogData), recovery_log_data_queue_buffer.data(),
      &recovery_log_data_queue_storage);
  sd_recovery_queue = xQueueCreateStatic(
      4, sizeof(protocol::RecoveryControl), sd_recovery_queue_buffer.data(),
      &sd_recovery_queue_storage);
  flash_log_queue = xQueueCreateStatic(
      32, sizeof(flight_log::SerializedRecord), flash_log_queue_buffer.data(),
      &flash_log_queue_storage);
  sd_log_queue = xQueueCreateStatic(
      64, sizeof(flight_log::SerializedRecord), sd_log_queue_buffer.data(),
      &sd_log_queue_storage);
  event_queue = xQueueCreateStatic(16, sizeof(EventRequest),
""",
)

replace_once(
    """      recovery_queue == nullptr || recovery_status_queue == nullptr ||
      event_queue == nullptr || persistence_queue == nullptr ||
""",
    """      recovery_queue == nullptr || recovery_status_queue == nullptr ||
      recovery_log_data_queue == nullptr || sd_recovery_queue == nullptr ||
      flash_log_queue == nullptr || sd_log_queue == nullptr ||
      event_queue == nullptr || persistence_queue == nullptr ||
""",
)

path.write_text(text, encoding="utf-8")
