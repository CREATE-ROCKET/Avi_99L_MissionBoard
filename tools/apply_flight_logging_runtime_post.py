from pathlib import Path

path = Path("src/runtime/production_runtime.cpp")
text = path.read_text(encoding="utf-8")


def replace_once(old: str, new: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"post置換対象が一意ではありません: count={count}\n{old[:240]}")
    text = text.replace(old, new, 1)


replace_once(
    """    status.flight_elapsed_us = mission_snapshot.elapsed_us;
    const bool air_data_error = !status.lps_sample_valid ||
""",
    """    status.flight_elapsed_us = mission_snapshot.elapsed_us;
    status.static_pressure_pa =
        static_cast<float>(latest_air_data.static_pressure_pa);
    status.ssc_temperature_celsius =
        static_cast<float>(latest_air_data.ssc_temperature_celsius);
    status.airspeed_mps = static_cast<float>(latest_air_data.airspeed_mps);
    const bool air_data_error = !status.lps_sample_valid ||
""",
)

replace_once(
    """    (void)xQueueOverwrite(status_queue, &status);

    uint16_t event_flags = 0;
""",
    """    (void)xQueueOverwrite(status_queue, &status);

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

    uint16_t event_flags = 0;
""",
)

path.write_text(text, encoding="utf-8")
