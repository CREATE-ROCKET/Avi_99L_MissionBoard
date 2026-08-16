from pathlib import Path
import re

path = Path("src/runtime/production_runtime.cpp")
text = path.read_text()


def once(old: str, new: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"replacement count={count}, expected=1: {old[:100]!r}")
    text = text.replace(old, new, 1)


once('#include <cstring>\n', '#include <cstring>\n#include <limits>\n')
once(
    '#include "runtime/flight_log.hpp"\n',
    '#include "runtime/flight_log.hpp"\n#include "runtime/flight_runtime_metadata.hpp"\n',
)
once(
    '#include "sensors/sensor_health.hpp"\n',
    '#include "sensors/sensor_health.hpp"\n#include "sensors/power_presence_runtime.hpp"\n',
)

once(
    'std::atomic<uint8_t> motor_voltage_raw{static_cast<uint8_t>(\n'
    '    protocol::quantization::BatteryError::unavailable)};\n',
    'std::atomic<uint8_t> motor_voltage_raw{static_cast<uint8_t>(\n'
    '    protocol::quantization::BatteryError::unavailable)};\n'
    "std::atomic<uint32_t> motor_bus_millivolts{9'000};\n"
    'std::atomic<bool> motor_bus_voltage_valid{false};\n'
    'std::atomic<bool> logic_power_present{false};\n'
    'std::atomic<bool> motor_power_present{false};\n',
)

once(
    '  bool fin_angle_available = false;\n'
    '  bool fin_zero_available = false;\n',
    '  bool fin_angle_available = false;\n'
    '  bool fin_zero_available = false;\n'
    '  bool command_fin_auto_hold_allowed = true;\n'
    '  bool command_fin_boot_hold_active = false;\n'
    '  bool command_fin_target_is_unwrapped = false;\n'
    '  uint16_t latest_encoder_angle_raw = flight_log::kUnknownEncoderZeroCount;\n'
    '  flight_runtime_metadata::FinZeroMetadata fin_zero_metadata{};\n'
    '  flight_runtime_metadata::EncoderTimingMetadata encoder_timing{};\n',
)

once(
    '  encoder_ready.store(pipeline_result == ESP_OK, std::memory_order_release);\n'
    '  const esp_err_t motor_result = motor_driver.initialize();\n',
    '  encoder_ready.store(pipeline_result == ESP_OK, std::memory_order_release);\n'
    '  const uint64_t encoder_initialization_us =\n'
    '      static_cast<uint64_t>(esp_timer_get_time());\n'
    '  encoder_timing.pipeline_state =\n'
    '      pipeline_result == ESP_OK\n'
    '          ? flight_runtime_metadata::EncoderPipelineState::warming_up\n'
    '          : flight_runtime_metadata::EncoderPipelineState::faulted;\n'
    '  encoder_timing.pipeline_ready_timestamp_us =\n'
    '      pipeline_result == ESP_OK ? encoder_initialization_us : 0;\n'
    '  flight_runtime_metadata::publishEncoderTiming(encoder_timing);\n'
    '  uint64_t next_encoder_reconnect_us =\n'
    "      pipeline_result == ESP_OK ? 0 : encoder_initialization_us + 1'000'000;\n"
    '  const esp_err_t motor_result = motor_driver.initialize();\n',
)

# Emergency stop and explicit FinFree must remain Free even if the encoder reconnects.
once(
    '          pending_fin_move = {};\n'
    '          command_fin_mode = CommandFinMode::free;\n'
    '          (void)motor_driver.coast();\n',
    '          pending_fin_move = {};\n'
    '          command_fin_mode = CommandFinMode::free;\n'
    '          command_fin_auto_hold_allowed = false;\n'
    '          command_fin_boot_hold_active = false;\n'
    '          command_fin_target_is_unwrapped = false;\n'
    '          (void)motor_driver.coast();\n',
)
once(
    '      } else if (code == mission::CommandCode::fin_free) {\n'
    '        pending_fin_move = {};\n'
    '        command_fin_mode = CommandFinMode::free;\n'
    '        actuator_output_inhibited.store(false, std::memory_order_release);\n',
    '      } else if (code == mission::CommandCode::fin_free) {\n'
    '        pending_fin_move = {};\n'
    '        command_fin_mode = CommandFinMode::free;\n'
    '        command_fin_auto_hold_allowed = false;\n'
    '        command_fin_boot_hold_active = false;\n'
    '        command_fin_target_is_unwrapped = false;\n'
    '        actuator_output_inhibited.store(false, std::memory_order_release);\n',
)

old_zero = '''          fin_zero_reference_rad = unwrapped_fin_rad;
          fin_zero_available = true;
          fin_angle_rad = 0.0;
          fin_rate_valid = false;
          fin_velocity.reset();
          fin_zero_configured.store(true, std::memory_order_release);
          fin_zero_hold_valid.store(false, std::memory_order_release);
          zero_hold_controller.resetValidity();
          pending_fin_move = {};
          command_fin_mode = CommandFinMode::free;
          actuator_output_inhibited.store(false, std::memory_order_release);
          transition = mission::TransitionResult::completed;
'''
new_zero = '''          fin_zero_reference_rad = unwrapped_fin_rad;
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
'''
once(old_zero, new_zero)

once(
    '          command_fin_target_rad = 0.0;\n'
    '          command_fin_mode = CommandFinMode::zero_hold;\n'
    '          actuator_output_inhibited.store(false, std::memory_order_release);\n',
    '          command_fin_target_rad = 0.0;\n'
    '          command_fin_target_is_unwrapped = false;\n'
    '          command_fin_auto_hold_allowed = false;\n'
    '          command_fin_boot_hold_active = false;\n'
    '          command_fin_mode = CommandFinMode::zero_hold;\n'
    '          actuator_output_inhibited.store(false, std::memory_order_release);\n',
)
once(
    '            command_fin_target_rad = target;\n'
    '            command_fin_mode = CommandFinMode::relative_move;\n',
    '            command_fin_target_rad = target;\n'
    '            command_fin_target_is_unwrapped = false;\n'
    '            command_fin_auto_hold_allowed = false;\n'
    '            command_fin_boot_hold_active = false;\n'
    '            command_fin_mode = CommandFinMode::relative_move;\n',
)

# Replace only the existing single-owner 1 kHz encoder acquisition block.
pattern = re.compile(
    r'    if \(encoder\.initialized\(\)\) \{\n.*?\n'
    r'    const bool fin_sample_valid =\n'
    r'        encoder_ready\.load\(std::memory_order_acquire\) && fin_zero_available &&\n'
    r'        fin_rate_valid && std::isfinite\(fin_angle_rad\) &&\n'
    r'        std::isfinite\(fin_rate_rad_s\);',
    re.S,
)
match = pattern.search(text)
if match is None:
    raise SystemExit('encoder acquisition block not found')
replacement = '''    const uint64_t encoder_now_us = static_cast<uint64_t>(esp_timer_get_time());
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
          std::printf("AS5047D reconnected\\n");
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
    if (mission_snapshot.state == protocol::MissionState::command_receive &&
        command_fin_auto_hold_allowed && command_fin_mode == CommandFinMode::free &&
        fin_observation_valid &&
        !actuator_output_inhibited.load(std::memory_order_acquire)) {
      command_fin_target_rad = unwrapped_fin_rad;
      command_fin_target_is_unwrapped = true;
      command_fin_boot_hold_active = true;
      command_fin_mode = CommandFinMode::position_hold;
    }'''
text = text[: match.start()] + replacement + text[match.end() :]

old_command_hold = '''    } else if (mission_snapshot.state ==
               protocol::MissionState::command_receive) {
      if (command_fin_mode == CommandFinMode::free) {
        motor_output_result = motor_driver.coast();
        motor_output_coasting = true;
      } else if (!motor_ready.load(std::memory_order_acquire) ||
                 !motor_driver.initialized() || !fin_sample_valid) {
        motor_output_result = motor_driver.brake();
        motor_output_braking = true;
        torque_error =
            protocol::quantization::TorqueError::controller_input_invalid;
      } else {
        const double target =
            command_fin_mode == CommandFinMode::zero_hold
                ? 0.0
                : command_fin_target_rad;
        const auto request = zero_hold_controller.compute(
            fin_angle_rad - target, fin_rate_rad_s);
        motor_output_result = applyTorque(request);
        motor_output_braking = !request.valid || !motor_command.valid;
      }
'''
new_command_hold = '''    } else if (mission_snapshot.state ==
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
'''
once(old_command_hold, new_command_hold)

# Both torque-mapper calls use measured motor bus voltage when ADC is valid.
old_map = '''      motor_command = torque_mapper.map(
          request.output_torque_nm, fin_angle_rad, fin_rate_rad_s,
          flight_config::kMotorBusVoltageV);
'''
new_map = '''      double motor_bus_voltage_v = flight_config::kMotorBusVoltageV;
      if (motor_bus_voltage_valid.load(std::memory_order_acquire)) {
        const uint32_t millivolts =
            motor_bus_millivolts.load(std::memory_order_acquire);
        if (millivolts >= 500U)
          motor_bus_voltage_v = static_cast<double>(millivolts) / 1'000.0;
      }
      motor_command = torque_mapper.map(
          request.output_torque_nm, fin_angle_rad, fin_rate_rad_s,
          motor_bus_voltage_v);
'''
count = text.count(old_map)
if count != 2:
    raise SystemExit(f"torque mapper count={count}, expected=2")
text = text.replace(old_map, new_map)

old_power = '''      publishVoltage(sample.logic, logic_voltage_raw, logic_numeric_valid);
      publishVoltage(sample.motor, motor_voltage_raw, motor_numeric_valid);
    }

    // Battery present threshold/debounceはVault上でTODO(HW_TEST)のため、
    // Flight Status bit5/6の判定にはまだ使用しない。
'''
new_power = '''      publishVoltage(sample.logic, logic_voltage_raw, logic_numeric_valid);
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
'''
once(old_power, new_power)

once(
    '        (status.state == protocol::MissionState::control\n'
    '             ? (1U << 4U)\n'
    '             : 0U) |\n'
    '        (can_healthy.load(std::memory_order_acquire) ? (1U << 8U) : 0U) |\n',
    '        (status.state == protocol::MissionState::control\n'
    '             ? (1U << 4U)\n'
    '             : 0U) |\n'
    '        (logic_power_present.load(std::memory_order_acquire) ? (1U << 5U) : 0U) |\n'
    '        (motor_power_present.load(std::memory_order_acquire) ? (1U << 6U) : 0U) |\n'
    '        (can_healthy.load(std::memory_order_acquire) ? (1U << 8U) : 0U) |\n',
)

# Initial IMU absence also gets a bounded 1 s reconnect attempt. Existing active-fault
# recovery remains faster (100 ms).
old_imu = '''    if (imu_data_loss_latched || imu_stale) {
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
'''
new_imu = '''    if (imu_data_loss_latched || imu_stale) {
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
'''
once(old_imu, new_imu)

path.write_text(text)
