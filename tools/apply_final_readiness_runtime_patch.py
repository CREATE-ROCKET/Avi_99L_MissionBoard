#!/usr/bin/env python3
from __future__ import annotations

import hashlib
from pathlib import Path

RUNTIME = Path("src/runtime/production_runtime.cpp")
EXPECTED_BLOB = "62277019ff1847910a8644937742ab451d355226"


def git_blob_sha(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode()
    return hashlib.sha1(header + data).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, got {count}")
    return text.replace(old, new, 1)


data = RUNTIME.read_bytes()
actual_blob = git_blob_sha(data)
if actual_blob != EXPECTED_BLOB:
    raise SystemExit(
        f"runtime blob changed: expected {EXPECTED_BLOB}, got {actual_blob}"
    )
text = data.decode()

text = replace_once(
    text,
    """  const esp_err_t pipeline_result =
      encoder_status_result == ESP_OK ? encoder.startPipelinedRead()
                                      : ESP_ERR_INVALID_STATE;
  if (encoder_result == ESP_OK && encoder_status_result != ESP_OK)
    (void)encoder.end();
""",
    """  const esp_err_t pipeline_result =
      encoder_status_result == ESP_OK ? encoder.startPipelinedRead()
                                      : ESP_ERR_INVALID_STATE;
  // status fault時もtransport objectは保持する。runtime loopのreadPipelined()
  // が1秒周期の自己復旧を開始し、抜き差し後の再接続を可能にする。
""",
    "keep encoder recoverable after initial status fault",
)

text = replace_once(
    text,
    """  bool fin_angle_available = false;
  bool fin_zero_available = false;
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
""",
    """  bool fin_angle_available = false;
  bool fin_zero_available = false;
  enum class CommandFinMode : uint8_t {
    free,
    zero_hold,
    position_hold,
    relative_move,
  };
  CommandFinMode command_fin_mode = CommandFinMode::free;
  // boot直後はzeroを暗黙設定せず、最初のvalidなmotor-side絶対位置をHoldする。
  bool initial_fin_hold_pending = true;
  bool command_fin_target_uses_unwrapped = false;
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
  uint16_t latest_encoder_raw = flight_log::kUnknownEncoderZeroCount;
  uint64_t latest_encoder_sample_timestamp_us = 0;
  uint32_t latest_encoder_read_latency_us = 0;
  uint32_t encoder_recovery_count_at_zero = 0;
  struct FinZeroMetadata {
    uint16_t encoder_zero_count{flight_log::kUnknownEncoderZeroCount};
    uint64_t configured_timestamp_us{};
    uint32_t flight_epoch_id{};
    flight_log::FinZeroApproachDirection approach_direction{
        flight_log::FinZeroApproachDirection::unknown};
    flight_log::FinZeroCalibrationMethod calibration_method{
        flight_log::FinZeroCalibrationMethod::unknown};
    flight_log::FinZeroGroundVerificationStatus ground_verification_status{
        flight_log::FinZeroGroundVerificationStatus::unknown};
    float measured_bidirectional_span_rad{NAN};
  } fin_zero_metadata;
""",
    "add current-hold and fin-zero runtime metadata",
)

text = replace_once(
    text,
    """          pending_fin_move = {};
          command_fin_mode = CommandFinMode::free;
          (void)motor_driver.coast();
""",
    """          pending_fin_move = {};
          command_fin_mode = CommandFinMode::free;
          initial_fin_hold_pending = false;
          command_fin_target_uses_unwrapped = false;
          (void)motor_driver.coast();
""",
    "actuator emergency keeps requested free mode",
)

text = replace_once(
    text,
    """      actuator_output_inhibited.store(true, std::memory_order_release);
      (void)motor_driver.coast();
""",
    """      actuator_output_inhibited.store(true, std::memory_order_release);
      initial_fin_hold_pending = false;
      command_fin_target_uses_unwrapped = false;
      command_fin_mode = CommandFinMode::free;
      (void)motor_driver.coast();
""",
    "emergency latch keeps requested free mode",
)

text = replace_once(
    text,
    """        if (transition == mission::TransitionResult::completed) {
          actuator_output_inhibited.store(false, std::memory_order_release);
          if (forced)
            final_detail = pending_start.readiness.missingMask();
          recovery_boot::storeFlightCheckpoint(state_machine.snapshot());
""",
    """        if (transition == mission::TransitionResult::completed) {
          actuator_output_inhibited.store(false, std::memory_order_release);
          if (forced)
            final_detail = pending_start.readiness.missingMask();
          if (fin_zero_configured.load(std::memory_order_acquire))
            fin_zero_metadata.flight_epoch_id =
                state_machine.snapshot().flight_epoch;
          recovery_boot::storeFlightCheckpoint(state_machine.snapshot());
""",
    "bind fin-zero metadata to flight epoch",
)

text = replace_once(
    text,
    """      } else if (code == mission::CommandCode::fin_free) {
        pending_fin_move = {};
        command_fin_mode = CommandFinMode::free;
        actuator_output_inhibited.store(false, std::memory_order_release);
        transition = mission::TransitionResult::completed;
""",
    """      } else if (code == mission::CommandCode::fin_free) {
        pending_fin_move = {};
        command_fin_mode = CommandFinMode::free;
        initial_fin_hold_pending = false;
        command_fin_target_uses_unwrapped = false;
        actuator_output_inhibited.store(false, std::memory_order_release);
        transition = mission::TransitionResult::completed;
""",
    "FinFree disables boot auto-hold",
)

text = replace_once(
    text,
    """          pending_fin_move = {};
          command_fin_mode = CommandFinMode::free;
          actuator_output_inhibited.store(false, std::memory_order_release);
          transition = mission::TransitionResult::completed;
        }
      } else if (code == mission::CommandCode::start_fin_zero_hold) {
""",
    """          pending_fin_move = {};
          command_fin_mode = CommandFinMode::free;
          initial_fin_hold_pending = false;
          command_fin_target_uses_unwrapped = false;
          actuator_output_inhibited.store(false, std::memory_order_release);
          fin_zero_metadata.encoder_zero_count = latest_encoder_raw;
          fin_zero_metadata.configured_timestamp_us =
              static_cast<uint64_t>(esp_timer_get_time());
          fin_zero_metadata.flight_epoch_id = 0;
          fin_zero_metadata.approach_direction =
              flight_log::FinZeroApproachDirection::unknown;
          fin_zero_metadata.calibration_method =
              flight_log::FinZeroCalibrationMethod::current_position;
          fin_zero_metadata.ground_verification_status =
              flight_log::FinZeroGroundVerificationStatus::unverified;
          fin_zero_metadata.measured_bidirectional_span_rad = NAN;
          encoder_recovery_count_at_zero = encoder.recoveryCount();
          transition = mission::TransitionResult::completed;
        }
      } else if (code == mission::CommandCode::start_fin_zero_hold) {
""",
    "record SetFinZero provenance",
)

text = replace_once(
    text,
    """          pending_fin_move = {};
          command_fin_target_rad = 0.0;
          command_fin_mode = CommandFinMode::zero_hold;
          actuator_output_inhibited.store(false, std::memory_order_release);
""",
    """          pending_fin_move = {};
          command_fin_target_rad = 0.0;
          command_fin_mode = CommandFinMode::zero_hold;
          initial_fin_hold_pending = false;
          command_fin_target_uses_unwrapped = false;
          actuator_output_inhibited.store(false, std::memory_order_release);
""",
    "zero-hold selects zero-relative feedback",
)

text = replace_once(
    text,
    """            command_fin_target_rad = target;
            command_fin_mode = CommandFinMode::relative_move;
            pending_fin_move =
""",
    """            command_fin_target_rad = target;
            command_fin_mode = CommandFinMode::relative_move;
            initial_fin_hold_pending = false;
            command_fin_target_uses_unwrapped = false;
            pending_fin_move =
""",
    "relative move selects zero-relative feedback",
)

old_encoder = """    if (encoder.initialized()) {
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
        if (fin_zero_available) {
          fin_angle_rad = unwrapped_fin_rad - fin_zero_reference_rad;
          fin_rate_valid = fin_velocity.update(sample.host_timestamp_us,
                                               fin_angle_rad,
                                               fin_rate_rad_s);
        } else {
          fin_rate_valid = false;
          fin_velocity.reset();
        }
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
"""
new_encoder = """    if (encoder.initialized()) {
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
        latest_encoder_raw = sample.angle_raw;
        latest_encoder_sample_timestamp_us = sample.host_timestamp_us;
        latest_encoder_read_latency_us = sample.read_latency_us;
        if (!fin_angle_available) {
          if (fin_zero_available || command_fin_target_uses_unwrapped) {
            const double reconnect_reference =
                fin_zero_available ? unwrapped_fin_rad : command_fin_target_rad;
            const double turns =
                std::round((reconnect_reference - wrapped) / kTwoPi);
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
        const double estimator_angle =
            fin_zero_available ? fin_angle_rad : unwrapped_fin_rad;
        fin_rate_valid = fin_velocity.update(sample.host_timestamp_us,
                                             estimator_angle,
                                             fin_rate_rad_s);
        encoder_ready.store(true, std::memory_order_release);

        if (initial_fin_hold_pending &&
            detector_state == protocol::MissionState::command_receive &&
            fin_rate_valid) {
          command_fin_target_rad = unwrapped_fin_rad;
          command_fin_target_uses_unwrapped = true;
          command_fin_mode = CommandFinMode::position_hold;
          initial_fin_hold_pending = false;
        }
      }
    } else {
      encoder_ready.store(false, std::memory_order_release);
      fin_rate_valid = false;
    }
    const bool fin_motion_sample_valid =
        encoder_ready.load(std::memory_order_acquire) && fin_angle_available &&
        fin_rate_valid && std::isfinite(unwrapped_fin_rad) &&
        std::isfinite(fin_rate_rad_s);
    const bool fin_sample_valid =
        fin_motion_sample_valid && fin_zero_available &&
        std::isfinite(fin_angle_rad);
"""
text = replace_once(text, old_encoder, new_encoder, "direct 1 kHz encoder and boot hold")

old_command_output = """      if (command_fin_mode == CommandFinMode::free) {
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
"""
new_command_output = """      if (command_fin_mode == CommandFinMode::free) {
        if (initial_fin_hold_pending) {
          // encoder位置が確定するまでは勝手に回さずBrake。valid rate取得後に
          // motor-side絶対位置Holdへ切り替える。
          motor_output_result = motor_driver.brake();
          motor_output_braking = true;
        } else {
          motor_output_result = motor_driver.coast();
          motor_output_coasting = true;
        }
      } else {
        const bool command_sample_valid =
            command_fin_target_uses_unwrapped ? fin_motion_sample_valid
                                              : fin_sample_valid;
        if (!motor_ready.load(std::memory_order_acquire) ||
            !motor_driver.initialized() || !command_sample_valid) {
          motor_output_result = motor_driver.brake();
          motor_output_braking = true;
          torque_error =
              protocol::quantization::TorqueError::controller_input_invalid;
        } else {
          const double target =
              command_fin_mode == CommandFinMode::zero_hold
                  ? 0.0
                  : command_fin_target_rad;
          const double feedback = command_fin_target_uses_unwrapped
                                      ? unwrapped_fin_rad
                                      : fin_angle_rad;
          const auto request = zero_hold_controller.compute(
              feedback - target, fin_rate_rad_s);
          motor_output_result = applyTorque(request);
          motor_output_braking = !request.valid || !motor_command.valid;
        }
      }
"""
text = replace_once(text, old_command_output, new_command_output, "CommandReceive current-position hold")

text = replace_once(
    text,
    """      const flight_log::Sample log_sample{
          static_cast<uint64_t>(esp_timer_get_time()),
""",
    """      const uint64_t log_timestamp_us =
          static_cast<uint64_t>(esp_timer_get_time());
      uint32_t encoder_sample_age_us = 0xFFFF'FFFFU;
      if (latest_encoder_sample_timestamp_us != 0 &&
          log_timestamp_us >= latest_encoder_sample_timestamp_us) {
        const uint64_t age = log_timestamp_us - latest_encoder_sample_timestamp_us;
        encoder_sample_age_us = static_cast<uint32_t>(
            std::min<uint64_t>(age, 0xFFFF'FFFFULL));
      }
      uint8_t encoder_diagnostic_flags = 0;
      if (encoder_ready.load(std::memory_order_acquire))
        encoder_diagnostic_flags |= flight_log::encoder_sample_valid;
      if (fin_rate_valid)
        encoder_diagnostic_flags |= flight_log::encoder_rate_valid;
      if (fin_zero_available &&
          encoder.recoveryCount() != encoder_recovery_count_at_zero)
        encoder_diagnostic_flags |=
            flight_log::encoder_reconnected_since_zero;

      const flight_log::Sample log_sample{
          log_timestamp_us,
""",
    "prepare encoder diagnostic log fields",
)

text = replace_once(
    text,
    """          fin_zero_configured.load(std::memory_order_acquire),
          flight_log::kUnknownEncoderZeroCount,
""",
    """          fin_zero_configured.load(std::memory_order_acquire),
          fin_zero_metadata.encoder_zero_count,
""",
    "write encoder zero raw count",
)

text = replace_once(
    text,
    """          static_cast<float>(flight_config::kAirData.pitot_coefficient_true_max),
          NAN};
""",
    """          static_cast<float>(flight_config::kAirData.pitot_coefficient_true_max),
          fin_zero_metadata.measured_bidirectional_span_rad,
          fin_zero_metadata.configured_timestamp_us,
          fin_zero_metadata.flight_epoch_id,
          fin_zero_metadata.approach_direction,
          fin_zero_metadata.calibration_method,
          fin_zero_metadata.ground_verification_status,
          encoder_diagnostic_flags,
          latest_encoder_sample_timestamp_us,
          latest_encoder_read_latency_us,
          encoder_sample_age_us,
          encoder.recoveryCount(),
          encoder.runtimeErrorCount()};
""",
    "append schema v2 fin-zero and encoder diagnostics",
)

RUNTIME.write_text(text)
print("runtime patch applied")
