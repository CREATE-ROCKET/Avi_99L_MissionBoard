#include "bringup/bringup_shell.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "bringup/power_bringup.hpp"
#include "bringup/safe_outputs.hpp"
#include "bringup/storage_bringup.hpp"
#include "config/board_config.hpp"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"

namespace bringup {
namespace {

struct Arguments {
  std::array<char *, 4> values{};
  std::size_t count{0};
  bool overflow{false};
};

Arguments splitArguments(char *line) {
  Arguments result{};
  char *cursor = line;
  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t')
      ++cursor;
    if (*cursor == '\0')
      break;
    if (result.count == result.values.size()) {
      result.overflow = true;
      return result;
    }
    result.values[result.count++] = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
      ++cursor;
    if (*cursor != '\0')
      *cursor++ = '\0';
  }
  return result;
}

bool parseUint32(const char *text, uint32_t minimum, uint32_t maximum,
                 uint32_t &value) {
  if (text == nullptr || *text == '\0' || *text == '-')
    return false;
  errno = 0;
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed < minimum ||
      parsed > maximum)
    return false;
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseFloat(const char *text, float minimum, float maximum, float &value) {
  if (text == nullptr || *text == '\0')
    return false;
  errno = 0;
  char *end = nullptr;
  const float parsed = std::strtof(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed) ||
      parsed < minimum || parsed > maximum)
    return false;
  value = parsed;
  return true;
}

bool noArguments(const Arguments &arguments) {
  return !arguments.overflow && arguments.count == 1;
}

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

const char *flashPhaseName(storage::FlashTestPhase phase) {
  switch (phase) {
  case storage::FlashTestPhase::none:
    return "none";
  case storage::FlashTestPhase::reboot_required:
    return "reboot_required";
  case storage::FlashTestPhase::reboot_verified_and_erased:
    return "reboot_verified_and_erased";
  }
  return "unknown";
}

} // 無名名前空間

esp_err_t BringupShell::initialize() {
  if (queue_ != nullptr)
    return ESP_OK;
  power_initialize_result_ = power::initialize();
  motor_initialize_result_ = motor_.initialize();

  queue_ = xQueueCreateStatic(1, sizeof(Command), queue_bytes_.data(),
                              &queue_storage_);
  if (queue_ == nullptr)
    return ESP_ERR_NO_MEM;
  worker_ = xTaskCreateStatic(workerEntry, "bringup_worker",
                              kWorkerStackWords, this, 5,
                              worker_stack_.data(), &worker_storage_);
  return worker_ == nullptr ? ESP_ERR_NO_MEM : ESP_OK;
}

void BringupShell::workerEntry(void *context) {
  static_cast<BringupShell *>(context)->workerLoop();
}

void BringupShell::workerLoop() {
  Command command{};
  while (true) {
    // commandが無い間だけ無期限待機する。driver操作のtimeoutとは分離される。
    if (xQueueReceive(queue_, &command, portMAX_DELAY) != pdTRUE)
      continue;
    const esp_err_t result = dispatch(command.text.data());
    command_running_.store(false);
    std::printf("command result: %s\nbringup> ", esp_err_to_name(result));
  }
}

void BringupShell::acceptLine() {
  line_[line_length_] = '\0';
  if (line_length_ == 0)
    return;

  if (std::strcmp(line_.data(), "motor-disarm") == 0) {
    const esp_err_t result = motor_.requestDisarm();
    std::printf("motor-disarm request: %s（通常は即時、出力lock競合時も"
                "5 ms以内に安全停止を再試行）\nbringup> ",
                esp_err_to_name(result));
    return;
  }

  if (command_running_.load()) {
    std::printf("ERR: test実行中のためcommandを拒否: %s\n", line_.data());
    std::printf("bringup> ");
    return;
  }

  Command command{};
  std::memcpy(command.text.data(), line_.data(), line_length_ + 1U);
  command_running_.store(true);
  if (xQueueSend(queue_, &command, 0) != pdTRUE) {
    command_running_.store(false);
    std::printf("ERR: command queue full\nbringup> ");
  }
}

[[noreturn]] void BringupShell::run() {
  std::printf("bringup> ");
  std::array<uint8_t, 64> input{};
  while (true) {
    const int count = usb_serial_jtag_read_bytes(
        input.data(), input.size(), pdMS_TO_TICKS(20));
    if (count <= 0)
      continue;
    for (int index = 0; index < count; ++index) {
      const uint8_t byte = input[index];
      if (byte == '\r' || byte == '\n') {
        if (!discarding_line_)
          acceptLine();
        discarding_line_ = false;
        line_length_ = 0;
      } else if (discarding_line_) {
        continue;
      } else if (byte == 0x08U || byte == 0x7FU) {
        if (line_length_ != 0)
          --line_length_;
      } else if (byte >= 0x20U && byte <= 0x7EU) {
        if (line_length_ + 1U < line_.size()) {
          line_[line_length_++] = static_cast<char>(byte);
        } else {
          line_length_ = 0;
          discarding_line_ = true;
          std::printf("ERR: commandが%zu bytes上限を超過\nbringup> ",
                      line_.size() - 1U);
        }
      } else {
        line_length_ = 0;
        discarding_line_ = true;
        std::printf("ERR: commandに非ASCII byte 0x%02X\nbringup> ", byte);
      }
    }
  }
}

void BringupShell::printHelp() const {
  std::printf(
      "help\nstatus\nspi-test\nencoder-test\nencoder-stream <seconds>\n"
      "imu-selftest\nimu-stream <seconds>\nimu-static <seconds>\n"
      "can-test\ncan-load-test <hz> <seconds>\n"
      "sts-probe\nsts-read\nsts-free\nsts-hold\nsts-small-move <deg>\n"
      "i2c-probe\nsd-test\nflash-test\nadc-stream <seconds>\n"
      "aux5v-on\naux5v-off\n"
      "calibrate\ncalibration-repeat <count>\n"
      "motor-arm\nmotor-disarm\nmotor-polarity\nmotor-step\nmotor-prbs\n"
      "motor-coast\nmotor-brake-test\ncombined-motor-imu-test\n");
}

void BringupShell::printStatus() const {
  std::printf(
      "status: safe=%s command_running=%s stream_active=%s dropped=%lu "
      "output_error=%lu\n"
      "spi: encoder_bus=%s imu_bus=%s encoder_device=%s imu_device=%s\n"
      "power: initialized=%s init_result=%s\n"
      "motor: initialized=%s init_result=%s armed=%s busy=%s pwm_hz=%lu\n"
      "gpio: IN2=%d IN1=%d AUX5V=%d PARA=%d\n",
      safe_outputs::initialized() ? "yes" : "no",
      command_running_.load() ? "yes" : "no",
      stream_.active() ? "yes" : "no",
      static_cast<unsigned long>(stream_.droppedFrames()),
      static_cast<unsigned long>(stream_.outputErrors()),
      spi_.encoderBusInitialized() ? "up" : "down",
      spi_.imuBusInitialized() ? "up" : "down",
      encoder_.initialized() ? "up" : "down",
      imu_.initialized() ? "up" : "down",
      power::initialized() ? "yes" : "no",
      esp_err_to_name(power_initialize_result_),
      motor_.initialized() ? "yes" : "no",
      esp_err_to_name(motor_initialize_result_), motor_.armed() ? "yes" : "no",
      motor_.busy() ? "yes" : "no",
      static_cast<unsigned long>(motor_.actualPwmFrequencyHz()),
      gpio_get_level(board::kMotorIn2), gpio_get_level(board::kMotorIn1),
      gpio_get_level(board::kAux5vEnable),
      gpio_get_level(board::kParaEnable));
  if (calibration_.hasLatest()) {
    const CalibrationAttempt &latest = calibration_.latest();
    std::printf(
        "calibration: latest=%lu imu_samples=%lu gyro_valid=%d "
        "gravity_valid=%d ssc_valid=%d errors=%lu\n",
        static_cast<unsigned long>(latest.id),
        static_cast<unsigned long>(latest.imu_samples), latest.gyro_valid,
        latest.gravity_valid, latest.ssc_valid,
        static_cast<unsigned long>(latest.error_count));
  } else {
    std::printf("calibration: none\n");
  }
}

esp_err_t BringupShell::dispatch(char *line) {
  const Arguments arguments = splitArguments(line);
  if (arguments.overflow || arguments.count == 0) {
    std::printf("ERR: invalid argument count\n");
    return ESP_ERR_INVALID_ARG;
  }
  const char *const command = arguments.values[0];

  if (std::strcmp(command, "help") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    printHelp();
    return ESP_OK;
  }
  if (std::strcmp(command, "status") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    printStatus();
    return ESP_OK;
  }
  if (std::strcmp(command, "spi-test") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    SpiTestResult result{};
    const esp_err_t status = spi_.test(result);
    std::printf("SPI: encoder=%s duplicate=%s end=%s second_end=%s devices=%zu "
                "imu=%s duplicate=%s end=%s second_end=%s devices=%zu "
                "invalid_config=%s timeout_api=%d pass=%d\n",
                esp_err_to_name(result.encoder.begin_result),
                esp_err_to_name(result.encoder.duplicate_begin_result),
                esp_err_to_name(result.encoder.end_result),
                esp_err_to_name(result.encoder.second_end_result),
                result.encoder.device_count,
                esp_err_to_name(result.imu.begin_result),
                esp_err_to_name(result.imu.duplicate_begin_result),
                esp_err_to_name(result.imu.end_result),
                esp_err_to_name(result.imu.second_end_result),
                result.imu.device_count,
                esp_err_to_name(result.invalid_config_result),
                result.timeout_api_valid,
                result.passed());
    return status;
  }
  if (std::strcmp(command, "encoder-test") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    esp_err_t status = spi_.begin();
    EncoderTestResult result{};
    if (status == ESP_OK)
      status = encoder_.test(spi_, result);
    if (spi_.encoderBusInitialized() || spi_.imuBusInitialized())
      rememberFirst(spi_.end(), status);
    std::printf(
        "encoder test: begin=%s status=%s read=%s pipeline=%s/%s/%s "
        "begin_flags=%d/%d/%d error_flags=%s end=%s raw=%u deg=%.6f "
        "AGC=%u magnitude=%u pass=%d\n",
                esp_err_to_name(result.begin_result),
                esp_err_to_name(result.status_result),
                esp_err_to_name(result.read_result),
                esp_err_to_name(result.pipeline_start_result),
                esp_err_to_name(result.pipeline_read_result),
                esp_err_to_name(result.pipeline_stop_result),
                result.begin_error_flags.parity_error,
                result.begin_error_flags.invalid_command,
                result.begin_error_flags.framing_error,
                esp_err_to_name(result.error_flags_result),
                esp_err_to_name(result.end_result),
                result.direct_sample.angle_raw,
        result.direct_sample.angle_degrees, result.status.agc,
        result.status.magnitude, result.passed());
    std::printf(
        "encoder status: mag_low=%d mag_high=%d cordic=%d offset_done=%d "
        "error_flags=%d/%d/%d\n",
        result.status.magnetic_too_low, result.status.magnetic_too_high,
        result.status.cordic_overflow,
        result.status.offset_compensation_finished,
        result.error_flags.parity_error, result.error_flags.invalid_command,
        result.error_flags.framing_error);
    return status;
  }
  if (std::strcmp(command, "encoder-stream") == 0) {
    uint32_t seconds = 0;
    if (arguments.count != 2 ||
        !parseUint32(arguments.values[1], 1, 3'600, seconds))
      return ESP_ERR_INVALID_ARG;
    esp_err_t status = spi_.begin();
    EncoderStreamResult result{};
    if (status == ESP_OK)
      status = encoder_.stream(spi_, seconds, stream_, result);
    if (spi_.encoderBusInitialized() || spi_.imuBusInitialized())
      rememberFirst(spi_.end(), status);
    std::printf(
        "encoder stream: samples=%lu/%lu driver=%lu parity=%lu sensor=%lu "
        "restart=%lu stream_error=%lu dropped=%lu output_error=%lu "
        "max_latency_us=%lu deadline_miss=%lu boundary=%lu raw=%u..%u "
        "final_status=%d/%d/%d final_flags=%d/%d/%d pass=%d\n",
        static_cast<unsigned long>(result.sample_count),
        static_cast<unsigned long>(result.requested_samples),
        static_cast<unsigned long>(result.driver_error_count),
        static_cast<unsigned long>(result.parity_error_count),
        static_cast<unsigned long>(result.sensor_error_count),
        static_cast<unsigned long>(result.pipeline_restart_count),
        static_cast<unsigned long>(result.stream_error_count),
        static_cast<unsigned long>(result.dropped_frames),
        static_cast<unsigned long>(result.output_errors),
        static_cast<unsigned long>(result.max_read_latency_us),
        static_cast<unsigned long>(result.deadline_miss_count),
        static_cast<unsigned long>(result.boundary_crossing_count),
        result.minimum_angle_raw, result.maximum_angle_raw,
        result.final_status.magnetic_too_low,
        result.final_status.magnetic_too_high,
        result.final_status.cordic_overflow,
        result.final_error_flags.parity_error,
        result.final_error_flags.invalid_command,
        result.final_error_flags.framing_error,
        result.passed());
    return status;
  }
  if (std::strcmp(command, "imu-selftest") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    esp_err_t status = spi_.begin();
    ImuSelfTestResult result{};
    if (status == ESP_OK)
      status = imu_.selfTest(spi_, result);
    if (spi_.encoderBusInitialized() || spi_.imuBusInitialized())
      rememberFirst(spi_.end(), status);
    std::printf("IMU selftest: who=0x%02X passed=%d restored=%d result=%s\n",
                result.who_am_i, result.detail.passed,
                result.detail.restored, esp_err_to_name(status));
    for (std::size_t axis = 0; axis < 3; ++axis) {
      std::printf(
          "IMU selftest axis=%zu accel=%d baseline=%ld stimulated=%ld "
          "response=%ld gyro=%d baseline=%ld stimulated=%ld response=%ld\n",
          axis, result.detail.accel_passed[axis],
          static_cast<long>(result.detail.accel_baseline[axis]),
          static_cast<long>(result.detail.accel_stimulated[axis]),
          static_cast<long>(result.detail.accel_response[axis]),
          result.detail.gyro_passed[axis],
          static_cast<long>(result.detail.gyro_baseline[axis]),
          static_cast<long>(result.detail.gyro_stimulated[axis]),
          static_cast<long>(result.detail.gyro_response[axis]));
    }
    return status;
  }
  if (std::strcmp(command, "imu-stream") == 0 ||
      std::strcmp(command, "imu-static") == 0) {
    uint32_t seconds = 0;
    if (arguments.count != 2 ||
        !parseUint32(arguments.values[1], 1, 3'600, seconds))
      return ESP_ERR_INVALID_ARG;
    esp_err_t status = spi_.begin();
    ImuStreamResult result{};
    if (status == ESP_OK) {
      status = std::strcmp(command, "imu-static") == 0
                   ? imu_.staticCapture(spi_, seconds, stream_, result)
                   : imu_.stream(spi_, seconds, stream_, result);
    }
    if (spi_.encoderBusInitialized() || spi_.imuBusInitialized())
      rememberFirst(spi_.end(), status);
    std::printf(
        "IMU stream: samples=%lu/%lu batches=%lu driver=%lu stream_error=%lu "
        "dropped=%lu output_error=%lu invalid_a=%lu invalid_g=%lu "
        "invalid_t=%lu odr_change=%lu/%lu lost=%u full=%lu fault=%lu "
        "nonmonotonic=%lu max_latency_us=%lu final_records=%u "
        "final_flags=%d/%d/%d pass=%d\n",
        static_cast<unsigned long>(result.sample_count),
        static_cast<unsigned long>(result.requested_samples),
        static_cast<unsigned long>(result.batch_count),
        static_cast<unsigned long>(result.driver_error_count),
        static_cast<unsigned long>(result.stream_error_count),
        static_cast<unsigned long>(result.dropped_frames),
        static_cast<unsigned long>(result.output_errors),
        static_cast<unsigned long>(result.invalid_acceleration_count),
        static_cast<unsigned long>(result.invalid_angular_velocity_count),
        static_cast<unsigned long>(result.invalid_temperature_count),
        static_cast<unsigned long>(result.accel_odr_change_count),
        static_cast<unsigned long>(result.gyro_odr_change_count),
        result.maximum_lost_packets,
        static_cast<unsigned long>(result.fifo_full_count),
        static_cast<unsigned long>(result.fifo_fault_count),
        static_cast<unsigned long>(result.timestamp_nonmonotonic_count),
        static_cast<unsigned long>(result.max_read_latency_us),
        result.final_fifo_status.records_available,
        result.final_fifo_status.full, result.final_fifo_status.faulted,
        result.final_fifo_status.lost_packets != 0, result.passed());
    return status;
  }
  if (std::strcmp(command, "can-test") == 0) {
    return noArguments(arguments) ? can_.test() : ESP_ERR_INVALID_ARG;
  }
  if (std::strcmp(command, "can-load-test") == 0) {
    uint32_t frequency = 0;
    uint32_t seconds = 0;
    return arguments.count == 3 &&
                   parseUint32(arguments.values[1], 1, 1'000, frequency) &&
                   parseUint32(arguments.values[2], 1, 3'600, seconds)
               ? can_.loadTest(frequency, seconds)
               : ESP_ERR_INVALID_ARG;
  }
  if (std::strcmp(command, "sts-probe") == 0)
    return noArguments(arguments) ? sts_.probe() : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "sts-read") == 0)
    return noArguments(arguments) ? sts_.read() : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "sts-free") == 0)
    return noArguments(arguments) ? sts_.free() : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "sts-hold") == 0)
    return noArguments(arguments) ? sts_.hold() : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "sts-small-move") == 0) {
    float degrees = 0.0F;
    return arguments.count == 2 &&
                   parseFloat(arguments.values[1], -3.0F, 3.0F, degrees) &&
                   degrees != 0.0F
               ? sts_.smallMove(degrees)
               : ESP_ERR_INVALID_ARG;
  }
  if (std::strcmp(command, "i2c-probe") == 0)
    return noArguments(arguments) ? i2c_.probe() : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "sd-test") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    storage::SdTestResult result{};
    const esp_err_t status = storage::sdTest(result);
    std::printf(
        "SD: bytes=%lu/%lu crc=%08lX/%08lX match=%d unmount=%d "
        "write=%.3fMiB/s read=%.3fMiB/s max_block_us=%llu errno=%d\n",
        static_cast<unsigned long>(result.written_bytes),
        static_cast<unsigned long>(result.requested_bytes),
        static_cast<unsigned long>(result.expected_crc32),
        static_cast<unsigned long>(result.read_crc32), result.content_matches,
        result.unmounted, result.write_mib_per_second,
        result.read_mib_per_second,
        static_cast<unsigned long long>(result.max_block_us), result.io_errno);
    return status;
  }
  if (std::strcmp(command, "flash-test") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    storage::FlashTestResult result{};
    const esp_err_t status = storage::flashTest(result);
    std::printf(
        "Flash: phase=%s partition=%d bytes=%lu crc=%08lX/%08lX match=%d "
        "erased=%d max_block_us=%llu\n",
        flashPhaseName(result.phase), result.partition_verified,
        static_cast<unsigned long>(result.tested_bytes),
        static_cast<unsigned long>(result.expected_crc32),
        static_cast<unsigned long>(result.read_crc32), result.content_matches,
        result.erased, static_cast<unsigned long long>(result.max_block_us));
    return status;
  }
  if (std::strcmp(command, "adc-stream") == 0) {
    uint32_t seconds = 0;
    if (arguments.count != 2 ||
        !parseUint32(arguments.values[1], 1, 3'600, seconds))
      return ESP_ERR_INVALID_ARG;
    power::AdcStreamResult result{};
    const esp_err_t status = power::adcStream(seconds, stream_, result);
    std::printf(
        "ADC: samples=%lu/%lu error=%lu stream_error=%lu dropped=%lu "
        "output_error=%lu deadline_miss=%lu duration_us=%llu "
        "max_latency_us=%llu logic_raw=%ld..%ld logic_pin_mean=%.5fV "
        "logic_source_mean=%.5fV motor_raw=%ld..%ld motor_pin_mean=%.5fV "
        "motor_source_mean=%.5fV\n",
        static_cast<unsigned long>(result.sample_count),
        static_cast<unsigned long>(result.requested_samples),
        static_cast<unsigned long>(result.adc_error_count),
        static_cast<unsigned long>(result.stream_error_count),
        static_cast<unsigned long>(result.dropped_frames),
        static_cast<unsigned long>(result.output_errors),
        static_cast<unsigned long>(result.deadline_miss_count),
        static_cast<unsigned long long>(result.duration_us),
        static_cast<unsigned long long>(result.max_sample_latency_us),
        static_cast<long>(result.logic.raw_min),
        static_cast<long>(result.logic.raw_max),
        result.logic.pin_voltage_mean_v,
        result.logic.source_voltage_mean_v,
        static_cast<long>(result.motor.raw_min),
        static_cast<long>(result.motor.raw_max),
        result.motor.pin_voltage_mean_v,
        result.motor.source_voltage_mean_v);
    return status;
  }
  if (std::strcmp(command, "aux5v-on") == 0)
    return noArguments(arguments) ? safe_outputs::setAux5v(true)
                                  : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "aux5v-off") == 0)
    return noArguments(arguments) ? safe_outputs::setAux5v(false)
                                  : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "calibrate") == 0) {
    if (!noArguments(arguments))
      return ESP_ERR_INVALID_ARG;
    CalibrationAttempt result{};
    const esp_err_t status = calibration_.calibrate(result);
    std::printf(
        "calibrate: id=%lu samples=%lu gyro=[%.6f %.6f %.6f]dps "
        "gravity_sensor=[%.6f %.6f %.6f]g norm=%.6f tilt=%.4fdeg "
        "ssc_valid=%d ssc_zero=%.3fPa errors=%lu\n",
        static_cast<unsigned long>(result.id),
        static_cast<unsigned long>(result.imu_samples),
        result.gyro_bias_dps[0], result.gyro_bias_dps[1],
        result.gyro_bias_dps[2], result.gravity_sensor_g[0],
        result.gravity_sensor_g[1], result.gravity_sensor_g[2],
        result.acceleration_norm_g, result.launcher_tilt_deg,
        result.ssc_valid, result.ssc_zero_pa,
        static_cast<unsigned long>(result.error_count));
    return status;
  }
  if (std::strcmp(command, "calibration-repeat") == 0) {
    uint32_t count = 0;
    if (arguments.count != 2 ||
        !parseUint32(arguments.values[1], 1, 100, count))
      return ESP_ERR_INVALID_ARG;
    CalibrationRepeatResult result{};
    return calibration_.repeat(count, result);
  }
  if (std::strcmp(command, "motor-arm") == 0)
    return noArguments(arguments) ? motor_.arm() : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "motor-disarm") == 0)
    return noArguments(arguments) ? motor_.disarm() : ESP_ERR_INVALID_ARG;

  MotorTestResult motor_result{};
  if (std::strcmp(command, "motor-polarity") == 0)
    return noArguments(arguments) ? motor_.polarity(motor_result)
                                  : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "motor-step") == 0)
    return noArguments(arguments) ? motor_.step(motor_result)
                                  : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "motor-prbs") == 0)
    return noArguments(arguments) ? motor_.prbs(motor_result)
                                  : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "motor-coast") == 0)
    return noArguments(arguments) ? motor_.coastTest(motor_result)
                                  : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "motor-brake-test") == 0)
    return noArguments(arguments) ? motor_.brakeTest(motor_result)
                                  : ESP_ERR_INVALID_ARG;
  if (std::strcmp(command, "combined-motor-imu-test") == 0)
    return noArguments(arguments)
               ? motor_.combinedMotorImuTest(motor_result)
               : ESP_ERR_INVALID_ARG;

  std::printf("ERR: unknown command: %s\n", command);
  return ESP_ERR_NOT_FOUND;
}

} // 名前空間 bringup
