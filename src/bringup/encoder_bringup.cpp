#include "bringup/encoder_bringup.hpp"

#include <algorithm>
#include <limits>

#include "config/board_config.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace bringup {
namespace {

class ExclusiveGuard {
public:
  explicit ExclusiveGuard(std::atomic<bool> &busy) : busy_(busy) {
    bool expected = false;
    acquired_ = busy_.compare_exchange_strong(expected, true);
  }
  ~ExclusiveGuard() {
    if (acquired_)
      busy_.store(false);
  }
  [[nodiscard]] bool acquired() const { return acquired_; }

private:
  std::atomic<bool> &busy_;
  bool acquired_{false};
};

uint64_t nowUs() {
  const int64_t value = esp_timer_get_time();
  return value < 0 ? 0 : static_cast<uint64_t>(value);
}

uint32_t elapsedUs(uint64_t started_at) {
  const uint64_t now = nowUs();
  const uint64_t elapsed = now >= started_at ? now - started_at : 0;
  return static_cast<uint32_t>(
      std::min<uint64_t>(elapsed, std::numeric_limits<uint32_t>::max()));
}

void rememberFirst(esp_err_t operation, esp_err_t &first_error) {
  if (first_error == ESP_OK && operation != ESP_OK)
    first_error = operation;
}

bool hasError(const AS5047D::ErrorFlags &flags) {
  return flags.parity_error || flags.invalid_command || flags.framing_error;
}

constexpr bool statusResponseAllZero(const AS5047D::Status &status) {
  return !status.magnetic_too_low && !status.magnetic_too_high &&
         !status.cordic_overflow && !status.offset_compensation_finished &&
         status.agc == 0 && status.magnitude == 0;
}

constexpr bool hasSensorFault(const AS5047D::Status &status) {
  return status.magnetic_too_low || status.magnetic_too_high ||
         status.cordic_overflow || statusResponseAllZero(status);
}

esp_err_t validatedStatusResult(esp_err_t result,
                                const AS5047D::Status &status) {
  // MISO stuck-lowではparityを含む全responseが0となりdriver上は成功し得る。
  // 角度0自体は正常値なので、DIAG/AGC/magnitude全体が0のstatusだけを拒否する。
  return result == ESP_OK && statusResponseAllZero(status)
             ? ESP_ERR_INVALID_RESPONSE
             : result;
}

static_assert(hasSensorFault(AS5047D::Status{}));
static_assert(!hasSensorFault(
    AS5047D::Status{false, false, false, true, 0, 0}));
static_assert(!hasSensorFault(
    AS5047D::Status{false, false, false, false, 0, 1}));

bool appendSample(StreamPayload &payload, const EncoderSample &sample) {
  return payload.u64(sample.host_timestamp_us) &&
         payload.u16(sample.angle_raw) && payload.f32(sample.angle_degrees) &&
         payload.f32(sample.angle_radians) &&
         payload.u32(sample.read_latency_us) &&
         payload.u8(sample.valid ? 0x01U : 0x00U);
}

} // 無名名前空間

bool EncoderTestResult::passed() const {
  return begin_result == ESP_OK && status_result == ESP_OK &&
         read_result == ESP_OK && pipeline_start_result == ESP_OK &&
         pipeline_read_result == ESP_OK && pipeline_stop_result == ESP_OK &&
         error_flags_result == ESP_OK && end_result == ESP_OK &&
         direct_sample.valid && pipelined_sample.valid &&
         !hasSensorFault(status) && !hasError(error_flags);
}

bool EncoderStreamResult::passed() const {
  return requested_samples != 0 && sample_count == requested_samples &&
         driver_error_count == 0 && parity_error_count == 0 &&
         sensor_error_count == 0 && stream_error_count == 0 &&
         dropped_frames == 0 && output_errors == 0 &&
         deadline_miss_count == 0 &&
         begin_result == ESP_OK && stream_begin_result == ESP_OK &&
         pipeline_start_result == ESP_OK && pipeline_stop_result == ESP_OK &&
         final_status_result == ESP_OK &&
         final_error_flags_result == ESP_OK &&
         stream_finish_result == ESP_OK && end_result == ESP_OK &&
         !hasSensorFault(final_status) && !hasError(final_error_flags);
}

esp_err_t EncoderBringup::beginImpl(SpiBringup &spi) {
  SPICREATE *const bus = spi.encoderBus();
  if (bus == nullptr)
    return ESP_ERR_INVALID_STATE;
  AS5047D::Config config{};
  config.frequency_hz = board::kEncoderSpiFrequencyHz;
  config.angle_source = AS5047D::AngleSource::compensated;
  return encoder_.begin(*bus, board::kEncoderCs, config);
}

esp_err_t EncoderBringup::readImpl(EncoderSample &sample, bool pipelined) {
  AS5047D::Data data{};
  const uint64_t started_at = nowUs();
  const esp_err_t result =
      pipelined ? encoder_.readPipelined(data) : encoder_.read(data);
  const uint32_t latency = elapsedUs(started_at);
  if (result == ESP_OK) {
    sample = {nowUs(), data.angle_raw, data.angle_degrees,
              data.angle_radians, latency, true};
  }
  return result;
}

esp_err_t EncoderBringup::endImpl() { return encoder_.end(); }

esp_err_t EncoderBringup::begin(SpiBringup &spi) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? beginImpl(spi) : ESP_ERR_INVALID_STATE;
}

esp_err_t EncoderBringup::read(EncoderSample &sample) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? readImpl(sample, false) : ESP_ERR_INVALID_STATE;
}

esp_err_t EncoderBringup::startPipelinedRead() {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? encoder_.startPipelinedRead()
                          : ESP_ERR_INVALID_STATE;
}

esp_err_t EncoderBringup::readPipelined(EncoderSample &sample) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? readImpl(sample, true) : ESP_ERR_INVALID_STATE;
}

esp_err_t EncoderBringup::stopPipelinedRead() {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? encoder_.stopPipelinedRead()
                          : ESP_ERR_INVALID_STATE;
}

esp_err_t EncoderBringup::getStatus(AS5047D::Status &status) {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  return validatedStatusResult(encoder_.getStatus(status), status);
}

esp_err_t
EncoderBringup::readAndClearErrorFlags(AS5047D::ErrorFlags &flags) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? encoder_.readAndClearErrorFlags(flags)
                          : ESP_ERR_INVALID_STATE;
}

esp_err_t EncoderBringup::end() {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? endImpl() : ESP_ERR_INVALID_STATE;
}

esp_err_t EncoderBringup::test(SpiBringup &spi, EncoderTestResult &result) {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired() || encoder_.initialized())
    return ESP_ERR_INVALID_STATE;

  result = {};
  result.begin_result = beginImpl(spi);
  if (result.begin_result != ESP_OK) {
    result.begin_error_flags = encoder_.lastErrorFlags();
    return result.begin_result;
  }

  result.status_result =
      validatedStatusResult(encoder_.getStatus(result.status), result.status);
  result.read_result = readImpl(result.direct_sample, false);
  result.pipeline_start_result = encoder_.startPipelinedRead();
  if (result.pipeline_start_result == ESP_OK) {
    result.pipeline_read_result = readImpl(result.pipelined_sample, true);
    if (encoder_.pipelinedReadActive())
      result.pipeline_stop_result = encoder_.stopPipelinedRead();
  }
  if (!encoder_.pipelinedReadActive())
    result.error_flags_result =
        encoder_.readAndClearErrorFlags(result.error_flags);
  result.end_result = endImpl();

  esp_err_t first_error = ESP_OK;
  rememberFirst(result.status_result, first_error);
  rememberFirst(result.read_result, first_error);
  rememberFirst(result.pipeline_start_result, first_error);
  rememberFirst(result.pipeline_read_result, first_error);
  rememberFirst(result.pipeline_stop_result, first_error);
  rememberFirst(result.error_flags_result, first_error);
  rememberFirst(result.end_result, first_error);
  if (first_error == ESP_OK && !result.passed())
    first_error = ESP_FAIL;
  return first_error;
}

esp_err_t EncoderBringup::stream(SpiBringup &spi, uint32_t seconds,
                                 StreamProtocol &protocol,
                                 EncoderStreamResult &result) {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired() || encoder_.initialized() || protocol.active())
    return ESP_ERR_INVALID_STATE;
  if (seconds == 0)
    return ESP_ERR_INVALID_ARG;
  const uint64_t requested =
      static_cast<uint64_t>(seconds) * board::kSensorRateHz;
  if (requested > std::numeric_limits<uint32_t>::max())
    return ESP_ERR_INVALID_ARG;

  static_assert(configTICK_RATE_HZ % board::kSensorRateHz == 0,
                "sensor rate must divide the RTOS tick rate");
  constexpr TickType_t kPeriodTicks =
      configTICK_RATE_HZ / board::kSensorRateHz;
  constexpr uint64_t kPeriodUs = 1'000'000U / board::kSensorRateHz;

  result = {};
  result.requested_samples = static_cast<uint32_t>(requested);
  esp_err_t first_error = ESP_OK;
  result.begin_result = beginImpl(spi);
  rememberFirst(result.begin_result, first_error);

  if (result.begin_result == ESP_OK) {
    const esp_err_t initialize_result = protocol.initialize();
    rememberFirst(initialize_result, first_error);
    if (initialize_result == ESP_OK) {
      result.stream_begin_result = protocol.beginCapture();
      rememberFirst(result.stream_begin_result, first_error);
    }
  }
  if (result.stream_begin_result == ESP_OK) {
    result.pipeline_start_result = encoder_.startPipelinedRead();
    rememberFirst(result.pipeline_start_result, first_error);
  }

  bool have_previous = false;
  uint16_t previous_raw = 0;
  result.minimum_angle_raw = std::numeric_limits<uint16_t>::max();
  TickType_t wake_time = xTaskGetTickCount();
  const uint64_t capture_started_at = nowUs();
  for (uint32_t index = 0;
       index < result.requested_samples &&
       result.pipeline_start_result == ESP_OK;
       ++index) {
    vTaskDelayUntil(&wake_time, kPeriodTicks);
    const uint64_t scheduled =
        capture_started_at + static_cast<uint64_t>(index + 1U) * kPeriodUs;
    if (nowUs() > scheduled + kPeriodUs)
      ++result.deadline_miss_count;

    EncoderSample sample{};
    const esp_err_t read_result = readImpl(sample, true);
    if (read_result != ESP_OK) {
      if (read_result == ESP_ERR_INVALID_CRC)
        ++result.parity_error_count;
      else if (read_result == ESP_ERR_INVALID_RESPONSE)
        ++result.sensor_error_count;
      else
        ++result.driver_error_count;

      if (!encoder_.pipelinedReadActive()) {
        const esp_err_t restart = encoder_.startPipelinedRead();
        if (restart == ESP_OK)
          ++result.pipeline_restart_count;
        else
          rememberFirst(restart, first_error);
      }
      continue;
    }

    ++result.sample_count;
    result.max_read_latency_us =
        std::max(result.max_read_latency_us, sample.read_latency_us);
    result.minimum_angle_raw =
        std::min(result.minimum_angle_raw, sample.angle_raw);
    result.maximum_angle_raw =
        std::max(result.maximum_angle_raw, sample.angle_raw);
    if (have_previous) {
      const uint16_t difference = previous_raw > sample.angle_raw
                                      ? previous_raw - sample.angle_raw
                                      : sample.angle_raw - previous_raw;
      if (difference > 8192U)
        ++result.boundary_crossing_count;
    }
    previous_raw = sample.angle_raw;
    have_previous = true;

    StreamPayload payload{};
    if (!appendSample(payload, sample)) {
      ++result.stream_error_count;
      rememberFirst(ESP_ERR_INVALID_SIZE, first_error);
      continue;
    }
    const esp_err_t stream_result =
        protocol.send(StreamRecordType::encoder, payload);
    if (stream_result != ESP_OK) {
      ++result.stream_error_count;
      rememberFirst(stream_result, first_error);
    }
  }

  if (encoder_.pipelinedReadActive()) {
    result.pipeline_stop_result = encoder_.stopPipelinedRead();
    rememberFirst(result.pipeline_stop_result, first_error);
  }
  if (encoder_.initialized() && !encoder_.pipelinedReadActive()) {
    result.final_status_result = validatedStatusResult(
        encoder_.getStatus(result.final_status), result.final_status);
    rememberFirst(result.final_status_result, first_error);
    result.final_error_flags_result =
        encoder_.readAndClearErrorFlags(result.final_error_flags);
    rememberFirst(result.final_error_flags_result, first_error);
  }
  if (encoder_.initialized()) {
    result.end_result = endImpl();
    rememberFirst(result.end_result, first_error);
  }
  if (protocol.active()) {
    result.stream_finish_result = protocol.finish();
    rememberFirst(result.stream_finish_result, first_error);
  }
  result.dropped_frames = protocol.droppedFrames();
  result.output_errors = protocol.outputErrors();
  if (result.sample_count == 0)
    result.minimum_angle_raw = 0;
  if (first_error == ESP_OK && !result.passed())
    first_error = ESP_FAIL;
  return first_error;
}

} // 名前空間 bringup
