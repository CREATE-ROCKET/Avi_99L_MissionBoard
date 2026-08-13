#include "bringup/sts_bringup.hpp"

#include <array>
#include <cmath>
#include <cinttypes>
#include <cstdio>

#include "STS3215.h"
#include "STSCREATE.h"
#include "avi_esp_libs/timeout.h"
#include "bringup/safe_outputs.hpp"
#include "config/board_config.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace bringup {
namespace {

// TODO(HW_TEST): para電源投入後に必要な安定待ち時間を実機で確定する
constexpr uint32_t kPowerStabilizationMs = 100;
// TODO(HW_TEST): 個体差・電源条件を含むservo起動上限を実機で確定する
constexpr uint32_t kServoReadyTimeoutMs = 1'500;
constexpr uint32_t kPingRetryDelayMs = 20;
// TODO(HW_TEST): bring-up時のSTS torque上限を実機で確定する
constexpr float kTorqueLimitPercent = 10.0F;
// TODO(HW_TEST): 最初の小角度移動のspeed/accelerationを実機で確定する
constexpr float kMoveSpeedDegS = 15.0F;
constexpr float kMoveAccelerationDegS2 = 30.0F;
constexpr uint32_t kTelemetryRateHz = 50;
constexpr uint32_t kTelemetrySamples = kTelemetryRateHz;
constexpr uint32_t kMoveObservationSamples = kTelemetryRateHz * 2;
constexpr std::size_t kTelemetryCapacity = kMoveObservationSamples;

static_assert(kTelemetrySamples <= kTelemetryCapacity);

struct TelemetryRecord {
  STS3215::Data data{};
  uint64_t latency_us{};
  uint32_t sample{};
  esp_err_t read_result{ESP_OK};
  uint8_t device_error{};
  bool data_valid{};
};

struct TelemetryReport {
  std::array<TelemetryRecord, kTelemetryCapacity> records{};
  std::size_t record_count{};
  uint32_t successful{};
  uint32_t errors{};
  uint32_t timeouts{};
  uint32_t fault_samples{};
  uint32_t device_error_samples{};
  uint64_t latency_sum_us{};
  uint64_t maximum_latency_us{};
  bool stopped_on_health_error{};
  bool buffer_full{};
};

struct SmallMoveReport {
  STS3215::Status current_status{};
  esp_err_t hold_result{ESP_ERR_INVALID_STATE};
  esp_err_t current_read_result{ESP_ERR_INVALID_STATE};
  esp_err_t move_result{ESP_ERR_INVALID_STATE};
  float current_degrees{};
  float target_degrees{};
  uint8_t hold_device_error{};
  uint8_t current_device_error{};
  uint8_t move_device_error{};
  bool hold_attempted{};
  bool current_read_attempted{};
  bool move_attempted{};
  bool position_mode{};
};

class BusyGuard {
public:
  explicit BusyGuard(std::atomic<bool> &busy) : busy_(busy) {
    bool expected = false;
    acquired_ = busy_.compare_exchange_strong(expected, true);
  }
  ~BusyGuard() {
    if (acquired_)
      busy_.store(false);
  }
  [[nodiscard]] bool acquired() const { return acquired_; }

private:
  std::atomic<bool> &busy_;
  bool acquired_{false};
};

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

const char *modeName(STS3215::OperatingMode mode) {
  switch (mode) {
  case STS3215::OperatingMode::position:
    return "position";
  case STS3215::OperatingMode::velocity:
    return "velocity";
  case STS3215::OperatingMode::pwm:
    return "pwm";
  case STS3215::OperatingMode::step:
    return "step";
  }
  return "unknown";
}

bool faulted(const STS3215::Status &status) {
  return status.overload || status.overcurrent || status.overtemperature ||
         status.encoder_fault || status.voltage_fault;
}

void printStatus(const STS3215::Status &status, uint8_t device_error) {
  std::printf(
      "STS status: raw=0x%02X device_error=0x%02X overload=%d overcurrent=%d "
      "overtemperature=%d encoder_fault=%d voltage_fault=%d\n",
      status.raw, device_error, status.overload, status.overcurrent,
      status.overtemperature, status.encoder_fault, status.voltage_fault);
}

void printData(uint32_t sample, const STS3215::Data &data,
               uint8_t device_error, int64_t latency_us) {
  std::printf(
      "STS sample=%" PRIu32
      " latency_us=%" PRId64
      " position_deg=%.3f speed_deg_s=%.3f load_raw=%d current_a=%.4f "
      "voltage_v=%.2f temperature_c=%.1f moving=%d status=0x%02X "
      "device_error=0x%02X\n",
      sample, latency_us, data.position_deg, data.speed_deg_s, data.load_raw,
      data.current_a, data.voltage_v, data.temperature_celsius, data.moving,
      data.status.raw, device_error);
}

STSCREATE::Config busConfig() {
  STSCREATE::Config config{};
  config.port = board::kParaUart;
  config.tx = board::kParaTx;
  config.rx = board::kParaRx;
  config.direction_enable = board::kParaDirectionEnable;
  config.baudrate = STSCREATE::Baudrate::bps1000000;
  config.lock_timeout = avi::Timeout::noWait();
  config.tx_timeout =
      avi::Timeout::milliseconds(board::kParaTxTimeoutMs);
  config.response_timeout =
      avi::Timeout::milliseconds(board::kParaResponseTimeoutMs);
  return config;
}

class Session {
public:
  Session(STSCREATE &bus, esp_err_t initialization_result)
      : bus_(bus), initialization_result_(initialization_result) {}
  ~Session() { (void)close(); }

  [[nodiscard]] esp_err_t open() {
    if (!safe_outputs::initialized())
      return ESP_ERR_INVALID_STATE;
    if (!bus_.initialized()) {
      const esp_err_t result = initialization_result_ == ESP_OK
                                   ? ESP_ERR_INVALID_STATE
                                   : initialization_result_;
      std::printf("STS command拒否: persistent bus=down init_result=%s\n",
                  esp_err_to_name(result));
      return result;
    }
    power_attempted_ = true;
    esp_err_t result = safe_outputs::setParaPower(true);
    if (result != ESP_OK)
      return result;
    power_enabled_ = true;
    const int64_t ready_deadline_us =
        esp_timer_get_time() +
        static_cast<int64_t>(kServoReadyTimeoutMs) * 1'000;
    vTaskDelay(pdMS_TO_TICKS(kPowerStabilizationMs));

    uint32_t ping_attempt = 0;
    uint8_t ping_device_error = 0;
    while (true) {
      ++ping_attempt;
      ping_device_error = 0;
      const int64_t ping_before = esp_timer_get_time();
      result = bus_.ping(board::kParaServoId, &ping_device_error);
      ping_latency_us_ = esp_timer_get_time() - ping_before;
      std::printf("STS ping: attempt=%" PRIu32
                  " result=%s latency_us=%" PRId64
                  " device_error=0x%02X\n",
                  ping_attempt, esp_err_to_name(result), ping_latency_us_,
                  ping_device_error);
      if (result == ESP_OK)
        break;

      // 通電直後の未応答と起動中の不完全応答だけを同一電源cycleで再試行する。
      if (result != ESP_ERR_TIMEOUT && result != ESP_ERR_INVALID_RESPONSE)
        return result;
      const int64_t remaining_us = ready_deadline_us - esp_timer_get_time();
      if (remaining_us <= 0)
        return result;
      const uint32_t remaining_ms =
          static_cast<uint32_t>((remaining_us + 999) / 1'000);
      const uint32_t delay_ms = remaining_ms < kPingRetryDelayMs
                                    ? remaining_ms
                                    : kPingRetryDelayMs;
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    if (ping_device_error != 0)
      std::printf("STS ping device status: 0x%02X。設定readで詳細を確認する。\n",
                  ping_device_error);

    std::printf("STS3215 beginはPINGと設定readのみを行い、motion/torque設定は変更しない。\n");
    const int64_t before = esp_timer_get_time();
    result = servo_.begin(bus_, board::kParaServoId);
    begin_latency_us_ = esp_timer_get_time() - before;
    std::printf("STS begin: result=%s latency_us=%" PRId64 "\n",
                esp_err_to_name(result), begin_latency_us_);
    return result;
  }

  [[nodiscard]] esp_err_t close() {
    if (closed_)
      return cleanup_result_;
    closed_ = true;
    esp_err_t result = ESP_OK;

    if (servo_.initialized()) {
      disable_attempted_ = true;
      disable_result_ = servo_.disableTorque();
      disable_device_error_ = servo_.lastDeviceError();
      rememberFirst(disable_result_, result);
      if (disable_result_ == ESP_OK && disable_device_error_ != 0)
        rememberFirst(ESP_ERR_INVALID_STATE, result);
      servo_end_attempted_ = true;
      servo_end_result_ = servo_.end();
      rememberFirst(servo_end_result_, result);
    } else if (power_enabled_ && bus_.initialized()) {
      // begin失敗時も可能な範囲でTorque SwitchへOFFを書き、安全側へ戻す。
      const uint8_t disabled = 0;
      raw_disable_attempted_ = true;
      raw_disable_result_ = bus_.write(
          board::kParaServoId,
          static_cast<uint8_t>(STS3215::Register::torque_switch), &disabled, 1,
          false);
      rememberFirst(raw_disable_result_, result);
    }

    if (power_attempted_) {
      power_off_attempted_ = true;
      power_off_result_ = safe_outputs::setParaPower(false);
      rememberFirst(power_off_result_, result);
      power_attempted_ = false;
      power_enabled_ = false;
    }
    cleanup_result_ = result;
    return result;
  }

  void printCleanupReport() const {
    if (disable_attempted_)
      std::printf("STS cleanup disableTorque: %s device_error=0x%02X\n",
                  esp_err_to_name(disable_result_), disable_device_error_);
    if (raw_disable_attempted_)
      std::printf("STS cleanup raw torque OFF: %s\n",
                  esp_err_to_name(raw_disable_result_));
    if (servo_end_attempted_)
      std::printf("STS cleanup servo end: %s\n",
                  esp_err_to_name(servo_end_result_));
    if (power_off_attempted_)
      std::printf("STS cleanup para power OFF: %s\n",
                  esp_err_to_name(power_off_result_));
  }

  [[nodiscard]] STS3215 &servo() { return servo_; }
  [[nodiscard]] int64_t beginLatencyUs() const { return begin_latency_us_; }

private:
  STSCREATE &bus_;
  esp_err_t initialization_result_{ESP_ERR_INVALID_STATE};
  STS3215 servo_;
  int64_t ping_latency_us_{0};
  int64_t begin_latency_us_{0};
  esp_err_t cleanup_result_{ESP_OK};
  esp_err_t disable_result_{ESP_OK};
  esp_err_t raw_disable_result_{ESP_OK};
  esp_err_t servo_end_result_{ESP_OK};
  esp_err_t power_off_result_{ESP_OK};
  uint8_t disable_device_error_{0};
  bool power_attempted_{false};
  bool power_enabled_{false};
  bool closed_{false};
  bool disable_attempted_{false};
  bool raw_disable_attempted_{false};
  bool servo_end_attempted_{false};
  bool power_off_attempted_{false};
};

esp_err_t finish(Session &session, esp_err_t operation) {
  const esp_err_t cleanup = session.close();
  rememberFirst(cleanup, operation);
  // アクチュエータとパラシュート電源を安全状態へ戻してからUSB出力を行う。
  session.printCleanupReport();
  return operation;
}

esp_err_t refreshAndReport(STS3215 &servo) {
  const int64_t before = esp_timer_get_time();
  const esp_err_t result = servo.refreshConfiguration();
  const int64_t latency = esp_timer_get_time() - before;
  std::printf("STS refreshConfiguration: %s latency_us=%" PRId64
              " valid=%d degrees_per_step=%.9f\n",
              esp_err_to_name(result), latency, servo.configurationValid(),
              servo.degreesPerStep());
  return result;
}

esp_err_t requireHealthy(STS3215 &servo) {
  STS3215::Status status{};
  const esp_err_t result = servo.getStatus(status);
  if (result != ESP_OK)
    return result;
  printStatus(status, servo.lastDeviceError());
  if (faulted(status) || servo.lastDeviceError() != 0) {
    std::printf("STS actuator command拒否: servo faultを検出。\n");
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

esp_err_t collectTelemetry(STS3215 &servo, uint32_t samples,
                           bool stop_when_stationary,
                           bool stop_on_health_error,
                           TelemetryReport &report) {
  esp_err_t operation = ESP_OK;
  bool moving_seen = false;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1'000 / kTelemetryRateHz);

  for (uint32_t sample = 0; sample < samples; ++sample) {
    if (report.record_count == report.records.size()) {
      report.buffer_full = true;
      rememberFirst(ESP_ERR_NO_MEM, operation);
      break;
    }
    if (sample != 0)
      vTaskDelayUntil(&last_wake, period);

    TelemetryRecord &record = report.records[report.record_count++];
    record.sample = sample;
    const int64_t before = esp_timer_get_time();
    record.read_result = servo.read(record.data);
    record.latency_us =
        static_cast<uint64_t>(esp_timer_get_time() - before);
    record.device_error = servo.lastDeviceError();
    report.latency_sum_us += record.latency_us;
    if (record.latency_us > report.maximum_latency_us)
      report.maximum_latency_us = record.latency_us;

    if (record.read_result != ESP_OK) {
      ++report.errors;
      if (record.read_result == ESP_ERR_TIMEOUT)
        ++report.timeouts;
      rememberFirst(record.read_result, operation);
      if (stop_on_health_error) {
        // トルク有効中は異常を検出したサンプルで監視を打ち切り、直ちに安全化処理へ進む。
        report.stopped_on_health_error = true;
        break;
      }
      continue;
    }

    record.data_valid = true;
    ++report.successful;
    moving_seen = moving_seen || record.data.moving;
    const bool sensor_fault = faulted(record.data.status);
    const bool device_error = record.device_error != 0;
    if (sensor_fault)
      ++report.fault_samples;
    if (device_error)
      ++report.device_error_samples;
    if (sensor_fault || device_error) {
      rememberFirst(ESP_ERR_INVALID_STATE, operation);
      if (stop_on_health_error) {
        report.stopped_on_health_error = true;
        break;
      }
    }
    if (stop_when_stationary && sample >= 5 && moving_seen &&
        !record.data.moving)
      break;
  }

  return operation;
}

void printTelemetry(const TelemetryReport &report) {
  for (std::size_t index = 0; index < report.record_count; ++index) {
    const TelemetryRecord &record = report.records[index];
    if (!record.data_valid) {
      std::printf("STS sample=%" PRIu32 " read=%s latency_us=%" PRIu64
                  " device_error=0x%02X\n",
                  record.sample, esp_err_to_name(record.read_result),
                  record.latency_us, record.device_error);
      continue;
    }
    printData(record.sample, record.data, record.device_error,
              static_cast<int64_t>(record.latency_us));
  }

  const uint32_t attempts = report.successful + report.errors;
  const double average_latency = attempts == 0
                                     ? 0.0
                                     : static_cast<double>(
                                           report.latency_sum_us) /
                                           attempts;
  std::printf("STS telemetry summary: samples=%" PRIu32
              " errors=%" PRIu32 " timeouts=%" PRIu32
              " fault_samples=%" PRIu32
              " device_error_samples=%" PRIu32
              " response_latency_avg_us=%.2f max_us=%" PRIu64
              " stopped_on_health_error=%d buffer_full=%d\n",
              report.successful, report.errors, report.timeouts,
              report.fault_samples, report.device_error_samples,
              average_latency, report.maximum_latency_us,
              report.stopped_on_health_error, report.buffer_full);
}

} // 無名名前空間

esp_err_t StsBringup::initialize() {
  if (bus_.initialized()) {
    initialization_result_ = ESP_OK;
    return ESP_OK;
  }
  initialization_result_ = bus_.begin(busConfig());
  return initialization_result_;
}

esp_err_t StsBringup::probe() {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

  Session session{bus_, initialization_result_};
  esp_err_t result = session.open();
  std::printf("STS probe begin: %s latency_us=%" PRId64 "\n",
              esp_err_to_name(result), session.beginLatencyUs());
  if (result != ESP_OK)
    return finish(session, result);

  rememberFirst(refreshAndReport(session.servo()), result);
  STS3215::OperatingMode mode{};
  const esp_err_t mode_result = session.servo().getOperatingMode(mode);
  if (mode_result == ESP_OK)
    std::printf("STS operating_mode=%s\n", modeName(mode));
  rememberFirst(mode_result, result);

  STS3215::Status status{};
  const int64_t before = esp_timer_get_time();
  const esp_err_t status_result = session.servo().getStatus(status);
  const int64_t latency = esp_timer_get_time() - before;
  if (status_result == ESP_OK)
    printStatus(status, session.servo().lastDeviceError());
  std::printf("STS getStatus: %s latency_us=%" PRId64 "\n",
              esp_err_to_name(status_result), latency);
  rememberFirst(status_result, result);
  if (status_result == ESP_OK &&
      (faulted(status) || session.servo().lastDeviceError() != 0))
    rememberFirst(ESP_ERR_INVALID_STATE, result);
  return finish(session, result);
}

esp_err_t StsBringup::read() {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

  Session session{bus_, initialization_result_};
  esp_err_t result = session.open();
  std::printf("STS read begin: %s latency_us=%" PRId64 "\n",
              esp_err_to_name(result), session.beginLatencyUs());
  if (result != ESP_OK)
    return finish(session, result);
  rememberFirst(refreshAndReport(session.servo()), result);
  TelemetryReport telemetry{};
  const bool telemetry_collected = result == ESP_OK;
  if (telemetry_collected)
    rememberFirst(collectTelemetry(session.servo(), kTelemetrySamples, false,
                                   false, telemetry),
                  result);
  const esp_err_t finished = finish(session, result);
  if (telemetry_collected)
    printTelemetry(telemetry);
  return finished;
}

esp_err_t StsBringup::free() {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

  Session session{bus_, initialization_result_};
  esp_err_t result = session.open();
  std::printf("STS free begin: %s latency_us=%" PRId64 "\n",
              esp_err_to_name(result), session.beginLatencyUs());
  esp_err_t disable = ESP_ERR_INVALID_STATE;
  uint8_t disable_device_error = 0;
  bool disable_attempted = false;
  if (result == ESP_OK) {
    disable_attempted = true;
    disable = session.servo().disableTorque();
    disable_device_error = session.servo().lastDeviceError();
    rememberFirst(disable, result);
    if (disable == ESP_OK && disable_device_error != 0)
      rememberFirst(ESP_ERR_INVALID_STATE, result);
  }
  const esp_err_t finished = finish(session, result);
  if (disable_attempted)
    std::printf("STS disableTorque: %s device_error=0x%02X。"
                "機械条件が許せば手動可動を確認する。\n",
                esp_err_to_name(disable), disable_device_error);
  return finished;
}

esp_err_t StsBringup::hold() {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

  Session session{bus_, initialization_result_};
  esp_err_t result = session.open();
  std::printf("STS hold begin: %s latency_us=%" PRId64 "\n",
              esp_err_to_name(result), session.beginLatencyUs());
  if (result != ESP_OK)
    return finish(session, result);
  rememberFirst(refreshAndReport(session.servo()), result);
  if (result == ESP_OK)
    rememberFirst(requireHealthy(session.servo()), result);

  STS3215::OperatingMode mode{};
  if (result == ESP_OK)
    rememberFirst(session.servo().getOperatingMode(mode), result);
  if (result == ESP_OK && mode != STS3215::OperatingMode::position &&
      mode != STS3215::OperatingMode::step) {
    std::printf("STS hold拒否: mode=%s。mode設定は変更しない。\n",
                modeName(mode));
    result = ESP_ERR_NOT_SUPPORTED;
  }
  esp_err_t hold_result = ESP_ERR_INVALID_STATE;
  uint8_t hold_device_error = 0;
  bool hold_attempted = false;
  TelemetryReport telemetry{};
  bool telemetry_collected = false;
  if (result == ESP_OK) {
    const STS3215::HoldConfig hold_config{
        STS3215::TorqueLimit::percent(kTorqueLimitPercent)};
    hold_attempted = true;
    hold_result = session.servo().holdCurrentPosition(hold_config);
    hold_device_error = session.servo().lastDeviceError();
    rememberFirst(hold_result, result);
    if (hold_result == ESP_OK && hold_device_error != 0)
      rememberFirst(ESP_ERR_INVALID_STATE, result);
    if (result == ESP_OK) {
      telemetry_collected = true;
      rememberFirst(collectTelemetry(session.servo(), kTelemetrySamples,
                                     false, true, telemetry),
                    result);
    }
  }
  const esp_err_t finished = finish(session, result);
  if (hold_attempted)
    std::printf("STS holdCurrentPosition: %s torque_limit=%.1f%% "
                "device_error=0x%02X\n",
                esp_err_to_name(hold_result), kTorqueLimitPercent,
                hold_device_error);
  if (telemetry_collected)
    printTelemetry(telemetry);
  return finished;
}

esp_err_t StsBringup::smallMove(float delta_degrees) {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  if (!std::isfinite(delta_degrees) || delta_degrees == 0.0F ||
      std::fabs(delta_degrees) > 3.0F)
    return ESP_ERR_INVALID_ARG;

  Session session{bus_, initialization_result_};
  esp_err_t result = session.open();
  std::printf("STS small move begin: %s latency_us=%" PRId64
              " delta_deg=%.3f\n",
              esp_err_to_name(result), session.beginLatencyUs(), delta_degrees);
  if (result != ESP_OK)
    return finish(session, result);
  rememberFirst(refreshAndReport(session.servo()), result);
  if (result == ESP_OK)
    rememberFirst(requireHealthy(session.servo()), result);

  STS3215::OperatingMode mode{};
  if (result == ESP_OK)
    rememberFirst(session.servo().getOperatingMode(mode), result);
  if (result == ESP_OK && mode != STS3215::OperatingMode::position &&
      mode != STS3215::OperatingMode::step) {
    std::printf("STS small move拒否: mode=%s。step modeへ変更しない。\n",
                modeName(mode));
    result = ESP_ERR_NOT_SUPPORTED;
  }

  const STS3215::HoldConfig hold_config{
      STS3215::TorqueLimit::percent(kTorqueLimitPercent)};
  SmallMoveReport move_report{};
  if (result == ESP_OK) {
    move_report.hold_attempted = true;
    move_report.hold_result =
        session.servo().holdCurrentPosition(hold_config);
    move_report.hold_device_error = session.servo().lastDeviceError();
    rememberFirst(move_report.hold_result, result);
    if (move_report.hold_result == ESP_OK &&
        move_report.hold_device_error != 0)
      rememberFirst(ESP_ERR_INVALID_STATE, result);
  }

  STS3215::Motion motion{};
  motion.speed_deg_s = kMoveSpeedDegS;
  motion.acceleration_deg_s2 = kMoveAccelerationDegS2;
  motion.torque_limit = STS3215::TorqueLimit::percent(kTorqueLimitPercent);

  if (result == ESP_OK && mode == STS3215::OperatingMode::step) {
    move_report.move_attempted = true;
    move_report.move_result =
        session.servo().moveRelativeDegrees(delta_degrees, motion);
    move_report.move_device_error = session.servo().lastDeviceError();
    rememberFirst(move_report.move_result, result);
    if (move_report.move_result == ESP_OK &&
        move_report.move_device_error != 0)
      rememberFirst(ESP_ERR_INVALID_STATE, result);
  } else if (result == ESP_OK && mode == STS3215::OperatingMode::position) {
    STS3215::Data current{};
    move_report.position_mode = true;
    move_report.current_read_attempted = true;
    move_report.current_read_result = session.servo().read(current);
    move_report.current_device_error = session.servo().lastDeviceError();
    rememberFirst(move_report.current_read_result, result);
    if (move_report.current_read_result == ESP_OK) {
      move_report.current_degrees = current.position_deg;
      move_report.current_status = current.status;
      if (faulted(current.status) || move_report.current_device_error != 0)
        rememberFirst(ESP_ERR_INVALID_STATE, result);
    }
    if (result == ESP_OK) {
      move_report.target_degrees = current.position_deg + delta_degrees;
      move_report.move_attempted = true;
      move_report.move_result = session.servo().moveAbsoluteDegrees(
          move_report.target_degrees, motion);
      move_report.move_device_error = session.servo().lastDeviceError();
      rememberFirst(move_report.move_result, result);
      if (move_report.move_result == ESP_OK &&
          move_report.move_device_error != 0)
        rememberFirst(ESP_ERR_INVALID_STATE, result);
    }
  }

  TelemetryReport telemetry{};
  const bool telemetry_collected = result == ESP_OK;
  if (telemetry_collected)
    rememberFirst(collectTelemetry(session.servo(), kMoveObservationSamples,
                                   true, true, telemetry),
                  result);
  const esp_err_t finished = finish(session, result);

  if (move_report.hold_attempted)
    std::printf("STS small move hold: %s device_error=0x%02X\n",
                esp_err_to_name(move_report.hold_result),
                move_report.hold_device_error);
  if (move_report.current_read_attempted) {
    std::printf("STS small move current read: %s position_deg=%.3f "
                "device_error=0x%02X\n",
                esp_err_to_name(move_report.current_read_result),
                move_report.current_degrees,
                move_report.current_device_error);
    if (move_report.current_read_result == ESP_OK)
      printStatus(move_report.current_status,
                  move_report.current_device_error);
  }
  if (move_report.move_attempted && move_report.position_mode) {
    std::printf("STS relative move(position): current=%.3f target=%.3f "
                "result=%s device_error=0x%02X"
                "（driverがposition limitを検証）\n",
                move_report.current_degrees, move_report.target_degrees,
                esp_err_to_name(move_report.move_result),
                move_report.move_device_error);
  } else if (move_report.move_attempted) {
    std::printf("STS relative move(step): %s device_error=0x%02X\n",
                esp_err_to_name(move_report.move_result),
                move_report.move_device_error);
  }
  if (telemetry_collected)
    printTelemetry(telemetry);
  return finished;
}

} // 名前空間 bringup
