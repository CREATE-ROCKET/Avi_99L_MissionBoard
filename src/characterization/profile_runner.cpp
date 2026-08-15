#include "characterization/profile_runner.hpp"

#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION

#include "characterization/characterization_config.hpp"
#include "characterization/fin_angle.hpp"
#include "characterization/fixed_epoch_assembler.hpp"
#include "characterization/position_guard.hpp"
#include "characterization/record_validation.hpp"
#include "characterization/shutdown_sequence.hpp"
#include "config/board_config.hpp"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#ifndef AVI_FIRMWARE_GIT_SHA
#define AVI_FIRMWARE_GIT_SHA "0000000000000000000000000000000000000000"
#endif

#ifndef AVI_ESP_LIBS_GIT_SHA
#define AVI_ESP_LIBS_GIT_SHA "0000000000000000000000000000000000000000"
#endif

namespace avi::characterization {
namespace {

constexpr std::uint64_t kRunStartLeadUs = 250'000U;
constexpr std::uint32_t kZeroCaptureTimeoutMs = 1'000U;

// 動作確認済みZeroHold（1024 scaleでKp=500, Kd=25）をpermilleへ概算移植する。
// 20 kHzで得たgainそのものを30 kHz flight値とはみなさず、zero approach/recenter専用に使う。
constexpr std::int32_t kApproachKpPermillePerDeg = 500;
constexpr std::int32_t kApproachKdPermillePerDegPerS = 25;
// 30 kHz motor-only実測breakawayが約15%なので、微小0.1 deg stepは短い16% pulseで歯面を動かす。
constexpr std::int16_t kApproachMinimumActivePermille = 160;
constexpr std::int16_t kApproachMaximumPermille = 300;
constexpr std::int32_t kApproachPulseRegionMilliDeg = 250;
constexpr std::uint32_t kApproachPulsePeriodEpochs = 20U;
constexpr std::uint32_t kApproachPulseOnEpochs = 8U;
constexpr std::int32_t kApproachDeadbandMilliDeg = 20;

std::uint64_t nowUs() noexcept {
  return static_cast<std::uint64_t>(
      std::max<std::int64_t>(esp_timer_get_time(), 0));
}

const char *stageName(AssemblyStage stage) noexcept {
  switch (stage) {
  case AssemblyStage::FV:
    return "FV";
  case AssemblyStage::FHPositive:
    return "FHP";
  case AssemblyStage::FHNegative:
    return "FHN";
  case AssemblyStage::M0:
    return "M0";
  case AssemblyStage::None:
    return "NONE";
  }
  return "NONE";
}

const char *completionName(CompletionCode completion) noexcept {
  switch (completion) {
  case CompletionCode::Normal:
    return "normal";
  case CompletionCode::Aborted:
    return "aborted";
  case CompletionCode::Unsupported:
    return "unsupported";
  }
  return "unknown";
}

const char *unsupportedReasonName(UnsupportedReason reason) noexcept {
  switch (reason) {
  case UnsupportedReason::None:
    return "None";
  case UnsupportedReason::TriggerCoalesced:
    return "TriggerCoalesced";
  case UnsupportedReason::IncompleteEpoch:
    return "IncompleteEpoch";
  case UnsupportedReason::InvalidRead:
    return "InvalidRead";
  case UnsupportedReason::DeadlineMiss:
    return "DeadlineMiss";
  case UnsupportedReason::QueueOverflow:
    return "QueueOverflow";
  case UnsupportedReason::WriterFailure:
    return "WriterFailure";
  case UnsupportedReason::SensorHealth:
    return "SensorHealth";
  case UnsupportedReason::OperatorMarkedUnsupported:
    return "OperatorMarkedUnsupported";
  }
  return "Unknown";
}

void rememberOperation(esp_err_t operation, esp_err_t &first) noexcept {
  if (first == ESP_OK && operation != ESP_OK)
    first = operation;
}

UnsupportedReason unsupportedReason(
    const SamplerStatistics &statistics) noexcept {
  if (statistics.trigger_coalesced_or_missed != 0U)
    return UnsupportedReason::TriggerCoalesced;
  // repeated/skippedがstartup epochだけに存在する場合はqualificationを落とさない。
  // steady側の欠落はsteady_state_incomplete_epochsで必ず検出する。
  if (statistics.pre_epoch_samples != 0U ||
      statistics.late_after_release != 0U ||
      statistics.steady_state_incomplete_epochs != 0U)
    return UnsupportedReason::IncompleteEpoch;
  if (statistics.invalid_samples != 0U ||
      statistics.encoder_transport_errors != 0U)
    return UnsupportedReason::InvalidRead;
  if (statistics.consumer_deadline_misses != 0U)
    return UnsupportedReason::DeadlineMiss;
  if (statistics.raw_queue_overflows != 0U ||
      statistics.writer_queue_overflows != 0U ||
      statistics.bucket_collisions != 0U)
    return UnsupportedReason::QueueOverflow;
  if (statistics.encoder_status_faults != 0U)
    return UnsupportedReason::SensorHealth;
  return UnsupportedReason::None;
}

AbortReason abortReasonForAcquisition(
    UnsupportedReason reason) noexcept {
  switch (reason) {
  case UnsupportedReason::DeadlineMiss:
    return AbortReason::Deadline;
  case UnsupportedReason::QueueOverflow:
    return AbortReason::QueueFull;
  case UnsupportedReason::IncompleteEpoch:
    return AbortReason::EncoderInvalid;
  case UnsupportedReason::TriggerCoalesced:
  case UnsupportedReason::InvalidRead:
  case UnsupportedReason::SensorHealth:
    return AbortReason::SamplerError;
  case UnsupportedReason::WriterFailure:
    return AbortReason::WriterError;
  case UnsupportedReason::None:
  case UnsupportedReason::OperatorMarkedUnsupported:
    return AbortReason::ValidationError;
  }
  return AbortReason::ValidationError;
}

void makeSafeSessionToken(const SessionId &session_id,
                          std::array<char, kSessionIdBytes> &token) noexcept {
  token = {};
  for (std::size_t index = 0U; index + 1U < token.size() &&
                              session_id[index] != '\0';
       ++index) {
    const unsigned char value =
        static_cast<unsigned char>(session_id[index]);
    const bool safe = (value >= 'a' && value <= 'z') ||
                      (value >= 'A' && value <= 'Z') ||
                      (value >= '0' && value <= '9') || value == '.' ||
                      value == '_' || value == '-';
    token[index] = safe ? static_cast<char>(value) : '_';
  }
}

MotorCommandRequest driveCommand(std::int16_t command) noexcept {
  if (command > 0)
    return {command, MotorMode::DriveIn2};
  if (command < 0)
    return {command, MotorMode::DriveIn1};
  return {0, MotorMode::Coast};
}

MotorCommandRequest finDirectionCommand(std::int32_t fin_direction,
                                        std::int16_t magnitude) noexcept {
  if (magnitude <= 0 || (fin_direction != -1 && fin_direction != 1))
    return {0, MotorMode::Coast};
  const std::int16_t command =
      commandForFinError(fin_direction, magnitude, kCommandToFinSign);
  return driveCommand(command);
}

MotorCommandRequest positionApproachCommand(
    std::int32_t error_millideg, std::int32_t fin_rate_millideg_s,
    std::uint32_t local_epoch, std::int16_t requested_limit) noexcept {
  const std::int64_t absolute_error =
      std::llabs(static_cast<long long>(error_millideg));
  if (absolute_error <= kApproachDeadbandMilliDeg)
    return {0, MotorMode::Coast};

  const std::int32_t direction = error_millideg > 0 ? 1 : -1;
  const std::int64_t rate_toward_target =
      static_cast<std::int64_t>(direction) * fin_rate_millideg_s;
  const std::int64_t proportional =
      absolute_error * kApproachKpPermillePerDeg / 1'000;
  const std::int64_t damping =
      rate_toward_target > 0
          ? rate_toward_target * kApproachKdPermillePerDegPerS / 1'000
          : 0;
  if (damping >= proportional && rate_toward_target > 0)
    return {0, MotorMode::Coast};

  const std::int16_t limit = static_cast<std::int16_t>(std::clamp<int>(
      requested_limit, kApproachMinimumActivePermille,
      kApproachMaximumPermille));
  const std::int64_t undamped =
      std::max<std::int64_t>(1, proportional - damping);
  const std::int16_t magnitude = static_cast<std::int16_t>(
      std::clamp<std::int64_t>(undamped, kApproachMinimumActivePermille,
                               limit));

  // 0.1 deg刻み付近で連続16%以上を入れて通り過ぎないよう、近傍だけpulse化する。
  if (absolute_error <= kApproachPulseRegionMilliDeg &&
      (local_epoch % kApproachPulsePeriodEpochs) >= kApproachPulseOnEpochs)
    return {0, MotorMode::Coast};
  return finDirectionCommand(direction, magnitude);
}

} // 無名名前空間

ProfileRunner::ProfileRunner(CampaignStateMachine &campaign,
                             actuators::ProductionMotorDriver &motor,
                             EncoderSampler &sampler,
                             LogWriterV5 &writer) noexcept
    : campaign_(campaign), motor_(motor), sampler_(sampler), writer_(writer),
      journal_(motorApply, &motor_) {}

bool ProfileRunner::consumerTimerCallback(
    gptimer_handle_t, const gptimer_alarm_event_data_t *, void *context) {
  auto &runner = *static_cast<ProfileRunner *>(context);
  // GPTimer ISRでは固定lifetimeのconsumer taskへ通知するだけに留める。
  // SAFETY: consumerはchar_runtime taskでrun中に削除されず、ISRではI/Oを行わない。
  const TaskHandle_t consumer =
      runner.consumer_task_.load(std::memory_order_acquire);
  if (!runner.consumer_timer_running_.load(std::memory_order_acquire) ||
      consumer == nullptr)
    return false;
  BaseType_t task_awoken = pdFALSE;
  vTaskNotifyGiveFromISR(consumer, &task_awoken);
  return task_awoken == pdTRUE;
}

esp_err_t ProfileRunner::startConsumerTimer(
    std::uint64_t epoch_zero_us) {
  if (consumer_timer_running_.load() ||
      epoch_zero_us >
          std::numeric_limits<std::uint64_t>::max() - kEpochDurationUs)
    return ESP_ERR_INVALID_STATE;
  const TaskHandle_t consumer = xTaskGetCurrentTaskHandle();
  if (consumer == nullptr)
    return ESP_ERR_INVALID_STATE;
  consumer_task_.store(consumer);
  writer_.setFailureNotificationTask(consumer);
  sampler_.setFailureNotificationTask(consumer);
  power_sampler_.setFailureNotificationTask(consumer);

  if (!consumer_timer_.initialized()) {
    const esp_err_t initialize_result =
        consumer_timer_.initialize(consumerTimerCallback, this);
    if (initialize_result != ESP_OK) {
      (void)stopConsumerTimer();
      return initialize_result;
    }
  }

  (void)xTaskNotifyStateClear(consumer);
  (void)ulTaskNotifyValueClear(consumer,
                               std::numeric_limits<std::uint32_t>::max());

  // first alarmをepoch 0終端の絶対時刻へ設定し、その後はhardware auto-reloadで
  // 固定1 ms周期を維持する。timer arm後のtask schedulingはalarm位相へ影響しない。
  consumer_timer_running_.store(true, std::memory_order_release);
  const esp_err_t result = consumer_timer_.start(
      epoch_zero_us + kEpochDurationUs, kEpochDurationUs);
  if (result != ESP_OK) {
    consumer_timer_running_.store(false, std::memory_order_release);
    (void)stopConsumerTimer();
    return result;
  }

  // epoch 0のcommand処理は従来どおりepoch zeroから開始する。
  while (nowUs() + 2'000U < epoch_zero_us)
    vTaskDelay(1U);
  while (nowUs() < epoch_zero_us)
    taskYIELD();
  return ESP_OK;
}

esp_err_t ProfileRunner::stopConsumerTimer() noexcept {
  consumer_timer_running_.store(false, std::memory_order_release);
  const esp_err_t result = consumer_timer_.stop();
  writer_.setFailureNotificationTask(nullptr);
  sampler_.setFailureNotificationTask(nullptr);
  power_sampler_.setFailureNotificationTask(nullptr);
  consumer_task_.store(nullptr, std::memory_order_release);
  return result;
}

esp_err_t ProfileRunner::motorApply(
    void *context, const MotorCommandRequest &request,
    std::uint64_t &completed_at_us) {
  auto &motor =
      *static_cast<actuators::ProductionMotorDriver *>(context);
  esp_err_t result = ESP_ERR_INVALID_ARG;
  switch (request.mode) {
  case MotorMode::Coast:
    result = motor.coast();
    break;
  case MotorMode::Brake:
    result = motor.brake();
    break;
  case MotorMode::DriveIn1:
  case MotorMode::DriveIn2: {
    control::MotorCommand command{};
    command.pwm_duty =
        std::abs(static_cast<double>(request.command_permille)) / 1'000.0;
    command.positive_in1 = request.mode == MotorMode::DriveIn1;
    command.brake = false;
    command.valid = true;
    result = motor.apply(command);
    break;
  }
  }
  completed_at_us = nowUs();
  return result;
}

esp_err_t ProfileRunner::arm(const SessionId &session_id) noexcept {
  if (!kHardwareDriveApproved || kCommandToFinSign == 0)
    return ESP_ERR_NOT_SUPPORTED;
  if (busy_.load() || session_id != campaign_.sessionId() ||
      !campaign_.canArmMotor())
    return ESP_ERR_INVALID_STATE;
  const bool zero_valid = campaign_.stage() == AssemblyStage::M0
                              ? m0_zero_valid_
                              : common_zero_valid_;
  if (!zero_valid)
    return ESP_ERR_INVALID_STATE;
  return journal_.arm();
}

esp_err_t ProfileRunner::disarm() noexcept {
  return journal_.disarm(nowUs());
}

void ProfileRunner::requestStop() noexcept {
  stop_requested_.store(true);
#if defined(ESP_PLATFORM)
  const TaskHandle_t consumer = consumer_task_.load();
  if (consumer != nullptr)
    xTaskNotifyGive(consumer);
#endif
}

esp_err_t
ProfileRunner::captureZero(ZeroReferenceKind reference_kind) noexcept {
  if (journal_.armed() || !campaign_.canArmMotor())
    return ESP_ERR_INVALID_STATE;
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;
  if (stop_requested_.exchange(false)) {
    busy_.store(false);
    return ESP_ERR_INVALID_STATE;
  }
  const bool correct_kind =
      (campaign_.stage() == AssemblyStage::M0 &&
       reference_kind == ZeroReferenceKind::M0) ||
      (campaign_.stage() != AssemblyStage::M0 &&
       reference_kind == ZeroReferenceKind::Common);
  if (!correct_kind) {
    busy_.store(false);
    return ESP_ERR_INVALID_ARG;
  }
  if ((reference_kind == ZeroReferenceKind::M0 && m0_zero_valid_) ||
      (reference_kind == ZeroReferenceKind::Common &&
       common_zero_valid_)) {
    busy_.store(false);
    return ESP_ERR_INVALID_STATE;
  }

  const std::uint64_t epoch_zero = nowUs() + 50'000U;
  esp_err_t result = sampler_.begin(EncoderRate::Hz1000, epoch_zero);
  RawEncoderSample captured{};
  const std::uint64_t deadline =
      nowUs() + static_cast<std::uint64_t>(kZeroCaptureTimeoutMs) * 1'000U;
  while (result == ESP_OK && nowUs() < deadline) {
    if (stop_requested_.exchange(false)) {
      result = ESP_ERR_INVALID_STATE;
      break;
    }
    if (sampler_.pop(captured) && captured.valid) {
      if (captured.angle_raw > 0x3FFFU)
        result = ESP_ERR_INVALID_RESPONSE;
      break;
    }
    if (sampler_.firstError() != ESP_OK) {
      result = sampler_.firstError();
      break;
    }
    vTaskDelay(1U);
  }
  if (!captured.valid && result == ESP_OK)
    result = ESP_ERR_TIMEOUT;
  rememberOperation(sampler_.stop(), result);
  if (stop_requested_.exchange(false) && result == ESP_OK)
    result = ESP_ERR_INVALID_STATE;
  if (result == ESP_OK) {
    if (reference_kind == ZeroReferenceKind::M0) {
      m0_zero_raw_ = captured.angle_raw;
      m0_zero_valid_ = true;
    } else {
      common_zero_raw_ = captured.angle_raw;
      common_zero_valid_ = true;
    }
  }
  busy_.store(false);
  return result;
}

LogHeaderV5 ProfileRunner::makeHeader(
    EncoderRate rate, RunKind run_kind,
    std::uint64_t epoch_zero_us) const {
  LogHeaderV5 header{};
  header.stage = campaign_.stage();
  header.encoder_rate = rate;
  header.run_kind = run_kind;
  header.profile_seed = campaign_.profileSeed();
  header.pwm_frequency_hz = board::kMotorPwmFrequencyHz;
  header.reset_reason = static_cast<std::uint32_t>(esp_reset_reason());
  header.epoch_zero_timestamp_us = epoch_zero_us;
  header.session_id = campaign_.sessionId();
  static_assert(sizeof(AVI_FIRMWARE_GIT_SHA) == 41U);
  static_assert(sizeof(AVI_ESP_LIBS_GIT_SHA) == 41U);
  std::memcpy(header.firmware_sha.data(), AVI_FIRMWARE_GIT_SHA,
              header.firmware_sha.size());
  std::memcpy(header.avi_esp_libs_sha.data(), AVI_ESP_LIBS_GIT_SHA,
              header.avi_esp_libs_sha.size());
  (void)std::snprintf(
      header.board_build_id.data(), header.board_build_id.size(),
      "avi99l-char-excite2-hw-approved=%u-sign=%+d",
      kHardwareDriveApproved ? 1U : 0U,
      static_cast<int>(kCommandToFinSign));
  return header;
}

MotorCommandRequest ProfileRunner::commandForEpisode(
    const ProfileEpisode &episode, std::uint32_t local_epoch,
    std::int32_t fin_angle_millideg) {
  const auto drive = [](std::int16_t command) {
    return driveCommand(command);
  };
  const std::int16_t limit = episode.command_limit_permille;
  switch (episode.phase) {
  case ProfilePhase::StationaryBaseline:
    return {0, MotorMode::Coast};
  case ProfilePhase::PolarityCheck:
    if (local_epoch < 100U)
      return drive(limit);
    if (local_epoch < 400U)
      return {0, MotorMode::Coast};
    if (local_epoch < 500U)
      return drive(static_cast<std::int16_t>(-limit));
    return {0, MotorMode::Coast};
  case ProfilePhase::BreakawaySweep: {
    constexpr std::array<std::int16_t, 9> levels{
        100, 125, 150, 175, 200, 225, 250, 275, 300};
    const std::size_t index = local_epoch / 400U;
    if (index >= levels.size())
      return {0, MotorMode::Coast};
    const std::int16_t magnitude = std::min(levels[index], limit);
    const std::uint32_t phase = local_epoch % 400U;
    if (phase < 80U)
      return drive(magnitude);
    if (phase < 200U)
      return {0, MotorMode::Coast};
    if (phase < 280U)
      return drive(static_cast<std::int16_t>(-magnitude));
    return {0, MotorMode::Coast};
  }
  case ProfilePhase::SustainedMotionSweep: {
    constexpr std::array<std::int16_t, 9> levels{
        300, 275, 250, 225, 200, 175, 150, 125, 100};
    const std::size_t index = local_epoch / 320U;
    if (index >= levels.size())
      return {0, MotorMode::Coast};
    const std::int16_t test = std::min(levels[index], limit);
    const std::uint32_t phase = local_epoch % 320U;
    if (phase < 60U)
      return drive(limit);
    if (phase < 140U)
      return drive(test);
    if (phase < 160U)
      return {0, MotorMode::Coast};
    if (phase < 220U)
      return drive(static_cast<std::int16_t>(-limit));
    if (phase < 300U)
      return drive(static_cast<std::int16_t>(-test));
    return {0, MotorMode::Coast};
  }
  case ProfilePhase::BoundedPulseGrid: {
    constexpr std::array<std::int16_t, 5> levels{100, 150, 200, 250, 300};
    const std::size_t level_index = local_epoch / 800U;
    if (level_index >= levels.size())
      return {0, MotorMode::Coast};
    const std::int16_t magnitude = std::min(levels[level_index], limit);
    const std::uint32_t phase = local_epoch % 400U;
    if (phase < 60U)
      return drive(magnitude);
    if (phase < 200U)
      return {0, MotorMode::Coast};
    if (phase < 260U)
      return drive(static_cast<std::int16_t>(-magnitude));
    return {0, MotorMode::Coast};
  }
  case ProfilePhase::Coast:
    if (local_epoch < 100U)
      return drive(limit);
    if (local_epoch < 1'100U)
      return {0, MotorMode::Coast};
    if (local_epoch < 1'200U)
      return drive(static_cast<std::int16_t>(-limit));
    return {0, MotorMode::Coast};
  case ProfilePhase::ShortBrake:
    if (local_epoch < 100U)
      return drive(limit);
    if (local_epoch < 350U)
      return {0, MotorMode::Brake};
    if (local_epoch < 1'000U)
      return {0, MotorMode::Coast};
    if (local_epoch < 1'100U)
      return drive(static_cast<std::int16_t>(-limit));
    if (local_epoch < 1'350U)
      return {0, MotorMode::Brake};
    return {0, MotorMode::Coast};
  case ProfilePhase::PositiveToNegative:
    if (local_epoch < 100U)
      return drive(limit);
    if (local_epoch < 200U)
      return drive(static_cast<std::int16_t>(-limit));
    return {0, MotorMode::Coast};
  case ProfilePhase::NegativeToPositive:
    if (local_epoch < 100U)
      return drive(static_cast<std::int16_t>(-limit));
    if (local_epoch < 200U)
      return drive(limit);
    return {0, MotorMode::Coast};
  case ProfilePhase::BoundedPrbs:
    if (local_epoch % 20U == 0U)
      pseudo_random_state_ =
          (pseudo_random_state_ >> 1U) ^
          (0x80200003U &
           (0U - (pseudo_random_state_ & 1U)));
    return drive((pseudo_random_state_ & 1U) != 0U ? limit : -limit);
  case ProfilePhase::BandLimitedNoise: {
    if (local_epoch % 50U == 0U)
      pseudo_random_state_ =
          pseudo_random_state_ * 1'664'525U + 1'013'904'223U;
    const std::uint32_t span =
        static_cast<std::uint32_t>(2 * static_cast<int>(limit) + 1);
    const std::int32_t value =
        static_cast<std::int32_t>(pseudo_random_state_ % span) - limit;
    return drive(static_cast<std::int16_t>(value));
  }
  case ProfilePhase::Chirp: {
    const std::uint32_t half_period =
        std::max<std::uint32_t>(
            10U, 100U - 90U * local_epoch / episode.duration_epochs);
    return drive(((local_epoch / half_period) & 1U) == 0U ? limit : -limit);
  }
  case ProfilePhase::Recenter:
    return positionApproachCommand(-fin_angle_millideg, 0, local_epoch,
                                   limit);
  case ProfilePhase::ZeroApproach:
  case ProfilePhase::Idle:
    return {0, MotorMode::Coast};
  }
  return {0, MotorMode::Coast};
}

esp_err_t ProfileRunner::runRateCheck(EncoderRate rate,
                                      std::uint32_t seconds) {
  if (seconds == 0U || journal_.armed() ||
      seconds > std::numeric_limits<std::uint32_t>::max() /
                    kConsumerRateHz)
    return ESP_ERR_INVALID_ARG;
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;
  if (stop_requested_.exchange(false)) {
    const esp_err_t stop_result =
        journal_.armed() ? journal_.disarm(nowUs()) : ESP_ERR_INVALID_STATE;
    busy_.store(false);
    return stop_result;
  }
  const CampaignStatus begin =
      campaign_.beginRun(rate, RunKind::RateCheck);
  esp_err_t result =
      begin == CampaignStatus::Ok
          ? run(rate, RunKind::RateCheck, seconds * kConsumerRateHz)
          : ESP_ERR_INVALID_STATE;
  busy_.store(false);
  return result;
}

esp_err_t ProfileRunner::runFullProfile(EncoderRate rate) {
  if (!journal_.armed())
    return ESP_ERR_INVALID_STATE;
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true))
    return ESP_ERR_INVALID_STATE;
  if (stop_requested_.exchange(false)) {
    const esp_err_t stop_result = journal_.disarm(nowUs());
    busy_.store(false);
    return stop_result == ESP_OK ? ESP_ERR_INVALID_STATE : stop_result;
  }
  const CampaignStatus begin = campaign_.beginRun(rate, RunKind::Full);
  if (begin != CampaignStatus::Ok) {
    busy_.store(false);
    return ESP_ERR_INVALID_STATE;
  }
  const ProfilePlan plan(campaign_.stage(), rate, campaign_.profileSeed());
  std::uint64_t total = 0U;
  for (const ProfileEpisode &episode : plan.commonEpisodes())
    total += episode.duration_epochs;
  esp_err_t result =
      total <= std::numeric_limits<std::uint32_t>::max()
          ? run(rate, RunKind::Full, static_cast<std::uint32_t>(total))
          : ESP_ERR_INVALID_SIZE;
  if (total > std::numeric_limits<std::uint32_t>::max())
    (void)campaign_.finishRun(RunOutcome::Failed);
  busy_.store(false);
  return result;
}

esp_err_t ProfileRunner::run(EncoderRate rate, RunKind run_kind,
                             std::uint32_t epochs) {
  const bool full = run_kind == RunKind::Full;
  const bool zero_valid = campaign_.stage() == AssemblyStage::M0
                              ? m0_zero_valid_
                              : common_zero_valid_;
  if (!zero_valid || epochs == 0U) {
    (void)campaign_.finishRun(RunOutcome::Failed);
    return ESP_ERR_INVALID_STATE;
  }
  const std::uint16_t zero_raw =
      campaign_.stage() == AssemblyStage::M0 ? m0_zero_raw_
                                             : common_zero_raw_;
  const ProfilePlan plan(campaign_.stage(), rate, campaign_.profileSeed());
  if (!plan.valid()) {
    (void)campaign_.finishRun(RunOutcome::Failed);
    return ESP_ERR_INVALID_ARG;
  }

  last_abort_reason_ = AbortReason::None;
  pseudo_random_state_ = campaign_.profileSeed();
  const std::uint64_t epoch_zero = nowUs() + kRunStartLeadUs;
  const LogHeaderV5 header = makeHeader(rate, run_kind, epoch_zero);
  char base_name[96]{};
  std::array<char, kSessionIdBytes> session_token{};
  makeSafeSessionToken(campaign_.sessionId(), session_token);
  const int name_length = std::snprintf(
      base_name, sizeof(base_name), "%s_%s_%u_%s",
      session_token.data(), stageName(campaign_.stage()),
      static_cast<unsigned>(rate),
      run_kind == RunKind::RateCheck ? "rate" : "full");
  if (name_length <= 0 ||
      static_cast<std::size_t>(name_length) >= sizeof(base_name)) {
    (void)campaign_.finishRun(RunOutcome::Failed);
    return ESP_ERR_INVALID_SIZE;
  }

  esp_err_t first_error = writer_.open(header, base_name);
  if (first_error != ESP_OK) {
    (void)journal_.disarm(nowUs());
    (void)campaign_.finishRun(RunOutcome::Failed);
    return first_error;
  }
  first_error = power_sampler_.begin();
  if (first_error != ESP_OK) {
    (void)journal_.apply({0, MotorMode::Coast}, nowUs());
    (void)journal_.disarm(nowUs());
    LogFooterV5 footer{};
    footer.statistics.first_error = first_error;
    (void)writer_.abortAndClose(footer);
    (void)campaign_.finishRun(RunOutcome::Failed);
    return first_error;
  }
  first_error = sampler_.begin(rate, epoch_zero);
  if (first_error != ESP_OK) {
    (void)journal_.apply({0, MotorMode::Coast}, nowUs());
    (void)journal_.disarm(nowUs());
    (void)sampler_.stop();
    (void)power_sampler_.stop();
    LogFooterV5 footer{};
    footer.statistics.first_error = first_error;
    (void)writer_.abortAndClose(footer);
    (void)campaign_.finishRun(RunOutcome::Failed);
    return first_error;
  }

  // epoch zeroより十分前に安全なCoastを一度だけ実機へ確定する。
  // 以後同じcommandはCommandJournalがdedupし、実際の最終apply時刻を保持する。
  first_error = journal_.apply({0, MotorMode::Coast}, nowUs());
  if (first_error != ESP_OK) {
    (void)journal_.disarm(nowUs());
    (void)sampler_.stop();
    (void)power_sampler_.stop();
    LogFooterV5 footer{};
    footer.statistics.first_error = first_error;
    (void)writer_.abortAndClose(footer);
    (void)campaign_.finishRun(RunOutcome::Failed);
    return first_error;
  }

  FixedEpochAssembler assembler(rate, kConsumerDeadlineBudgetUs);
  assembler.reset(epoch_zero, zero_raw);
  PositionGuard guard(
      {kRoutineEnvelopeMilliDeg, kHardAbortMilliDeg, kCommandToFinSign});
  ZeroApproachController positive(ApproachBranch::FromPositive);
  ZeroApproachController negative(ApproachBranch::FromNegative);
  bool positive_started = false;
  bool negative_started = false;
  bool have_angle = false;
  std::int32_t fin_angle = 0;
  std::int32_t previous_fin_angle = 0;
  std::int32_t fin_rate = 0;
  std::uint64_t sequence = 0U;
  std::uint32_t qualification_valid = 0U;
  std::uint32_t qualification_total = 0U;
  std::uint64_t vbus_invalid = 0U;
  std::uint32_t episode_offset = 0U;
  std::size_t episode_index = 0U;
  ShutdownSequence shutdown{};
  bool qualification_stop = false;
  std::uint64_t runtime_deadline_misses = 0U;
  std::uint64_t command_deadline_misses = 0U;
  std::uint64_t release_deadline_misses = 0U;
  std::uint64_t consumer_schedule_misses = 0U;

  const auto stopForFatal = [&](esp_err_t cause, AbortReason reason,
                                std::uint64_t timestamp_us) noexcept {
    if (cause == ESP_OK)
      cause = ESP_ERR_INVALID_RESPONSE;
    // 安全停止は毎回試すが、診断原因とabort理由は最初の値を保持する。
    (void)journal_.stopForError(cause, timestamp_us);
    if (first_error == ESP_OK) {
      first_error = cause;
      last_abort_reason_ = reason;
    }
  };

  const auto stopForQualification = [&]() noexcept {
    qualification_stop = true;
    (void)journal_.disarm(nowUs());
  };

  const auto stopForLatchedFailure = [&]() noexcept {
    const esp_err_t writer_error = writer_.firstError();
    if (writer_error != ESP_OK) {
      stopForFatal(writer_error, AbortReason::WriterError, nowUs());
      return true;
    }
    const esp_err_t power_error = power_sampler_.firstError();
    if (power_error != ESP_OK) {
      ++vbus_invalid;
      stopForFatal(power_error, AbortReason::VbusInvalid, nowUs());
      return true;
    }
    const esp_err_t sampler_error = sampler_.firstError();
    if (sampler_error == ESP_OK)
      return false;
    if (!full) {
      stopForQualification();
      return true;
    }
    stopForFatal(sampler_error, AbortReason::SamplerError, nowUs());
    return true;
  };

  first_error = startConsumerTimer(epoch_zero);
  if (first_error != ESP_OK) {
    (void)journal_.apply({0, MotorMode::Coast}, nowUs());
    (void)journal_.disarm(nowUs());
    (void)stopConsumerTimer();
    (void)sampler_.stop();
    (void)power_sampler_.stop();
    LogFooterV5 footer{};
    footer.statistics.first_error = first_error;
    (void)writer_.abortAndClose(footer);
    (void)campaign_.finishRun(RunOutcome::Failed);
    return first_error;
  }

  for (std::uint32_t epoch = 0U; epoch < epochs; ++epoch) {
    if (stopForLatchedFailure())
      break;
    const std::uint64_t epoch_start =
        epoch_zero + static_cast<std::uint64_t>(epoch) * kEpochDurationUs;
    const std::uint64_t apply_deadline =
        epoch_start + kConsumerDeadlineBudgetUs;
    ProfileEpisode episode{ProfilePhase::StationaryBaseline,
                           ApproachBranch::None, 1U, epochs, 0, false};
    std::uint32_t local_epoch = epoch;
    if (full) {
      while (episode_index + 1U < plan.commonEpisodes().size() &&
             epoch >= episode_offset +
                          plan.commonEpisodes()[episode_index]
                              .duration_epochs) {
        episode_offset +=
            plan.commonEpisodes()[episode_index].duration_epochs;
        ++episode_index;
      }
      episode = plan.commonEpisodes()[episode_index];
      local_epoch = epoch - episode_offset;
    }

    MotorCommandRequest request =
        commandForEpisode(episode, local_epoch, fin_angle);
    std::int32_t target_fin_angle = 0;
    ZeroApproachController *approach = nullptr;
    if (episode.phase == ProfilePhase::ZeroApproach) {
      if (episode.branch == ApproachBranch::FromPositive) {
        approach = &positive;
        if (!positive_started) {
          positive.reset(nowUs());
          positive_started = true;
        }
      } else {
        approach = &negative;
        if (!negative_started) {
          negative.reset(nowUs());
          negative_started = true;
        }
      }
      target_fin_angle = approach->targetMilliDeg();
      if (!approach->complete()) {
        const std::int64_t error =
            static_cast<std::int64_t>(target_fin_angle) - fin_angle;
        if (error > std::numeric_limits<std::int32_t>::max() ||
            error < std::numeric_limits<std::int32_t>::min()) {
          request = {0, MotorMode::Coast};
        } else {
          request = positionApproachCommand(
              static_cast<std::int32_t>(error), fin_rate, local_epoch,
              episode.command_limit_permille);
        }
      }
    }

    GuardState guard_state = GuardState::Allow;
    esp_err_t preapply_error = ESP_OK;
    AbortReason preapply_abort = AbortReason::None;
    if (stop_requested_.load()) {
      request = {0, MotorMode::Coast};
      preapply_error = ESP_ERR_INVALID_STATE;
      preapply_abort = AbortReason::StopRequested;
    } else if (full && !have_angle &&
               request.command_permille != 0) {
      request = {0, MotorMode::Coast};
      guard_state = GuardState::Abort;
      preapply_error = ESP_ERR_INVALID_RESPONSE;
      preapply_abort = AbortReason::EncoderInvalid;
    } else if (have_angle) {
      PositionGuardInput input{};
      input.fin_angle_millideg = fin_angle;
      input.fin_rate_millideg_s = fin_rate;
      input.predicted_stopping_delta_millideg =
          fin_rate / kPredictedStoppingRateDivisor;
      input.requested = request;
      input.encoder_valid = true;
      input.consumer_deadline_met = true;
      const PositionGuardDecision decision = guard.evaluate(input);
      guard_state = decision.action;
      request = decision.permitted;
      if (decision.action == GuardState::Abort) {
        preapply_error = ESP_ERR_INVALID_STATE;
        preapply_abort = AbortReason::PositionGuard;
      }
    }

    // 100 us deadlineは「このepochでhardware stateを変える必要がある場合」だけ
    // 評価する。継続中のCoast/Drive/Brakeは過去の実apply時刻をそのまま保持する。
    const bool command_change_required =
        !journal_.requestAlreadyApplied(request);
    const ImmutableCommandEvidence before_command = journal_.snapshot(nowUs());
    const std::uint64_t apply_started_us = nowUs();
    bool apply_deadline_missed =
        command_change_required && apply_started_us > apply_deadline;
    esp_err_t apply_result = ESP_OK;
    if (apply_deadline_missed) {
      ++runtime_deadline_misses;
      ++command_deadline_misses;
      guard_state = GuardState::Abort;
      // 遅れたDrive/Brakeを実機へ出さない。requestedは元指令のまま残し、
      // applied=Coast/result=timeoutとして一つのgenerationへ記録する。
      apply_result = journal_.rejectAndCoast(
          request, ESP_ERR_TIMEOUT, apply_started_us);
    } else {
      apply_result = journal_.apply(request, apply_started_us);
    }
    ImmutableCommandEvidence epoch_command = journal_.snapshot(nowUs());
    const bool command_generation_changed =
        epoch_command.command_generation != before_command.command_generation;
    const std::uint64_t apply_completed_us =
        epoch_command.command_apply_timestamp_us;
    if (!apply_deadline_missed && command_generation_changed &&
        apply_completed_us > apply_deadline) {
      apply_deadline_missed = true;
      ++runtime_deadline_misses;
      ++command_deadline_misses;
      guard_state = GuardState::Abort;
    }
    // command generation/requested/applied/mode/apply timestampは同じrealtime ownerで
    // 一度だけ値copyし、writer taskはこのimmutable evidenceしか見ない。
    epoch_command.logger_snapshot_timestamp_us =
        std::max(nowUs(), apply_completed_us);
    if (preapply_error != ESP_OK)
      stopForFatal(preapply_error, preapply_abort, nowUs());
    if (apply_deadline_missed) {
      if (full)
        stopForFatal(ESP_ERR_TIMEOUT, AbortReason::Deadline, nowUs());
      else
        stopForQualification();
    } else if (apply_result != ESP_OK) {
      stopForFatal(apply_result, AbortReason::MotorApplyError, nowUs());
    }

    // 先行異常とcommand deadlineが競合したepochは証拠を混同せず終了する。
    if (preapply_error != ESP_OK && apply_deadline_missed)
      break;

    const std::uint64_t epoch_end =
        epoch_zero + (static_cast<std::uint64_t>(epoch) + 1U) *
                         kEpochDurationUs;
    const std::uint32_t consumer_notifications =
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (stopForLatchedFailure())
      break;
    const std::uint64_t release_us = nowUs();
    if (stop_requested_.load() &&
        last_abort_reason_ == AbortReason::None) {
      stopForFatal(ESP_ERR_INVALID_STATE, AbortReason::StopRequested,
                   release_us);
    }
    if (last_abort_reason_ == AbortReason::StopRequested &&
        release_us < epoch_end)
      break;
    const bool release_deadline_missed =
        release_us > epoch_end + kConsumerDeadlineBudgetUs;
    if (release_deadline_missed) {
      ++release_deadline_misses;
      if (!apply_deadline_missed)
        ++runtime_deadline_misses;
      if (full)
        stopForFatal(ESP_ERR_TIMEOUT, AbortReason::Deadline, release_us);
      else
        stopForQualification();
    }
    const bool consumer_schedule_invalid =
        !release_deadline_missed &&
        (consumer_notifications != 1U || release_us < epoch_end ||
         release_us >= epoch_end + kEpochDurationUs);
    if (consumer_schedule_invalid) {
      ++consumer_schedule_misses;
      if (!apply_deadline_missed)
        ++runtime_deadline_misses;
      if (full)
        stopForFatal(ESP_ERR_TIMEOUT, AbortReason::Deadline, release_us);
      else
        stopForQualification();
    }

    std::uint64_t snapshot_us = nowUs();
    PowerEvidence power_evidence{};
    bool power_measurement_valid =
        power_sampler_.latest(snapshot_us, power_evidence) &&
        power_sampler_.firstError() == ESP_OK;
    snapshot_us = nowUs();
    if (power_evidence.capture_timestamp_us > snapshot_us ||
        (power_evidence.capture_timestamp_us != 0U &&
         snapshot_us - power_evidence.capture_timestamp_us > 100'000U))
      power_measurement_valid = false;
    // rate-checkはmotor電源を物理遮断した状態でも実施するため0 Vを許容する。
    // fullだけは実駆動が必要なので、ADC計測が有効でも0 mVならVbusInvalidとする。
    const bool motor_power_present = power_evidence.motor_millivolts > 0U;
    const bool power_valid =
        power_measurement_valid && (!full || motor_power_present);
    epoch_command.logger_snapshot_timestamp_us = snapshot_us;
    if (writer_.firstError() != ESP_OK) {
      stopForFatal(writer_.firstError(), AbortReason::WriterError,
                   release_us);
      if (release_us < epoch_end)
        break;
    }
    if (!power_valid) {
      ++vbus_invalid;
      esp_err_t power_error = power_evidence.read_result == ESP_OK
                                  ? power_sampler_.firstError()
                                  : power_evidence.read_result;
      if (power_error == ESP_OK)
        power_error = ESP_ERR_INVALID_RESPONSE;
      stopForFatal(power_error, AbortReason::VbusInvalid, snapshot_us);
    }

    RawEncoderSample raw{};
    while (sampler_.pop(raw)) {
      const AddResult added = assembler.add(raw);
      if (added == AddResult::BucketCollision) {
        if (full)
          stopForFatal(ESP_ERR_NO_MEM, AbortReason::QueueFull, nowUs());
        else
          stopForQualification();
        break;
      }
    }
    EncoderEpochBlock block{};
    if (!assembler.release(epoch, release_us, block)) {
      stopForFatal(ESP_ERR_INVALID_SIZE, AbortReason::ValidationError,
                   nowUs());
      break;
    }
    if (apply_deadline_missed)
      block.flags = static_cast<std::uint16_t>(block.flags | EpochDeadline);

    if ((block.flags & EpochAggregateValid) != 0U) {
      std::int32_t converted_angle = 0;
      if (!finAngleMilliDegreesFromUnwrappedQ16(
              block.mean_unwrapped_counts_q16, zero_raw,
              converted_angle)) {
        stopForFatal(ESP_ERR_INVALID_SIZE, AbortReason::PositionGuard,
                     nowUs());
        guard_state = GuardState::Abort;
      } else {
        previous_fin_angle = fin_angle;
        fin_angle = converted_angle;
        const std::int64_t rate =
            have_angle
                ? static_cast<std::int64_t>(fin_angle -
                                            previous_fin_angle) *
                      1'000
                : 0;
        if (rate < std::numeric_limits<std::int32_t>::min() ||
            rate > std::numeric_limits<std::int32_t>::max()) {
          stopForFatal(ESP_ERR_INVALID_SIZE, AbortReason::PositionGuard,
                       nowUs());
          guard_state = GuardState::Abort;
        } else {
          fin_rate = static_cast<std::int32_t>(rate);
          have_angle = true;
        }
      }
    } else if (full && first_error == ESP_OK &&
               !((block.flags & EpochStartup) != 0U &&
                 block.epoch_index == 0U &&
                 block.invalid_sample_count == 0U &&
                 (block.flags & EpochDeadline) == 0U)) {
      // startup epochだけはrepeated/skippedを許容する。steady-state欠落は即停止。
      stopForFatal(ESP_ERR_INVALID_RESPONSE, AbortReason::EncoderInvalid,
                   nowUs());
    }
    if ((block.flags & EpochDeadline) != 0U && full) {
      stopForFatal(ESP_ERR_TIMEOUT, AbortReason::Deadline, nowUs());
    } else if ((block.flags & EpochDeadline) != 0U && !full) {
      stopForQualification();
    }
    if (sampler_.firstError() != ESP_OK) {
      if (full) {
        stopForFatal(sampler_.firstError(), AbortReason::SamplerError,
                     nowUs());
      } else {
        stopForQualification();
      }
    }

    if (approach != nullptr && have_angle &&
        last_abort_reason_ == AbortReason::None) {
      const ApproachUpdate update =
          approach->update(fin_angle, fin_rate, release_us);
      if (update == ApproachUpdate::overshoot_abort) {
        stopForFatal(ESP_ERR_INVALID_RESPONSE, AbortReason::Overshoot,
                     nowUs());
      } else if (update == ApproachUpdate::timeout_abort) {
        stopForFatal(ESP_ERR_TIMEOUT, AbortReason::Timeout, nowUs());
      } else if (update == ApproachUpdate::monotonicity_abort ||
                 update == ApproachUpdate::configuration_abort) {
        stopForFatal(ESP_ERR_INVALID_STATE, AbortReason::ValidationError,
                     nowUs());
      }
    }
    if (approach != nullptr &&
        local_epoch + 1U >= episode.duration_epochs &&
        !approach->complete() && first_error == ESP_OK) {
      stopForFatal(ESP_ERR_TIMEOUT, AbortReason::Timeout, nowUs());
    }

    if ((first_error != ESP_OK || qualification_stop) &&
        journal_.armed())
      (void)journal_.disarm(nowUs());

    ImmutableLogRecord record{};
    record.sequence = sequence++;
    record.stage = campaign_.stage();
    record.encoder_rate = rate;
    record.profile_phase = episode.phase;
    record.approach_branch = episode.approach_branch;
    record.zero_reference_kind = plan.zeroReferenceKind();
    record.run_kind = run_kind;
    record.qualification = full ? 1U : 0U;
    record.episode_index = episode.episode_index;
    record.target_fin_angle_millideg = target_fin_angle;
    record.fin_angle_millideg = fin_angle;
    record.fin_rate_millideg_s = fin_rate;
    record.guard_state = guard_state;
    record.abort_reason = last_abort_reason_;
    record.first_error = first_error;
    record.command = epoch_command;
    record.power = power_evidence;
    record.power.valid = power_valid;
    if (!power_valid && record.power.read_result == ESP_OK)
      record.power.read_result = ESP_ERR_INVALID_RESPONSE;
    record.encoder = block;

    if (hasError(validateRecord(record))) {
      stopForFatal(ESP_ERR_INVALID_RESPONSE, AbortReason::ValidationError,
                   nowUs());
    } else {
      const esp_err_t enqueue_result = writer_.enqueue(record);
      if (enqueue_result != ESP_OK) {
        stopForFatal(enqueue_result,
                     enqueue_result == ESP_ERR_NO_MEM
                         ? AbortReason::QueueFull
                         : AbortReason::WriterError,
                     nowUs());
      } else if (enqueue_result == ESP_OK) {
        ++qualification_total;
        if ((block.flags & EpochAggregateValid) != 0U)
          ++qualification_valid;
      }
    }
    if (writer_.firstError() != ESP_OK)
      stopForFatal(writer_.firstError(), AbortReason::WriterError, nowUs());
    if (first_error != ESP_OK || qualification_stop)
      break;
  }

  std::uint32_t wire_shutdown_mask = 0U;
  bool shutdown_prefix_intact = true;
  const auto markShutdown = [&](esp_err_t operation) {
    if (shutdown_prefix_intact && operation == ESP_OK)
      wire_shutdown_mask = (wire_shutdown_mask << 1U) | 1U;
    else
      shutdown_prefix_intact = false;
    rememberOperation(operation, first_error);
  };

  const esp_err_t coast_result =
      journal_.apply({0, MotorMode::Coast}, nowUs());
  markShutdown(coast_result);
  (void)shutdown.mark(ShutdownStep::Coast);
  const esp_err_t disarm_result = journal_.disarm(nowUs());
  markShutdown(disarm_result);
  (void)shutdown.mark(ShutdownStep::Disarm);
  esp_err_t sampling_cleanup_result = stopConsumerTimer();
  const esp_err_t sampler_diagnostic_result = sampler_.stop();
  const esp_err_t power_diagnostic_result = power_sampler_.stop();
  rememberOperation(sampler_.stopCleanupError(), sampling_cleanup_result);
  markShutdown(sampling_cleanup_result);
  (void)shutdown.mark(ShutdownStep::SamplingStop);
  RawEncoderSample discarded{};
  while (sampler_.pop(discarded)) {
  }
  markShutdown(ESP_OK);
  (void)shutdown.mark(ShutdownStep::RealtimeQueueDrain);

  SamplerStatistics statistics = sampler_.statistics();
  const SamplerStatistics &epoch_statistics = assembler.statistics();
  statistics.pre_epoch_samples = epoch_statistics.pre_epoch_samples;
  statistics.repeated_samples = epoch_statistics.repeated_samples;
  statistics.skipped_samples = epoch_statistics.skipped_samples;
  statistics.invalid_samples = epoch_statistics.invalid_samples;
  statistics.late_after_release = epoch_statistics.late_after_release;
  statistics.startup_incomplete_epochs =
      epoch_statistics.startup_incomplete_epochs;
  statistics.steady_state_incomplete_epochs =
      epoch_statistics.steady_state_incomplete_epochs;
  statistics.consumer_deadline_misses = runtime_deadline_misses;
  statistics.writer_queue_overflows = writer_.writerQueueOverflows();
  statistics.vbus_invalid_samples = vbus_invalid;
  const UnsupportedReason acquisition_reason = unsupportedReason(statistics);
  if (power_diagnostic_result != ESP_OK && first_error == ESP_OK) {
    first_error = power_diagnostic_result;
    last_abort_reason_ = AbortReason::VbusInvalid;
  }
  if (full && acquisition_reason != UnsupportedReason::None &&
      first_error == ESP_OK) {
    first_error = sampler_diagnostic_result == ESP_OK
                      ? ESP_ERR_INVALID_RESPONSE
                      : sampler_diagnostic_result;
    last_abort_reason_ = abortReasonForAcquisition(acquisition_reason);
  } else if (!full && acquisition_reason == UnsupportedReason::None &&
             sampler_diagnostic_result != ESP_OK && first_error == ESP_OK) {
    first_error = sampler_diagnostic_result;
    last_abort_reason_ = AbortReason::SamplerError;
  }
  if (qualification_stop && acquisition_reason == UnsupportedReason::None &&
      first_error == ESP_OK) {
    first_error = ESP_ERR_INVALID_RESPONSE;
    last_abort_reason_ = AbortReason::ValidationError;
  }

  const esp_err_t writer_sync_result = writer_.drainAndSync();
  if (writer_sync_result == ESP_OK) {
    markShutdown(ESP_OK);
    markShutdown(ESP_OK);
  } else {
    markShutdown(writer_sync_result);
    shutdown_prefix_intact = false;
  }
  (void)shutdown.mark(ShutdownStep::WriterQueueDrain);
  (void)shutdown.mark(ShutdownStep::Sync);
  statistics.first_error = first_error;
  const UnsupportedReason unsupported =
      full ? UnsupportedReason::None : acquisition_reason;
  const bool supported = first_error == ESP_OK &&
                         unsupported == UnsupportedReason::None;
  LogFooterV5 footer{};
  footer.completion =
      first_error != ESP_OK
          ? CompletionCode::Aborted
          : (supported ? CompletionCode::Normal
                       : CompletionCode::Unsupported);
  footer.rate_supported = supported;
  footer.unsupported_reason =
      footer.completion == CompletionCode::Unsupported
          ? unsupported
          : UnsupportedReason::None;
  footer.statistics = statistics;
  footer.qualification_valid_epochs =
      run_kind == RunKind::RateCheck ? qualification_valid : 0U;
  footer.qualification_total_epochs =
      run_kind == RunKind::RateCheck ? qualification_total : 0U;
  // wire上はcoast/disarm/sampling/realtime-drain/writer-drain/syncの6段階。
  footer.shutdown_step_mask = wire_shutdown_mask;
  (void)shutdown.mark(ShutdownStep::Footer);
  const esp_err_t close_result = writer_.close(footer);
  rememberOperation(close_result, first_error);
  (void)shutdown.mark(ShutdownStep::Close);

  if (!full) {
    const CompletionCode reported_completion =
        first_error != ESP_OK
            ? CompletionCode::Aborted
            : (supported ? CompletionCode::Normal
                         : CompletionCode::Unsupported);
    const UnsupportedReason reported_reason =
        reported_completion == CompletionCode::Unsupported
            ? unsupported
            : UnsupportedReason::None;
    std::printf(
        "CHAR_RATE_RESULT rate=%u classification=%s reason=%s "
        "valid=%u total=%u trigger-coalesced=%llu pre-epoch=%llu "
        "repeated=%llu skipped=%llu invalid=%llu late=%llu "
        "startup-incomplete=%llu steady-incomplete=%llu deadline=%llu "
        "command-deadline=%llu release-deadline=%llu schedule-miss=%llu "
        "raw-queue-overflow=%llu writer-queue-overflow=%llu "
        "encoder-transport=%llu encoder-status=%llu vbus-invalid=%llu "
        "first-error=%s\n",
        static_cast<unsigned>(rate), completionName(reported_completion),
        unsupportedReasonName(reported_reason),
        static_cast<unsigned>(qualification_valid),
        static_cast<unsigned>(qualification_total),
        static_cast<unsigned long long>(
            statistics.trigger_coalesced_or_missed),
        static_cast<unsigned long long>(statistics.pre_epoch_samples),
        static_cast<unsigned long long>(statistics.repeated_samples),
        static_cast<unsigned long long>(statistics.skipped_samples),
        static_cast<unsigned long long>(statistics.invalid_samples),
        static_cast<unsigned long long>(statistics.late_after_release),
        static_cast<unsigned long long>(statistics.startup_incomplete_epochs),
        static_cast<unsigned long long>(
            statistics.steady_state_incomplete_epochs),
        static_cast<unsigned long long>(statistics.consumer_deadline_misses),
        static_cast<unsigned long long>(command_deadline_misses),
        static_cast<unsigned long long>(release_deadline_misses),
        static_cast<unsigned long long>(consumer_schedule_misses),
        static_cast<unsigned long long>(statistics.raw_queue_overflows),
        static_cast<unsigned long long>(statistics.writer_queue_overflows),
        static_cast<unsigned long long>(statistics.encoder_transport_errors),
        static_cast<unsigned long long>(statistics.encoder_status_faults),
        static_cast<unsigned long long>(statistics.vbus_invalid_samples),
        esp_err_to_name(first_error));
  }

  CampaignStatus finish_status = CampaignStatus::Ok;
  if (first_error != ESP_OK)
    finish_status = campaign_.finishRun(RunOutcome::Failed);
  else if (!supported)
    finish_status =
        campaign_.finishRun(RunOutcome::Unsupported, unsupported);
  else
    finish_status = campaign_.finishRun(RunOutcome::Accepted);
  if (finish_status != CampaignStatus::Ok && first_error == ESP_OK)
    first_error = ESP_ERR_INVALID_STATE;
  return first_error;
}

} // 名前空間 avi::characterization

#endif
