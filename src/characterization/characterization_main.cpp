#if defined(AVI_99L_CHARACTERIZATION) && AVI_99L_CHARACTERIZATION

#include "actuators/production_motor.hpp"
#include "bringup/power_bringup.hpp"
#include "bringup/safe_outputs.hpp"
#include "characterization/campaign_state_machine.hpp"
#include "characterization/characterization_config.hpp"
#include "characterization/encoder_sampler.hpp"
#include "characterization/log_writer_v5.hpp"
#include "characterization/profile_runner.hpp"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#ifndef AVI_FIRMWARE_GIT_SHA
#define AVI_FIRMWARE_GIT_SHA "0000000000000000000000000000000000000000"
#endif

#ifndef AVI_ESP_LIBS_GIT_SHA
#define AVI_ESP_LIBS_GIT_SHA "0000000000000000000000000000000000000000"
#endif

namespace {

using namespace avi::characterization;

constexpr std::uint32_t kRtcCookie = 0x394C4D30U;
constexpr char kNvsNamespace[] = "char_v5";
constexpr char kNvsHandoffKey[] = "m0_handoff";
constexpr std::size_t kPersistedHandoffBytes =
    4U + kSessionIdBytes + 4U + 1U + 1U + 4U;

RTC_NOINIT_ATTR volatile std::uint32_t rtc_handoff_cookie;

void putU32(std::array<std::uint8_t, kPersistedHandoffBytes> &bytes,
            std::size_t offset, std::uint32_t value) noexcept {
  for (std::size_t index = 0U; index < 4U; ++index)
    bytes[offset + index] =
        static_cast<std::uint8_t>(value >> (index * 8U));
}

std::uint32_t getU32(
    const std::array<std::uint8_t, kPersistedHandoffBytes> &bytes,
    std::size_t offset) noexcept {
  std::uint32_t value = 0U;
  for (std::size_t index = 0U; index < 4U; ++index)
    value |= static_cast<std::uint32_t>(bytes[offset + index]) <<
             (index * 8U);
  return value;
}

std::array<std::uint8_t, kPersistedHandoffBytes>
encodeHandoff(const M0Handoff &handoff) noexcept {
  std::array<std::uint8_t, kPersistedHandoffBytes> bytes{};
  putU32(bytes, 0U, handoff.schema_version);
  std::memcpy(bytes.data() + 4U, handoff.session_id.data(),
              handoff.session_id.size());
  putU32(bytes, 4U + kSessionIdBytes, handoff.profile_seed);
  bytes[8U + kSessionIdBytes] = handoff.completed_stage_mask;
  bytes[9U + kSessionIdBytes] =
      static_cast<std::uint8_t>(handoff.expected_stage);
  putU32(bytes, 10U + kSessionIdBytes, handoff.checksum);
  return bytes;
}

bool decodeHandoff(
    const std::array<std::uint8_t, kPersistedHandoffBytes> &bytes,
    M0Handoff &handoff) noexcept {
  handoff = {};
  handoff.schema_version = getU32(bytes, 0U);
  std::memcpy(handoff.session_id.data(), bytes.data() + 4U,
              handoff.session_id.size());
  handoff.profile_seed = getU32(bytes, 4U + kSessionIdBytes);
  handoff.completed_stage_mask = bytes[8U + kSessionIdBytes];
  handoff.expected_stage =
      static_cast<AssemblyStage>(bytes[9U + kSessionIdBytes]);
  handoff.checksum = getU32(bytes, 10U + kSessionIdBytes);
  return CampaignStateMachine::validateHandoff(handoff);
}

esp_err_t loadHandoff(M0Handoff &handoff, bool &present) noexcept {
  present = false;
  nvs_handle_t handle{};
  esp_err_t result = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    return ESP_OK;
  if (result != ESP_OK)
    return result;

  std::size_t size = 0U;
  result = nvs_get_blob(handle, kNvsHandoffKey, nullptr, &size);
  if (result == ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    return ESP_OK;
  }
  if (result != ESP_OK || size != kPersistedHandoffBytes) {
    nvs_close(handle);
    return result == ESP_OK ? ESP_ERR_INVALID_SIZE : result;
  }

  std::array<std::uint8_t, kPersistedHandoffBytes> bytes{};
  result = nvs_get_blob(handle, kNvsHandoffKey, bytes.data(), &size);
  nvs_close(handle);
  present = result == ESP_OK;
  if (result != ESP_OK)
    return result;
  return decodeHandoff(bytes, handoff) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t persistHandoff(const M0Handoff &handoff) noexcept {
  if (!CampaignStateMachine::validateHandoff(handoff))
    return ESP_ERR_INVALID_ARG;
  const auto bytes = encodeHandoff(handoff);
  nvs_handle_t handle{};
  esp_err_t result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (result == ESP_OK)
    result = nvs_set_blob(handle, kNvsHandoffKey, bytes.data(), bytes.size());
  if (result == ESP_OK)
    result = nvs_commit(handle);
  if (handle != 0U)
    nvs_close(handle);
  return result;
}

esp_err_t eraseHandoff() noexcept {
  nvs_handle_t handle{};
  esp_err_t result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    return ESP_OK;
  if (result != ESP_OK)
    return result;
  result = nvs_erase_key(handle, kNvsHandoffKey);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    result = ESP_OK;
  if (result == ESP_OK)
    result = nvs_commit(handle);
  nvs_close(handle);
  return result;
}

ResetKind resetKind(esp_reset_reason_t reason) noexcept {
  switch (reason) {
  case ESP_RST_POWERON:
    return ResetKind::PowerOn;
  case ESP_RST_SW:
    return ResetKind::Software;
  case ESP_RST_PANIC:
    return ResetKind::Panic;
  case ESP_RST_INT_WDT:
  case ESP_RST_TASK_WDT:
  case ESP_RST_WDT:
    return ResetKind::Watchdog;
  case ESP_RST_DEEPSLEEP:
    return ResetKind::DeepSleep;
  case ESP_RST_BROWNOUT:
    return ResetKind::Brownout;
  default:
    return ResetKind::Unknown;
  }
}

esp_err_t initializeConsole() noexcept {
  if (!usb_serial_jtag_is_driver_installed()) {
    usb_serial_jtag_driver_config_t config{};
    config.tx_buffer_size = 8'192;
    config.rx_buffer_size = 1'024;
    const esp_err_t result = usb_serial_jtag_driver_install(&config);
    if (result != ESP_OK)
      return result;
  }
  usb_serial_jtag_vfs_use_driver();
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
  return ESP_OK;
}

[[noreturn]] void holdSafe() noexcept {
  (void)bringup::safe_outputs::motorCoast();
  (void)bringup::safe_outputs::setAux5v(false);
  (void)bringup::safe_outputs::setParaPower(false);
  for (;;)
    vTaskDelay(pdMS_TO_TICKS(1'000));
}

enum class WorkKind : std::uint8_t {
  None = 0,
  NewSession,
  ConfirmStage,
  CaptureZero,
  RateCheck,
  Arm,
  Disarm,
  RunFull,
  CompleteStage,
  PrepareM0,
  ResumeM0,
};

struct WorkRequest {
  WorkKind kind{WorkKind::None};
  SessionId session_id{};
  AssemblyStage stage{AssemblyStage::None};
  EncoderRate rate{EncoderRate::Hz1000};
  ZeroReferenceKind zero_kind{ZeroReferenceKind::Common};
  std::uint32_t value{0U};
  std::array<char, 16> literal{};
  bool asynchronous{false};
};

struct WorkResult {
  esp_err_t operation_result{ESP_OK};
  CampaignStatus campaign_status{CampaignStatus::Ok};
};

struct StatusSnapshot {
  CampaignState state{CampaignState::Idle};
  AssemblyStage stage{AssemblyStage::None};
  SessionId session_id{};
  std::uint32_t profile_seed{0U};
  std::uint8_t completed_stage_mask{0U};
  std::array<RateResult, 3> rate_results{};
  bool full_1000_completed{false};
  bool armed{false};
  AbortReason last_abort{AbortReason::None};
  bool handoff_present{false};
  esp_err_t handoff_result{ESP_OK};
  CampaignStatus boot_resume_status{CampaignStatus::Ok};
};

const char *workName(WorkKind kind) noexcept {
  switch (kind) {
  case WorkKind::NewSession:
    return "new-session";
  case WorkKind::ConfirmStage:
    return "confirm-stage";
  case WorkKind::CaptureZero:
    return "zero-capture";
  case WorkKind::RateCheck:
    return "rate-check";
  case WorkKind::Arm:
    return "arm";
  case WorkKind::Disarm:
    return "stop";
  case WorkKind::RunFull:
    return "run";
  case WorkKind::CompleteStage:
    return "complete-stage";
  case WorkKind::PrepareM0:
    return "prepare-m0";
  case WorkKind::ResumeM0:
    return "resume-m0";
  case WorkKind::None:
    return "none";
  }
  return "none";
}

class CharacterizationRuntime {
public:
  CharacterizationRuntime() noexcept
      : runner_(campaign_, motor_, sampler_, writer_) {}

  void configureBoot(const M0Handoff &handoff, bool handoff_present,
                     esp_err_t handoff_result, ResetKind reset_kind,
                     bool rtc_cookie_survived) noexcept {
    handoff_present_ = handoff_present;
    handoff_result_ = handoff_result;
    if (handoff_result == ESP_OK && handoff_present)
      boot_resume_status_ =
          campaign_.resumeM0(handoff, reset_kind, rtc_cookie_survived);
  }

  esp_err_t initialize() noexcept {
    const esp_err_t writer_result = writer_.initialize();
    if (writer_result != ESP_OK)
      return writer_result;
    work_queue_ = xQueueCreateStatic(
        1U, sizeof(WorkRequest), work_queue_storage_.data(),
        &work_queue_control_);
    result_ack_ = xSemaphoreCreateBinaryStatic(&result_ack_storage_);
    startup_ack_ = xSemaphoreCreateBinaryStatic(&startup_ack_storage_);
    status_mutex_ = xSemaphoreCreateMutexStatic(&status_mutex_storage_);
    if (work_queue_ == nullptr || result_ack_ == nullptr ||
        startup_ack_ == nullptr || status_mutex_ == nullptr)
      return ESP_ERR_NO_MEM;

    console_task_ = xTaskCreateStaticPinnedToCore(
        consoleEntry, "char_console", sizeof(console_stack_), this, 5,
        console_stack_, &console_task_control_, 0);
    if (console_task_ == nullptr)
      return ESP_ERR_NO_MEM;
    worker_task_ = xTaskCreateStaticPinnedToCore(
        workerEntry, "char_runtime", sizeof(worker_stack_), this, 21,
        worker_stack_, &worker_task_control_, 1);
    if (worker_task_ == nullptr)
      return ESP_ERR_NO_MEM;
    if (xSemaphoreTake(startup_ack_, portMAX_DELAY) != pdTRUE)
      return ESP_FAIL;
    if (startup_result_ != ESP_OK)
      return startup_result_;
    xTaskNotifyGive(console_task_);
    return ESP_OK;
  }

  esp_err_t submit(const WorkRequest &request, WorkResult &result) noexcept {
    bool expected = false;
    if (!work_active_.compare_exchange_strong(expected, true))
      return ESP_ERR_INVALID_STATE;
    while (xSemaphoreTake(result_ack_, 0U) == pdTRUE) {
    }
    if (xQueueSend(work_queue_, &request, 0U) != pdTRUE) {
      work_active_.store(false);
      return ESP_ERR_NO_MEM;
    }
    if (request.asynchronous) {
      result = {};
      return ESP_OK;
    }
    if (xSemaphoreTake(result_ack_, portMAX_DELAY) != pdTRUE)
      return ESP_FAIL;
    result = last_result_;
    return ESP_OK;
  }

  esp_err_t requestStop() noexcept {
    while (work_active_.load()) {
      if (runner_.busy()) {
        runner_.requestStop();
        return ESP_OK;
      }
      vTaskDelay(1U);
    }
    return ESP_ERR_INVALID_STATE;
  }

  StatusSnapshot status() noexcept {
    StatusSnapshot snapshot{};
    if (xSemaphoreTake(status_mutex_, portMAX_DELAY) == pdTRUE) {
      snapshot = status_;
      (void)xSemaphoreGive(status_mutex_);
    } else {
      snapshot.handoff_result = ESP_FAIL;
    }
    return snapshot;
  }

  bool workActive() const noexcept { return work_active_.load(); }
  WorkKind currentWork() const noexcept { return current_work_.load(); }

private:
  static void workerEntry(void *context) noexcept {
    static_cast<CharacterizationRuntime *>(context)->workerLoop();
  }
  static void consoleEntry(void *context) noexcept {
    // 初期化完了通知まではconsoleを公開しない。
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    static_cast<CharacterizationRuntime *>(context)->consoleLoop();
  }

  void publishStatus() noexcept {
    StatusSnapshot next{};
    next.state = campaign_.state();
    next.stage = campaign_.stage();
    next.session_id = campaign_.sessionId();
    next.profile_seed = campaign_.profileSeed();
    next.completed_stage_mask = campaign_.completedStageMask();
    next.rate_results = campaign_.rateResults();
    next.full_1000_completed = campaign_.full1000Completed();
    next.armed = runner_.armed();
    next.last_abort = runner_.lastAbortReason();
    next.handoff_present = handoff_present_;
    next.handoff_result = handoff_result_;
    next.boot_resume_status = boot_resume_status_;
    if (xSemaphoreTake(status_mutex_, portMAX_DELAY) == pdTRUE) {
      status_ = next;
      (void)xSemaphoreGive(status_mutex_);
    }
  }

  WorkResult execute(const WorkRequest &request) noexcept {
    WorkResult result{};
    switch (request.kind) {
    case WorkKind::NewSession:
      if (handoff_present_ || handoff_result_ != ESP_OK) {
        result.operation_result = ESP_ERR_INVALID_STATE;
      } else {
        result.campaign_status =
            campaign_.startNewSession(request.session_id, request.value);
      }
      break;
    case WorkKind::ConfirmStage:
      result.campaign_status = campaign_.confirmStage(request.stage);
      break;
    case WorkKind::CaptureZero:
      result.operation_result = runner_.captureZero(request.zero_kind);
      break;
    case WorkKind::RateCheck:
      result.operation_result =
          runner_.runRateCheck(request.rate, request.value);
      break;
    case WorkKind::Arm:
      result.operation_result = runner_.arm(request.session_id);
      break;
    case WorkKind::Disarm:
      result.operation_result = runner_.disarm();
      break;
    case WorkKind::RunFull:
      result.operation_result = runner_.runFullProfile(request.rate);
      break;
    case WorkKind::CompleteStage: {
      const esp_err_t safe_result = runner_.disarm();
      if (safe_result != ESP_OK) {
        result.operation_result = safe_result;
        break;
      }
      if (campaign_.state() == CampaignState::Completed) {
        if (handoff_present_) {
          result.operation_result = eraseHandoff();
          if (result.operation_result == ESP_OK) {
            handoff_present_ = false;
            handoff_result_ = ESP_OK;
          } else {
            handoff_result_ = result.operation_result;
          }
        } else {
          result.campaign_status = CampaignStatus::InvalidState;
        }
        break;
      }
      const bool completing_m0 = campaign_.stage() == AssemblyStage::M0;
      if (completing_m0) {
        const bool rates_resolved =
            std::all_of(campaign_.rateResults().begin(),
                        campaign_.rateResults().end(), [](RateResult rate) {
                          return rate != RateResult::Pending;
                        });
        if (campaign_.state() != CampaignState::Ready ||
            !campaign_.full1000Completed() || !rates_resolved) {
          result.campaign_status = CampaignStatus::InvalidState;
          break;
        }
        // M0の重複駆動を防ぐため、遷移前に永続handoffを消去する。
        if (handoff_present_) {
          result.operation_result = eraseHandoff();
          if (result.operation_result != ESP_OK) {
            handoff_result_ = result.operation_result;
            break;
          }
          handoff_present_ = false;
          handoff_result_ = ESP_OK;
        }
      }
      result.campaign_status = campaign_.completeStage();
      break;
    }
    case WorkKind::PrepareM0: {
      result.operation_result = runner_.disarm();
      if (result.operation_result != ESP_OK)
        break;
      M0Handoff handoff{};
      result.campaign_status = campaign_.prepareM0(handoff);
      if (result.campaign_status == CampaignStatus::Ok) {
        result.operation_result = persistHandoff(handoff);
        if (result.operation_result == ESP_OK) {
          handoff_present_ = true;
          handoff_result_ = ESP_OK;
          boot_resume_status_ = CampaignStatus::PowerCycleRequired;
        } else {
          handoff_result_ = result.operation_result;
          campaign_.fault();
        }
      }
      break;
    }
    case WorkKind::ResumeM0:
      result.campaign_status =
          campaign_.resumeM0(request.session_id, request.literal.data());
      break;
    case WorkKind::None:
      result.operation_result = ESP_ERR_INVALID_ARG;
      break;
    }
    return result;
  }

  void workerLoop() noexcept {
    startup_result_ = motor_.initialize();
    if (startup_result_ == ESP_OK)
      startup_result_ = bringup::power::initialize();
    if (startup_result_ != ESP_OK)
      (void)motor_.coast();
    publishStatus();
    (void)xSemaphoreGive(startup_ack_);
    if (startup_result_ != ESP_OK)
      vTaskSuspend(nullptr);

    for (;;) {
      WorkRequest request{};
      if (xQueueReceive(work_queue_, &request, portMAX_DELAY) != pdTRUE)
        continue;
      current_work_.store(request.kind);
      last_result_ = execute(request);
      publishStatus();
      current_work_.store(WorkKind::None);
      work_active_.store(false);
      if (request.asynchronous) {
        std::printf("CHAR_%s %s complete esp=%s campaign=%u abort=%u\n",
                    last_result_.operation_result == ESP_OK &&
                            last_result_.campaign_status == CampaignStatus::Ok
                        ? "OK"
                        : "ERROR",
                    workName(request.kind),
                    esp_err_to_name(last_result_.operation_result),
                    static_cast<unsigned>(last_result_.campaign_status),
                    static_cast<unsigned>(status_.last_abort));
      } else {
        (void)xSemaphoreGive(result_ack_);
      }
    }
  }

  void consoleLoop() noexcept;

  CampaignStateMachine campaign_{};
  actuators::ProductionMotorDriver motor_{};
  EncoderSampler sampler_{};
  LogWriterV5 writer_{};
  ProfileRunner runner_;
  bool handoff_present_{false};
  esp_err_t handoff_result_{ESP_OK};
  CampaignStatus boot_resume_status_{CampaignStatus::Ok};

  StaticQueue_t work_queue_control_{};
  std::array<std::uint8_t, sizeof(WorkRequest)> work_queue_storage_{};
  QueueHandle_t work_queue_{nullptr};
  StaticSemaphore_t result_ack_storage_{};
  SemaphoreHandle_t result_ack_{nullptr};
  StaticSemaphore_t startup_ack_storage_{};
  SemaphoreHandle_t startup_ack_{nullptr};
  StaticSemaphore_t status_mutex_storage_{};
  SemaphoreHandle_t status_mutex_{nullptr};
  StaticTask_t worker_task_control_{};
  StackType_t worker_stack_[12'288]{};
  TaskHandle_t worker_task_{nullptr};
  StaticTask_t console_task_control_{};
  StackType_t console_stack_[4'096]{};
  TaskHandle_t console_task_{nullptr};
  WorkResult last_result_{};
  esp_err_t startup_result_{ESP_ERR_NOT_FINISHED};
  StatusSnapshot status_{};
  std::atomic<bool> work_active_{false};
  std::atomic<WorkKind> current_work_{WorkKind::None};
};

bool parsePositiveU32(const char *text, std::uint32_t &value) noexcept {
  if (text == nullptr || text[0] == '\0' || text[0] == '+' ||
      text[0] == '-')
    return false;
  errno = 0;
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(text, &end, 10);
  if (errno != 0 || end == nullptr || *end != '\0' || parsed == 0UL ||
      parsed > std::numeric_limits<std::uint32_t>::max())
    return false;
  value = static_cast<std::uint32_t>(parsed);
  return true;
}

bool parseSessionId(const char *text, SessionId &session_id) noexcept {
  session_id = {};
  if (text == nullptr)
    return false;
  const std::size_t length = std::strlen(text);
  if (length == 0U || length > kSessionIdMaximumPrintableBytes)
    return false;
  std::memcpy(session_id.data(), text, length);
  return CampaignStateMachine::validSessionId(session_id);
}

bool parseRate(const char *text, EncoderRate &rate) noexcept {
  if (text == nullptr)
    return false;
  if (std::strcmp(text, "1000") == 0)
    rate = EncoderRate::Hz1000;
  else if (std::strcmp(text, "2000") == 0)
    rate = EncoderRate::Hz2000;
  else if (std::strcmp(text, "5000") == 0)
    rate = EncoderRate::Hz5000;
  else
    return false;
  return true;
}

bool parseStage(const char *text, AssemblyStage &stage) noexcept {
  if (text == nullptr)
    return false;
  if (std::strcmp(text, "FV") == 0)
    stage = AssemblyStage::FV;
  else if (std::strcmp(text, "FH+") == 0)
    stage = AssemblyStage::FHPositive;
  else if (std::strcmp(text, "FH-") == 0)
    stage = AssemblyStage::FHNegative;
  else
    return false;
  return true;
}

const char *stateName(CampaignState state) noexcept {
  switch (state) {
  case CampaignState::Idle:
    return "idle";
  case CampaignState::AwaitingStageConfirmation:
    return "awaiting-stage-confirmation";
  case CampaignState::Ready:
    return "ready";
  case CampaignState::Running:
    return "running";
  case CampaignState::AwaitingM0Handoff:
    return "awaiting-m0-handoff";
  case CampaignState::PowerCycleRequired:
    return "power-cycle-required";
  case CampaignState::AwaitingM0Confirmation:
    return "awaiting-m0-confirmation";
  case CampaignState::Completed:
    return "completed";
  case CampaignState::Faulted:
    return "faulted";
  }
  return "unknown";
}

const char *stageName(AssemblyStage stage) noexcept {
  switch (stage) {
  case AssemblyStage::None:
    return "none";
  case AssemblyStage::FV:
    return "FV";
  case AssemblyStage::FHPositive:
    return "FH+";
  case AssemblyStage::FHNegative:
    return "FH-";
  case AssemblyStage::M0:
    return "M0";
  }
  return "unknown";
}

const char *rateResultName(RateResult result) noexcept {
  switch (result) {
  case RateResult::Pending:
    return "pending";
  case RateResult::Accepted:
    return "accepted";
  case RateResult::Unsupported:
    return "unsupported";
  }
  return "unknown";
}

void printCommands() noexcept {
  std::printf(
      "CHAR_COMMANDS new-session status confirm-stage zero-capture "
      "rate-check arm run stop complete-stage prepare-m0 resume-m0\n");
  std::printf(
      "CHAR_NOTICE confirm-stage requires <FV|FH+|FH-> "
      "ORIENTATION_CONFIRMED after physical fixture inspection\n");
}

void CharacterizationRuntime::consoleLoop() noexcept {
  std::array<char, 192> line{};
  for (;;) {
    std::size_t line_length = 0U;
    bool invalid_byte = false;
    bool line_too_long = false;
    for (;;) {
      const int value = std::fgetc(stdin);
      if (value == EOF) {
        std::clearerr(stdin);
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }
      if (value == '\n')
        break;
      if (value == '\r')
        continue;
      if (value != '\t' && (value < 0x20 || value > 0x7E)) {
        invalid_byte = true;
        continue;
      }
      if (line_length + 1U >= line.size()) {
        line_too_long = true;
        continue;
      }
      line[line_length++] = static_cast<char>(value);
    }
    line[line_length] = '\0';
    if (invalid_byte || line_too_long) {
      std::printf("CHAR_ERROR parse %s\n",
                  invalid_byte ? "invalid-byte" : "line-too-long");
      continue;
    }

    std::array<char *, 7> arguments{};
    std::size_t count = 0U;
    bool too_many = false;
    char *save = nullptr;
    for (char *token = ::strtok_r(line.data(), " \t\r\n", &save);
         token != nullptr;
         token = ::strtok_r(nullptr, " \t\r\n", &save)) {
      if (count == arguments.size()) {
        too_many = true;
        break;
      }
      arguments[count++] = token;
    }
    if (count == 0U)
      continue;
    if (too_many || count < 2U ||
        std::strcmp(arguments[0], "char") != 0) {
      std::printf("CHAR_ERROR parse expected-char-command\n");
      continue;
    }

    const char *const verb = arguments[1];
    if (std::strcmp(verb, "help") == 0 && count == 2U) {
      std::printf("CHAR_OK help\n");
      printCommands();
      continue;
    }
    if (std::strcmp(verb, "status") == 0 && count == 2U) {
      const StatusSnapshot snapshot = status();
      std::printf(
          "CHAR_OK status state=%s stage=%s session=%s seed=%lu "
          "active=%s operation=%s armed=%s hardware-approved=%s "
          "command-to-fin-sign=%d "
          "mask=0x%02x full1000=%s rates=%s,%s,%s abort=%u "
          "handoff=%s handoff-result=%s boot-resume=%u\n",
          stateName(snapshot.state), stageName(snapshot.stage),
          snapshot.session_id.data(),
          static_cast<unsigned long>(snapshot.profile_seed),
          workActive() ? "true" : "false", workName(currentWork()),
          snapshot.armed ? "true" : "false",
          kHardwareDriveApproved ? "true" : "false",
          static_cast<int>(kCommandToFinSign),
          static_cast<unsigned>(snapshot.completed_stage_mask),
          snapshot.full_1000_completed ? "true" : "false",
          rateResultName(snapshot.rate_results[0]),
          rateResultName(snapshot.rate_results[1]),
          rateResultName(snapshot.rate_results[2]),
          static_cast<unsigned>(snapshot.last_abort),
          snapshot.handoff_present ? "present" : "missing",
          esp_err_to_name(snapshot.handoff_result),
          static_cast<unsigned>(snapshot.boot_resume_status));
      continue;
    }
    if (std::strcmp(verb, "stop") == 0 && count == 2U) {
      esp_err_t result = requestStop();
      WorkResult work_result{};
      if (result == ESP_ERR_INVALID_STATE) {
        WorkRequest disarm{};
        disarm.kind = WorkKind::Disarm;
        result = submit(disarm, work_result);
        if (result == ESP_OK)
          result = work_result.operation_result;
      }
      std::printf("CHAR_%s stop %s\n", result == ESP_OK ? "OK" : "ERROR",
                  esp_err_to_name(result));
      continue;
    }

    WorkRequest request{};
    bool valid = true;
    if (std::strcmp(verb, "new-session") == 0 && count == 4U) {
      request.kind = WorkKind::NewSession;
      valid = parseSessionId(arguments[2], request.session_id) &&
              parsePositiveU32(arguments[3], request.value);
    } else if (std::strcmp(verb, "confirm-stage") == 0 && count == 4U) {
      request.kind = WorkKind::ConfirmStage;
      valid = parseStage(arguments[2], request.stage) &&
              std::strcmp(arguments[3], "ORIENTATION_CONFIRMED") == 0;
    } else if (std::strcmp(verb, "zero-capture") == 0 && count == 3U) {
      request.kind = WorkKind::CaptureZero;
      request.asynchronous = true;
      if (std::strcmp(arguments[2], "common") == 0)
        request.zero_kind = ZeroReferenceKind::Common;
      else if (std::strcmp(arguments[2], "m0") == 0)
        request.zero_kind = ZeroReferenceKind::M0;
      else
        valid = false;
    } else if (std::strcmp(verb, "rate-check") == 0 && count == 4U) {
      request.kind = WorkKind::RateCheck;
      request.asynchronous = true;
      valid = parseRate(arguments[2], request.rate) &&
              parsePositiveU32(arguments[3], request.value);
    } else if (std::strcmp(verb, "arm") == 0 && count == 3U) {
      request.kind = WorkKind::Arm;
      valid = parseSessionId(arguments[2], request.session_id);
    } else if (std::strcmp(verb, "run") == 0 && count == 4U &&
               std::strcmp(arguments[2], "full") == 0) {
      request.kind = WorkKind::RunFull;
      request.asynchronous = true;
      valid = parseRate(arguments[3], request.rate);
    } else if (std::strcmp(verb, "complete-stage") == 0 && count == 2U) {
      request.kind = WorkKind::CompleteStage;
    } else if (std::strcmp(verb, "prepare-m0") == 0 && count == 2U) {
      request.kind = WorkKind::PrepareM0;
    } else if (std::strcmp(verb, "resume-m0") == 0 && count == 4U) {
      request.kind = WorkKind::ResumeM0;
      valid = parseSessionId(arguments[2], request.session_id) &&
              std::strlen(arguments[3]) < request.literal.size();
      if (valid)
        std::memcpy(request.literal.data(), arguments[3],
                    std::strlen(arguments[3]));
    } else {
      valid = false;
    }

    if (!valid || request.kind == WorkKind::None) {
      std::printf("CHAR_ERROR %s invalid-arguments\n", verb);
      continue;
    }
    WorkResult result{};
    const esp_err_t submit_result = submit(request, result);
    if (submit_result != ESP_OK) {
      std::printf("CHAR_ERROR %s %s\n", verb,
                  esp_err_to_name(submit_result));
    } else if (request.asynchronous) {
      std::printf("CHAR_OK %s queued\n", verb);
    } else {
      const bool ok = result.operation_result == ESP_OK &&
                      result.campaign_status == CampaignStatus::Ok;
      std::printf("CHAR_%s %s esp=%s campaign=%u\n", ok ? "OK" : "ERROR",
                  verb, esp_err_to_name(result.operation_result),
                  static_cast<unsigned>(result.campaign_status));
      if (ok && request.kind == WorkKind::PrepareM0)
        std::printf(
            "CHAR_NOTICE power-cycle-required disconnect USB, logic "
            "battery, motor battery, and every external supply; wait "
            "until board and UART are fully unpowered; reset button or "
            "one-source disconnect is invalid; then remove the fin and "
            "inspect the reassembly before cold boot\n");
    }
  }
}

} // 無名名前空間

extern "C" void app_main() {
  const esp_err_t safe_result = bringup::safe_outputs::initialize();
  if (safe_result != ESP_OK)
    holdSafe();
  const esp_err_t console_result = initializeConsole();
  if (console_result != ESP_OK)
    holdSafe();
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  const bool rtc_cookie_survived = rtc_handoff_cookie == kRtcCookie;
  rtc_handoff_cookie = kRtcCookie;
  const esp_reset_reason_t reset_reason = esp_reset_reason();
  const esp_err_t nvs_result = nvs_flash_init();
  M0Handoff handoff{};
  bool handoff_present = false;
  const esp_err_t handoff_result =
      nvs_result == ESP_OK ? loadHandoff(handoff, handoff_present)
                           : nvs_result;

  std::printf(
      "99L characterization schema=%u firmware=%s Avi_ESP_Libs=%s\n",
      static_cast<unsigned>(kSchemaVersion), AVI_FIRMWARE_GIT_SHA,
      AVI_ESP_LIBS_GIT_SHA);
  std::printf("reset=%u rtc_cookie_survived=%s nvs=%s handoff=%s\n",
              static_cast<unsigned>(reset_reason),
              rtc_cookie_survived ? "true" : "false",
              esp_err_to_name(nvs_result), esp_err_to_name(handoff_result));
  if (nvs_result != ESP_OK) {
    std::printf("CHAR_ERROR init nvs=%s\n", esp_err_to_name(nvs_result));
    holdSafe();
  }

  static CharacterizationRuntime runtime;
  runtime.configureBoot(handoff, handoff_present, handoff_result,
                        resetKind(reset_reason), rtc_cookie_survived);
  const esp_err_t runtime_result = runtime.initialize();
  if (runtime_result != ESP_OK) {
    std::printf("CHAR_ERROR init runtime=%s\n",
                esp_err_to_name(runtime_result));
    holdSafe();
  }
  std::printf("safe_outputs=%s runtime=%s hardware-approved=%s "
              "command-to-fin-sign=%d\n",
              esp_err_to_name(safe_result), esp_err_to_name(runtime_result),
              kHardwareDriveApproved ? "true" : "false",
              static_cast<int>(kCommandToFinSign));
  std::printf("motor profile is never started automatically\n");
  printCommands();
  vTaskDelete(nullptr);
}

#endif
