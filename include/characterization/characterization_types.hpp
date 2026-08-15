#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace avi::characterization {

constexpr std::uint16_t kSchemaVersion = 5U;
constexpr std::uint32_t kProfileContractVersion = 1U;
constexpr std::uint32_t kConsumerRateHz = 1'000U;
constexpr std::uint64_t kEpochDurationUs = 1'000U;
constexpr std::uint32_t kConsumerDeadlineBudgetUs = 100U;
constexpr std::size_t kMaximumEncoderSamplesPerEpoch = 5U;
constexpr std::size_t kSessionIdBytes = 32U;
constexpr std::size_t kSessionIdMaximumPrintableBytes = 31U;
constexpr float kTotalReduction = 176.175F;
constexpr float kFinMechanicalLimitDeg = 15.0F;
constexpr float kRoutineEnvelopeDeg = 8.0F;
constexpr float kHardAbortDeg = 10.0F;
constexpr std::int32_t kFinMechanicalLimitMilliDeg = 15'000;
constexpr std::int32_t kRoutineEnvelopeMilliDeg = 8'000;
constexpr std::int32_t kHardAbortMilliDeg = 10'000;
constexpr float kBacklashFullWidthDeg = 0.344F;
constexpr std::int32_t kApproachStepMilliDeg = 100;
constexpr std::uint16_t kRawDiagnosticMask = 0x007FU;
constexpr std::int16_t kMaximumCommandPermille = 150;

using SessionId = std::array<char, kSessionIdBytes>;

enum class AssemblyStage : std::uint8_t {
  None = 0,
  FV = 1,
  FHPositive = 2,
  FHNegative = 3,
  M0 = 4,
  none = None,
  fv = FV,
  fh_positive = FHPositive,
  fh_negative = FHNegative,
  m0 = M0,
};

enum class CampaignState : std::uint8_t {
  Idle = 0,
  AwaitingStageConfirmation = 1,
  Ready = 2,
  Running = 3,
  AwaitingM0Handoff = 4,
  PowerCycleRequired = 5,
  AwaitingM0Confirmation = 6,
  Completed = 7,
  Faulted = 8,
  idle = Idle,
  awaiting_stage_confirmation = AwaitingStageConfirmation,
  ready = Ready,
  running = Running,
  awaiting_m0_handoff = AwaitingM0Handoff,
  power_cycle_required = PowerCycleRequired,
  completed = Completed,
  faulted = Faulted,
};

enum class EncoderRate : std::uint32_t {
  Hz1000 = 1'000U,
  Hz2000 = 2'000U,
  Hz5000 = 5'000U,
  hz_1000 = Hz1000,
  hz_2000 = Hz2000,
  hz_5000 = Hz5000,
};

enum class MotorMode : std::uint8_t {
  Coast = 0,
  DriveIn1 = 1,
  DriveIn2 = 2,
  Brake = 3,
  coast = Coast,
  drive_in1 = DriveIn1,
  drive_in2 = DriveIn2,
  brake = Brake,
};

enum class ProfilePhase : std::uint8_t {
  Idle = 0,
  StationaryBaseline = 1,
  ZeroApproach = 2,
  PolarityCheck = 3,
  BreakawaySweep = 4,
  SustainedMotionSweep = 5,
  BoundedPulseGrid = 6,
  Coast = 7,
  ShortBrake = 8,
  PositiveToNegative = 9,
  NegativeToPositive = 10,
  BoundedPrbs = 11,
  BandLimitedNoise = 12,
  Chirp = 13,
  Recenter = 14,
};

enum class ApproachBranch : std::uint8_t {
  None = 0,
  FromPositive = 1,
  FromNegative = 2,
  none = None,
  from_positive = FromPositive,
  from_negative = FromNegative,
};

enum class GuardState : std::uint8_t {
  Allow = 0,
  Coast = 1,
  Abort = 2,
  allow = Allow,
  force_coast = Coast,
  abort_run = Abort,
};
using PositionGuardAction = GuardState;

enum class PositionGuardReason : std::uint8_t {
  None = 0,
  SoftLimitOutward = 1,
  HardLimit = 2,
  PredictedStopExceedsLimit = 3,
  EncoderInvalid = 4,
  ConsumerDeadlineMiss = 5,
  CommandModeMismatch = 6,
  PolarityUnconfigured = 7,
  ConfigurationInvalid = 8,
  none = None,
  soft_limit_outward = SoftLimitOutward,
  hard_limit = HardLimit,
  predicted_stop_exceeds_limit = PredictedStopExceedsLimit,
  encoder_invalid = EncoderInvalid,
  consumer_deadline_miss = ConsumerDeadlineMiss,
  command_mode_mismatch = CommandModeMismatch,
  polarity_unconfigured = PolarityUnconfigured,
  configuration_invalid = ConfigurationInvalid,
};

enum class AbortReason : std::uint16_t {
  None = 0,
  StopRequested = 1,
  EncoderInvalid = 2,
  PositionGuard = 3,
  Overshoot = 4,
  Timeout = 5,
  Deadline = 6,
  QueueFull = 7,
  WriterError = 8,
  MotorApplyError = 9,
  VbusInvalid = 10,
  SamplerError = 11,
  ValidationError = 12,
  StageError = 13,
};

enum class RunKind : std::uint8_t { None = 0, RateCheck = 1, Full = 2 };
enum class CompletionCode : std::uint8_t {
  Normal = 1,
  Aborted = 2,
  Unsupported = 3,
};
enum class UnsupportedReason : std::uint8_t {
  None = 0,
  TriggerCoalesced = 1,
  IncompleteEpoch = 2,
  InvalidRead = 3,
  DeadlineMiss = 4,
  QueueOverflow = 5,
  WriterFailure = 6,
  SensorHealth = 7,
  OperatorMarkedUnsupported = 8,
};

enum class ResetKind : std::uint8_t {
  PowerOn = 0,
  Software = 1,
  Watchdog = 2,
  Panic = 3,
  Brownout = 4,
  DeepSleep = 5,
  Unknown = 255,
  power_on = PowerOn,
  software = Software,
  watchdog = Watchdog,
  panic = Panic,
  brownout = Brownout,
  unknown = Unknown,
};

enum class ZeroReferenceKind : std::uint8_t {
  Common = 0,
  M0 = 1,
  full_fin_common = Common,
  m0_local = M0,
};

enum class CampaignStatus : std::uint8_t {
  Ok = 0,
  InvalidState = 1,
  InvalidArgument = 2,
  WrongStage = 3,
  WrongRateOrder = 4,
  MandatoryRateUnsupported = 5,
  PowerCycleRequired = 6,
  InvalidHandoff = 7,
  QueueFull = 8,
  SnapshotInvalid = 9,
  ColdPowerCycleNotProven = 10,
  FinRemovalConfirmationRequired = 11,
  SessionIdMismatch = 12,
  RateNotQualified = 13,
  ok = Ok,
  invalid_state = InvalidState,
  invalid_argument = InvalidArgument,
  wrong_stage = WrongStage,
  wrong_rate_order = WrongRateOrder,
  mandatory_rate_cannot_be_skipped = MandatoryRateUnsupported,
  power_cycle_required = PowerCycleRequired,
  invalid_handoff = InvalidHandoff,
  queue_full = QueueFull,
  snapshot_invalid = SnapshotInvalid,
  cold_power_cycle_not_proven = ColdPowerCycleNotProven,
  fin_removal_confirmation_required = FinRemovalConfirmationRequired,
  session_id_mismatch = SessionIdMismatch,
  rate_not_qualified = RateNotQualified,
};

enum class RunOutcome : std::uint8_t {
  Accepted = 0,
  Failed = 1,
  Unsupported = 2,
  accepted = Accepted,
  failed_retryable = Failed,
  unsupported = Unsupported,
};
enum class RateResult : std::uint8_t {
  Pending = 0,
  Accepted = 1,
  Unsupported = 2,
  pending = Pending,
  accepted = Accepted,
  unsupported = Unsupported,
};

enum class AddResult : std::uint8_t {
  Accepted = 0,
  PreEpoch = 1,
  AlreadyReleased = 2,
  BucketCollision = 3,
  accepted = Accepted,
  pre_epoch = PreEpoch,
  already_released = AlreadyReleased,
  bucket_collision = BucketCollision,
};
using AddSampleResult = AddResult;

enum EpochFlags : std::uint16_t {
  EpochNone = 0U,
  EpochIncomplete = 1U << 0U,
  EpochRepeated = 1U << 1U,
  EpochSkipped = 1U << 2U,
  EpochInvalid = 1U << 3U,
  EpochLate = 1U << 4U,
  EpochDeadline = 1U << 5U,
  EpochAggregateValid = 1U << 6U,
  EpochStartup = 1U << 7U,
  epoch_none = EpochNone,
  epoch_incomplete = EpochIncomplete,
  epoch_repeated_sample = EpochRepeated,
  epoch_skipped_sample = EpochSkipped,
  epoch_invalid_sample = EpochInvalid,
  epoch_late_after_release = EpochLate,
  epoch_consumer_deadline_miss = EpochDeadline,
  epoch_aggregate_valid = EpochAggregateValid,
  epoch_startup = EpochStartup,
};

enum RawDiagnosticFlags : std::uint16_t {
  DiagnosticParityError = 1U << 0U,
  DiagnosticInvalidCommand = 1U << 1U,
  DiagnosticFramingError = 1U << 2U,
  DiagnosticMagneticTooLow = 1U << 3U,
  DiagnosticMagneticTooHigh = 1U << 4U,
  DiagnosticCordicOverflow = 1U << 5U,
  DiagnosticOffsetCompensationFinished = 1U << 6U,
};

struct MotorCommandRequest {
  std::int16_t command_permille{0};
  union {
    MotorMode mode{MotorMode::Coast};
    MotorMode requested_mode;
  };
};
using MotorRequest = MotorCommandRequest;

struct MotorCommandApplied {
  union {
    std::int16_t command_permille{0};
    std::int16_t applied_command_permille;
  };
  union {
    MotorMode mode{MotorMode::Coast};
    MotorMode applied_mode;
  };
  union {
    std::int32_t result{0};
    std::int32_t result_code;
  };
  std::uint64_t command_apply_timestamp_us{0};
};
using MotorApplyResult = MotorCommandApplied;

struct ImmutableCommandEvidence {
  std::uint64_t command_generation{0};
  std::int16_t requested_command_permille{0};
  MotorMode requested_motor_mode{MotorMode::Coast};
  std::int16_t applied_command_permille{0};
  MotorMode applied_motor_mode{MotorMode::Coast};
  std::int32_t apply_result_code{0};
  std::uint64_t command_apply_timestamp_us{0};
  std::uint64_t logger_snapshot_timestamp_us{0};
};
using CommandSnapshot = ImmutableCommandEvidence;

struct RawEncoderSample {
  union {
    std::uint64_t generation{0};
    std::uint64_t sample_generation;
  };
  std::uint64_t scheduled_timestamp_us{0};
  std::uint64_t capture_timestamp_us{0};
  std::uint16_t angle_raw{0};
  union {
    std::uint16_t diagnostic_flags{0};
    std::uint16_t status_flags;
  };
  std::int32_t read_result_code{0};
  bool valid{false};
  std::uint8_t slot{0};
};
using EncoderRawSample = RawEncoderSample;

struct EncoderEpochBlock {
  std::uint64_t epoch_index{0};
  std::uint64_t epoch_start_timestamp_us{0};
  std::uint64_t epoch_end_timestamp_us{0};
  std::uint64_t release_timestamp_us{0};
  std::int32_t consumer_lateness_us{0};
  std::uint8_t expected_sample_count{0};
  std::uint8_t observed_sample_count{0};
  std::uint8_t actual_sample_count{0};
  std::uint8_t selected_sample_count{0};
  std::uint8_t valid_sample_count{0};
  std::uint8_t repeated_sample_count{0};
  std::uint8_t skipped_sample_count{0};
  std::uint8_t invalid_sample_count{0};
  std::uint16_t flags{EpochNone};
  std::int64_t mean_unwrapped_counts_q16{0};
  std::array<RawEncoderSample, kMaximumEncoderSamplesPerEpoch> samples{};
  std::array<bool, kMaximumEncoderSamplesPerEpoch> sample_present{};
};

struct PowerEvidence {
  std::uint64_t capture_timestamp_us{0};
  std::uint16_t motor_millivolts{0};
  std::int32_t read_result{0};
  bool valid{false};
};

struct ImmutableLogRecord {
  std::uint64_t sequence{0};
  AssemblyStage stage{AssemblyStage::None};
  EncoderRate encoder_rate{EncoderRate::Hz1000};
  ProfilePhase profile_phase{ProfilePhase::Idle};
  ApproachBranch approach_branch{ApproachBranch::None};
  ZeroReferenceKind zero_reference_kind{ZeroReferenceKind::Common};
  RunKind run_kind{RunKind::None};
  std::uint8_t qualification{0};
  std::uint32_t episode_index{0};
  std::int32_t target_fin_angle_millideg{0};
  std::int32_t fin_angle_millideg{0};
  std::int32_t fin_rate_millideg_s{0};
  GuardState guard_state{GuardState::Allow};
  AbortReason abort_reason{AbortReason::None};
  std::int32_t first_error{0};
  ImmutableCommandEvidence command{};
  PowerEvidence power{};
  EncoderEpochBlock encoder{};
};
using CharacterizationRecord = ImmutableLogRecord;

struct LogHeaderV5 {
  AssemblyStage stage{AssemblyStage::None};
  EncoderRate encoder_rate{EncoderRate::Hz1000};
  RunKind run_kind{RunKind::None};
  std::uint32_t profile_seed{0};
  std::uint32_t pwm_frequency_hz{0};
  std::uint32_t reset_reason{0};
  float total_reduction{kTotalReduction};
  float physical_limit_deg{kFinMechanicalLimitDeg};
  float routine_guard_deg{kRoutineEnvelopeDeg};
  float hard_abort_deg{kHardAbortDeg};
  float backlash_full_width_deg{kBacklashFullWidthDeg};
  std::uint64_t epoch_zero_timestamp_us{0};
  SessionId session_id{};
  std::array<char, 40> firmware_sha{};
  std::array<char, 40> avi_esp_libs_sha{};
  std::array<char, 64> board_build_id{};
};

struct SamplerStatistics {
  std::uint64_t released_epochs{0};
  std::uint64_t incomplete_epochs{0};
  std::uint64_t bucket_collisions{0};
  std::uint64_t trigger_coalesced_or_missed{0};
  std::uint64_t pre_epoch_samples{0};
  std::uint64_t repeated_samples{0};
  std::uint64_t skipped_samples{0};
  std::uint64_t invalid_samples{0};
  union {
    std::uint64_t late_after_release{0};
    std::uint64_t late_samples;
  };
  std::uint64_t startup_incomplete_epochs{0};
  std::uint64_t steady_state_incomplete_epochs{0};
  std::uint64_t consumer_deadline_misses{0};
  std::uint64_t raw_queue_overflows{0};
  std::uint64_t writer_queue_overflows{0};
  std::uint64_t encoder_transport_errors{0};
  std::uint64_t encoder_status_faults{0};
  std::uint64_t vbus_invalid_samples{0};
  std::int32_t first_error{0};
};
using EpochStatistics = SamplerStatistics;

struct LogFooterV5 {
  CompletionCode completion{CompletionCode::Aborted};
  bool rate_supported{false};
  UnsupportedReason unsupported_reason{UnsupportedReason::None};
  std::uint64_t total_records{0};
  std::uint64_t first_sequence{0};
  std::uint64_t last_sequence{0};
  SamplerStatistics statistics{};
  std::uint32_t qualification_valid_epochs{0};
  std::uint32_t qualification_total_epochs{0};
  std::uint32_t shutdown_step_mask{0};
  std::uint32_t file_crc32{0};
};

struct M0Handoff {
  std::uint32_t schema_version{kSchemaVersion};
  SessionId session_id{};
  std::uint32_t profile_seed{0};
  std::uint8_t completed_stage_mask{0};
  AssemblyStage expected_stage{AssemblyStage::M0};
  std::uint32_t checksum{0};
};
using PersistedM0Handoff = M0Handoff;

constexpr std::uint8_t expectedSamplesPerEpoch(EncoderRate rate) noexcept {
  switch (rate) {
  case EncoderRate::Hz1000:
    return 1U;
  case EncoderRate::Hz2000:
    return 2U;
  case EncoderRate::Hz5000:
    return 5U;
  }
  return 0U;
}

constexpr bool isSupportedEncoderRate(EncoderRate rate) noexcept {
  return expectedSamplesPerEpoch(rate) != 0U;
}

constexpr bool motorModeMatchesCommand(std::int16_t command_permille,
                                       MotorMode mode) noexcept {
  switch (mode) {
  case MotorMode::DriveIn1:
    return command_permille < 0;
  case MotorMode::DriveIn2:
    return command_permille > 0;
  case MotorMode::Coast:
  case MotorMode::Brake:
    return command_permille == 0;
  }
  return false;
}

constexpr bool isKnownStage(AssemblyStage stage) noexcept {
  return stage == AssemblyStage::FV || stage == AssemblyStage::FHPositive ||
         stage == AssemblyStage::FHNegative || stage == AssemblyStage::M0;
}

constexpr std::uint8_t stageBit(AssemblyStage stage) noexcept {
  switch (stage) {
  case AssemblyStage::FV:
    return 1U << 0U;
  case AssemblyStage::FHPositive:
    return 1U << 1U;
  case AssemblyStage::FHNegative:
    return 1U << 2U;
  case AssemblyStage::M0:
    return 1U << 3U;
  case AssemblyStage::None:
    return 0U;
  }
  return 0U;
}

static_assert(std::is_trivially_copyable_v<RawEncoderSample>);
static_assert(std::is_trivially_copyable_v<ImmutableCommandEvidence>);
static_assert(std::is_trivially_copyable_v<ImmutableLogRecord>);

} // 名前空間 avi::characterization
