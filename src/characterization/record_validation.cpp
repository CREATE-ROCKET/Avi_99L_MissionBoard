#include "characterization/record_validation.hpp"

namespace avi::characterization {
namespace {

constexpr bool knownPhase(ProfilePhase phase) noexcept {
  return static_cast<std::uint8_t>(phase) <=
         static_cast<std::uint8_t>(ProfilePhase::Recenter);
}

constexpr bool knownBranch(ApproachBranch branch) noexcept {
  return static_cast<std::uint8_t>(branch) <=
         static_cast<std::uint8_t>(ApproachBranch::FromNegative);
}

constexpr bool knownGuard(GuardState guard) noexcept {
  return static_cast<std::uint8_t>(guard) <=
         static_cast<std::uint8_t>(GuardState::Abort);
}

constexpr bool knownAbort(AbortReason reason) noexcept {
  return static_cast<std::uint16_t>(reason) <=
         static_cast<std::uint16_t>(AbortReason::StageError);
}

constexpr bool knownRunKind(RunKind kind) noexcept {
  return kind == RunKind::RateCheck || kind == RunKind::Full;
}

} // 無名名前空間

RecordValidationError
validateRecordStrict(const ImmutableLogRecord &record) noexcept {
  RecordValidationError error = RecordValidationError::None;
  if (!isKnownStage(record.stage))
    error = error | RecordValidationError::InvalidStage;
  if (!isSupportedEncoderRate(record.encoder_rate))
    error = error | RecordValidationError::InvalidEncoderRate;
  if (!knownPhase(record.profile_phase) ||
      !knownBranch(record.approach_branch) ||
      !knownGuard(record.guard_state) || !knownAbort(record.abort_reason) ||
      !knownRunKind(record.run_kind))
    error = error | RecordValidationError::InvalidEnum;
  if (!motorModeMatchesCommand(record.command.requested_command_permille,
                               record.command.requested_motor_mode))
    error = error | RecordValidationError::RequestedCommandModeMismatch;
  if (!motorModeMatchesCommand(record.command.applied_command_permille,
                               record.command.applied_motor_mode))
    error = error | RecordValidationError::AppliedCommandModeMismatch;
  if (record.command.requested_command_permille <
          -kMaximumCommandPermille ||
      record.command.requested_command_permille >
          kMaximumCommandPermille ||
      record.command.applied_command_permille <
          -kMaximumCommandPermille ||
      record.command.applied_command_permille >
          kMaximumCommandPermille)
    error = error | RecordValidationError::InvalidGeneration;
  if (record.command.command_generation == 0U)
    error = error | RecordValidationError::InvalidGeneration;

  // command_apply_timestampは「現在のapplied stateへ最後に遷移した実時刻」。
  // 同一commandが複数epoch継続する場合はepoch_startより前であることが正常。
  if (record.command.command_apply_timestamp_us == 0U ||
      record.command.command_apply_timestamp_us >
          record.encoder.release_timestamp_us ||
      record.encoder.release_timestamp_us >
          record.command.logger_snapshot_timestamp_us ||
      record.command.logger_snapshot_timestamp_us <
          record.command.command_apply_timestamp_us)
    error = error | RecordValidationError::InvalidCommandTimestamp;
  const EncoderEpochBlock &block = record.encoder;
  const bool command_applied_this_epoch =
      record.command.command_apply_timestamp_us >=
      record.encoder.epoch_start_timestamp_us;
  const bool command_apply_target_late =
      command_applied_this_epoch &&
      record.command.command_apply_timestamp_us >
          record.encoder.epoch_start_timestamp_us +
              kConsumerDeadlineBudgetUs;
  const bool command_apply_hard_deadline_missed =
      command_applied_this_epoch &&
      record.command.command_apply_timestamp_us >=
          record.encoder.epoch_start_timestamp_us + kEpochDurationUs;

  if (block.epoch_end_timestamp_us !=
          block.epoch_start_timestamp_us + kEpochDurationUs ||
      block.release_timestamp_us < block.epoch_end_timestamp_us)
    error = error | RecordValidationError::InvalidEpochTimestamp;
  if (block.expected_sample_count !=
      expectedSamplesPerEpoch(record.encoder_rate))
    error = error | RecordValidationError::WrongExpectedSampleCount;
  const bool counts_consistent =
      block.expected_sample_count <= kMaximumEncoderSamplesPerEpoch &&
      block.selected_sample_count <= block.expected_sample_count &&
      static_cast<unsigned>(block.valid_sample_count) +
              block.invalid_sample_count ==
          block.selected_sample_count &&
      static_cast<unsigned>(block.selected_sample_count) +
              block.skipped_sample_count ==
          block.expected_sample_count &&
      static_cast<unsigned>(block.selected_sample_count) +
              block.repeated_sample_count ==
          block.actual_sample_count;
  if (!counts_consistent)
    error = error | RecordValidationError::InconsistentCounts;

  const bool complete =
      counts_consistent &&
      block.valid_sample_count == block.expected_sample_count &&
      block.repeated_sample_count == 0U &&
      block.skipped_sample_count == 0U &&
      block.invalid_sample_count == 0U;
  const bool aggregate_valid =
      (block.flags & EpochAggregateValid) != 0U;
  const bool incomplete = (block.flags & EpochIncomplete) != 0U;
  const bool startup_incomplete = (block.flags & EpochStartup) != 0U;
  const bool deadline_flag = (block.flags & EpochDeadline) != 0U;
  const bool release_late =
      block.consumer_lateness_us >
      static_cast<std::int32_t>(kConsumerDeadlineBudgetUs);
  // 旧V5は100 us command遅延やrelease遅延にもEpochDeadlineを付けていた。
  // 新しいfull motor-IDは100 usを診断targetとして扱い、1 epoch以上のcommand遅延だけ
  // fatal flagにする。rate-checkの旧strict契約はdecode互換のため維持する。
  const bool legacy_command_deadline =
      deadline_flag && command_apply_target_late &&
      !command_apply_hard_deadline_missed;
  if (complete != aggregate_valid || complete == incomplete ||
      startup_incomplete != (block.epoch_index == 0U && incomplete) ||
      ((block.flags & EpochRepeated) != 0U) !=
          (block.repeated_sample_count != 0U) ||
      ((block.flags & EpochSkipped) != 0U) !=
          (block.skipped_sample_count != 0U) ||
      ((block.flags & EpochInvalid) != 0U) !=
          (block.invalid_sample_count != 0U) ||
      (command_apply_hard_deadline_missed && !deadline_flag) ||
      (record.run_kind == RunKind::RateCheck && command_apply_target_late &&
       !deadline_flag) ||
      (deadline_flag && !command_apply_target_late && !release_late) ||
      (block.flags & ~0x00FFU) != 0U)
    error = error | RecordValidationError::InconsistentFlags;
  if (record.run_kind == RunKind::Full &&
      (command_apply_hard_deadline_missed || legacy_command_deadline) &&
      (record.first_error == 0 ||
       record.abort_reason != AbortReason::Deadline))
    error = error | RecordValidationError::InvalidCommandTimestamp;

  std::uint64_t previous_generation = 0U;
  std::uint64_t previous_scheduled = 0U;
  std::uint64_t previous_capture = 0U;
  for (std::size_t index = 0;
       index < kMaximumEncoderSamplesPerEpoch; ++index) {
    if (!block.sample_present[index])
      continue;
    const RawEncoderSample &sample = block.samples[index];
    const std::uint64_t slot_count =
        block.expected_sample_count == 0U
            ? 1U
            : block.expected_sample_count;
    const std::uint64_t slot_start =
        block.epoch_start_timestamp_us +
        index * kEpochDurationUs / slot_count;
    const std::uint64_t slot_end =
        block.epoch_start_timestamp_us +
        (index + 1U) * kEpochDurationUs / slot_count;
    if (sample.capture_timestamp_us < block.epoch_start_timestamp_us ||
        sample.capture_timestamp_us >= block.epoch_end_timestamp_us ||
        sample.capture_timestamp_us < slot_start ||
        sample.capture_timestamp_us >= slot_end ||
        sample.scheduled_timestamp_us == 0U ||
        sample.scheduled_timestamp_us > sample.capture_timestamp_us ||
        sample.generation == 0U || sample.angle_raw > 0x3FFFU ||
        sample.slot != index ||
        (previous_generation != 0U &&
         (sample.generation <= previous_generation ||
          sample.scheduled_timestamp_us <= previous_scheduled ||
          sample.capture_timestamp_us <= previous_capture)))
      error = error | RecordValidationError::SampleOutsideEpoch;
    previous_generation = sample.generation;
    previous_scheduled = sample.scheduled_timestamp_us;
    previous_capture = sample.capture_timestamp_us;
    const bool diagnostic_valid =
        (sample.diagnostic_flags & ~kRawDiagnosticMask) == 0U &&
        ((sample.valid && sample.read_result_code == 0 &&
          (sample.diagnostic_flags & 0x003FU) == 0U) ||
         (!sample.valid &&
          (sample.read_result_code != 0 ||
           (sample.diagnostic_flags & 0x003FU) != 0U)));
    if (!diagnostic_valid)
      error = error | RecordValidationError::InvalidRawDiagnostic;
  }

  if ((record.power.valid &&
       (record.power.read_result != 0 ||
        record.power.capture_timestamp_us == 0U ||
        record.power.capture_timestamp_us >
            record.command.logger_snapshot_timestamp_us ||
        record.command.logger_snapshot_timestamp_us -
                record.power.capture_timestamp_us >
            100'000U)) ||
      (!record.power.valid &&
       (record.power.read_result == 0 ||
        record.power.capture_timestamp_us >
            record.command.logger_snapshot_timestamp_us)))
    error = error | RecordValidationError::InvalidVbus;
  if ((record.stage == AssemblyStage::M0) !=
      (record.zero_reference_kind == ZeroReferenceKind::M0))
    error = error | RecordValidationError::InvalidEnum;
  if ((record.first_error == 0) !=
      (record.abort_reason == AbortReason::None))
    error = error | RecordValidationError::InvalidGeneration;
  const std::uint8_t expected_qualification =
      record.run_kind == RunKind::RateCheck ? 0U : 1U;
  if (record.qualification != expected_qualification)
    error = error | RecordValidationError::InvalidEnum;
  return error;
}

RecordValidationError
validateRecord(const ImmutableLogRecord &record) noexcept {
#if defined(ESP_PLATFORM) && defined(AVI_99L_CHARACTERIZATION) &&             \
    AVI_99L_CHARACTERIZATION
  // 実機characterizationではrealtime callerにfull validationを実行させない。
  // char_writerがqueueから値copyしたimmutable recordへvalidateRecordStrict()を
  // 実行してからwire encodeし、失敗時は既存failure notificationで停止させる。
  (void)record;
  return RecordValidationError::None;
#else
  return validateRecordStrict(record);
#endif
}

} // 名前空間 avi::characterization
