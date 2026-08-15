#include "characterization/log_format_v5.hpp"

#include "characterization/campaign_state_machine.hpp"
#include "characterization/record_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace avi::characterization::wire_v5 {
namespace {

void putU16(std::uint8_t *destination, std::uint16_t value) noexcept {
  destination[0] = static_cast<std::uint8_t>(value);
  destination[1] = static_cast<std::uint8_t>(value >> 8U);
}

void putI16(std::uint8_t *destination, std::int16_t value) noexcept {
  putU16(destination, static_cast<std::uint16_t>(value));
}

void putU32(std::uint8_t *destination, std::uint32_t value) noexcept {
  for (std::size_t index = 0; index < 4U; ++index)
    destination[index] =
        static_cast<std::uint8_t>(value >> (index * 8U));
}

void putI32(std::uint8_t *destination, std::int32_t value) noexcept {
  putU32(destination, static_cast<std::uint32_t>(value));
}

void putU64(std::uint8_t *destination, std::uint64_t value) noexcept {
  for (std::size_t index = 0; index < 8U; ++index)
    destination[index] =
        static_cast<std::uint8_t>(value >> (index * 8U));
}

void putF32(std::uint8_t *destination, float value) noexcept {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  putU32(destination, bits);
}

std::uint16_t getU16(const std::uint8_t *source) noexcept {
  return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(source[0]) |
      static_cast<std::uint16_t>(source[1]) << 8U);
}

std::int16_t getI16(const std::uint8_t *source) noexcept {
  return static_cast<std::int16_t>(getU16(source));
}

std::uint32_t getU32(const std::uint8_t *source) noexcept {
  std::uint32_t value = 0U;
  for (std::size_t index = 0; index < 4U; ++index)
    value |= static_cast<std::uint32_t>(source[index]) << (index * 8U);
  return value;
}

std::int32_t getI32(const std::uint8_t *source) noexcept {
  return static_cast<std::int32_t>(getU32(source));
}

std::uint64_t getU64(const std::uint8_t *source) noexcept {
  std::uint64_t value = 0U;
  for (std::size_t index = 0; index < 8U; ++index)
    value |= static_cast<std::uint64_t>(source[index]) << (index * 8U);
  return value;
}

float getF32(const std::uint8_t *source) noexcept {
  const std::uint32_t bits = getU32(source);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

template <std::size_t Size>
void copyFixed(std::uint8_t *destination,
               const std::array<char, Size> &source) noexcept {
  std::memcpy(destination, source.data(), Size);
}

bool allZero(const std::uint8_t *data, std::size_t size) noexcept {
  for (std::size_t index = 0; index < size; ++index) {
    if (data[index] != 0U)
      return false;
  }
  return true;
}

template <std::size_t Size>
bool magicMatches(const std::uint8_t *data,
                  const std::array<std::uint8_t, Size> &magic) noexcept {
  return std::equal(magic.begin(), magic.end(), data);
}

bool knownCompletion(CompletionCode completion) noexcept {
  return completion == CompletionCode::Normal ||
         completion == CompletionCode::Aborted ||
         completion == CompletionCode::Unsupported;
}

bool knownUnsupported(UnsupportedReason reason) noexcept {
  return static_cast<std::uint8_t>(reason) <=
         static_cast<std::uint8_t>(
             UnsupportedReason::OperatorMarkedUnsupported);
}

bool validFooterState(CompletionCode completion, bool rate_supported,
                      UnsupportedReason reason) noexcept {
  if (completion == CompletionCode::Normal)
    return rate_supported && reason == UnsupportedReason::None;
  if (completion == CompletionCode::Unsupported)
    return !rate_supported && reason != UnsupportedReason::None;
  if (completion == CompletionCode::Aborted)
    return reason == UnsupportedReason::None;
  return false;
}

bool knownRun(RunKind kind) noexcept {
  return kind == RunKind::RateCheck || kind == RunKind::Full;
}

bool physicalMetadataValid(const LogHeaderV5 &header) noexcept {
  const bool finite_positive =
      std::isfinite(header.total_reduction) &&
      std::isfinite(header.physical_limit_deg) &&
      std::isfinite(header.routine_guard_deg) &&
      std::isfinite(header.hard_abort_deg) &&
      std::isfinite(header.backlash_full_width_deg) &&
      header.total_reduction > 0.0F && header.physical_limit_deg > 0.0F &&
      header.routine_guard_deg > 0.0F && header.hard_abort_deg > 0.0F &&
      header.backlash_full_width_deg > 0.0F;
  return finite_positive &&
         header.routine_guard_deg < header.hard_abort_deg &&
         header.hard_abort_deg < header.physical_limit_deg;
}

bool sha40Valid(const std::array<char, 40> &value) noexcept {
  for (const char character : value) {
    const bool digit = character >= '0' && character <= '9';
    const bool lower = character >= 'a' && character <= 'f';
    const bool upper = character >= 'A' && character <= 'F';
    if (!digit && !lower && !upper)
      return false;
  }
  return true;
}

template <std::size_t Size>
bool fixedTextValid(const std::array<char, Size> &value,
                    unsigned char minimum) noexcept {
  bool have_text = false;
  bool terminated = false;
  for (const char character : value) {
    if (character == '\0') {
      terminated = true;
      continue;
    }
    if (terminated)
      return false;
    const auto byte = static_cast<unsigned char>(character);
    if (byte < minimum || byte > 0x7EU)
      return false;
    have_text = true;
  }
  return have_text && terminated;
}

bool shutdownPrefix(std::uint32_t mask) noexcept {
  return mask <= 0x3FU && (mask & (mask + 1U)) == 0U;
}

bool fatalAcquisitionCountersAreZero(
    const SamplerStatistics &statistics) noexcept {
  return statistics.trigger_coalesced_or_missed == 0U &&
         statistics.pre_epoch_samples == 0U &&
         statistics.repeated_samples == 0U &&
         statistics.invalid_samples == 0U &&
         statistics.late_after_release == 0U &&
         statistics.steady_state_incomplete_epochs == 0U &&
         statistics.consumer_deadline_misses == 0U &&
         statistics.raw_queue_overflows == 0U &&
         statistics.writer_queue_overflows == 0U &&
         statistics.encoder_transport_errors == 0U &&
         statistics.encoder_status_faults == 0U &&
         statistics.vbus_invalid_samples == 0U;
}

bool footerFieldsValid(const LogFooterV5 &footer) noexcept {
  const bool has_records = footer.total_records != 0U;
  const bool sequence_valid =
      has_records
          ? footer.last_sequence >= footer.first_sequence &&
                footer.last_sequence - footer.first_sequence + 1U ==
                    footer.total_records
          : footer.first_sequence == 0U && footer.last_sequence == 0U;
  const bool finished = footer.completion == CompletionCode::Normal ||
                        footer.completion == CompletionCode::Unsupported;
  const bool normal_valid =
      footer.completion != CompletionCode::Normal ||
      (has_records && footer.statistics.first_error == 0 &&
       fatalAcquisitionCountersAreZero(footer.statistics));
  return sequence_valid &&
         footer.qualification_valid_epochs <=
             footer.qualification_total_epochs &&
         shutdownPrefix(footer.shutdown_step_mask) &&
         (!finished || footer.shutdown_step_mask == 0x3FU) && normal_valid;
}

bool hasUnsupportedEvidence(const LogFooterV5 &footer) noexcept {
  const SamplerStatistics &statistics = footer.statistics;
  switch (footer.unsupported_reason) {
  case UnsupportedReason::None:
    return false;
  case UnsupportedReason::TriggerCoalesced:
    return statistics.trigger_coalesced_or_missed != 0U;
  case UnsupportedReason::IncompleteEpoch:
    return statistics.pre_epoch_samples != 0U ||
           statistics.repeated_samples != 0U ||
           statistics.late_after_release != 0U ||
           statistics.steady_state_incomplete_epochs != 0U;
  case UnsupportedReason::InvalidRead:
    return statistics.invalid_samples != 0U ||
           statistics.encoder_transport_errors != 0U;
  case UnsupportedReason::DeadlineMiss:
    return statistics.consumer_deadline_misses != 0U;
  case UnsupportedReason::QueueOverflow:
    return statistics.bucket_collisions != 0U ||
           statistics.raw_queue_overflows != 0U ||
           statistics.writer_queue_overflows != 0U;
  case UnsupportedReason::WriterFailure:
    return statistics.first_error != 0;
  case UnsupportedReason::SensorHealth:
    return statistics.encoder_status_faults != 0U;
  case UnsupportedReason::OperatorMarkedUnsupported:
    return true;
  }
  return false;
}

} // 無名名前空間

std::uint32_t crc32(const std::uint8_t *data, std::size_t size,
                    std::uint32_t previous_crc) noexcept {
  if (data == nullptr && size != 0U)
    return 0U;
  std::uint32_t crc = previous_crc ^ 0xFFFFFFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (std::uint8_t bit = 0; bit < 8U; ++bit)
      crc = (crc >> 1U) ^
            (0xEDB88320U & (0U - (crc & 1U)));
  }
  return crc ^ 0xFFFFFFFFU;
}

bool encodeHeader(const LogHeaderV5 &header, HeaderBytes &bytes) noexcept {
  if (!isKnownStage(header.stage) ||
      !isSupportedEncoderRate(header.encoder_rate) ||
      !knownRun(header.run_kind) ||
      !CampaignStateMachine::validSessionId(header.session_id) ||
      !sha40Valid(header.firmware_sha) ||
      !sha40Valid(header.avi_esp_libs_sha) ||
      !fixedTextValid(header.board_build_id, 0x21U) ||
      header.profile_seed == 0U || header.pwm_frequency_hz == 0U ||
      header.epoch_zero_timestamp_us == 0U ||
      !physicalMetadataValid(header))
    return false;

  bytes.fill(0U);
  std::copy(kHeaderMagic.begin(), kHeaderMagic.end(), bytes.begin());
  putU16(bytes.data() + 8U, kSchemaVersion);
  putU16(bytes.data() + 10U, kHeaderBytes);
  putU16(bytes.data() + 12U, kRecordBytes);
  putU16(bytes.data() + 14U, kFooterBytes);
  bytes[16] = static_cast<std::uint8_t>(header.stage);
  putU16(bytes.data() + 18U, kConsumerRateHz);
  putU32(bytes.data() + 20U,
         static_cast<std::uint32_t>(header.encoder_rate));
  putU32(bytes.data() + 24U, kProfileContractVersion);
  putU32(bytes.data() + 28U, header.profile_seed);
  putU32(bytes.data() + 32U, header.pwm_frequency_hz);
  putU32(bytes.data() + 36U, header.reset_reason);
  putF32(bytes.data() + 40U, header.total_reduction);
  putF32(bytes.data() + 44U, header.physical_limit_deg);
  putF32(bytes.data() + 48U, header.routine_guard_deg);
  putF32(bytes.data() + 52U, header.hard_abort_deg);
  putF32(bytes.data() + 56U, header.backlash_full_width_deg);
  putU64(bytes.data() + 64U, header.epoch_zero_timestamp_us);
  copyFixed(bytes.data() + 72U, header.session_id);
  copyFixed(bytes.data() + 104U, header.firmware_sha);
  copyFixed(bytes.data() + 144U, header.avi_esp_libs_sha);
  copyFixed(bytes.data() + 184U, header.board_build_id);
  const std::uint32_t flags =
      header.run_kind == RunKind::RateCheck ? 1U : 2U;
  putU32(bytes.data() + 248U, flags);
  putU32(bytes.data() + 252U, crc32(bytes.data(), 252U));
  return true;
}

bool encodeRecord(const ImmutableLogRecord &record,
                  RecordBytes &bytes) noexcept {
  if (hasError(validateRecord(record)))
    return false;
  bytes.fill(0U);
  std::copy(kRecordMagic.begin(), kRecordMagic.end(), bytes.begin());
  putU64(bytes.data() + 4U, record.sequence);
  putU64(bytes.data() + 12U, record.encoder.epoch_index);
  putU64(bytes.data() + 20U,
         record.encoder.epoch_start_timestamp_us);
  putU64(bytes.data() + 28U, record.encoder.epoch_end_timestamp_us);
  putU64(bytes.data() + 36U, record.encoder.release_timestamp_us);
  putU64(bytes.data() + 44U,
         record.command.command_apply_timestamp_us);
  putU64(bytes.data() + 52U,
         record.command.logger_snapshot_timestamp_us);
  putU64(bytes.data() + 60U, record.power.capture_timestamp_us);
  putU64(bytes.data() + 68U, record.command.command_generation);
  putU32(bytes.data() + 76U, record.episode_index);
  putI32(bytes.data() + 80U, record.target_fin_angle_millideg);
  putI32(bytes.data() + 84U, record.fin_angle_millideg);
  putI32(bytes.data() + 88U, record.fin_rate_millideg_s);
  putI32(bytes.data() + 92U, record.encoder.consumer_lateness_us);
  putI32(bytes.data() + 96U, record.command.apply_result_code);
  putI32(bytes.data() + 100U, record.power.read_result);
  putI16(bytes.data() + 104U,
         record.command.requested_command_permille);
  putI16(bytes.data() + 106U, record.command.applied_command_permille);
  putU16(bytes.data() + 108U, record.power.motor_millivolts);
  putU16(bytes.data() + 110U,
         static_cast<std::uint16_t>(record.abort_reason));
  bytes[112] = static_cast<std::uint8_t>(record.stage);
  bytes[113] = static_cast<std::uint8_t>(record.profile_phase);
  bytes[114] = static_cast<std::uint8_t>(record.approach_branch);
  bytes[115] =
      static_cast<std::uint8_t>(record.command.requested_motor_mode);
  bytes[116] =
      static_cast<std::uint8_t>(record.command.applied_motor_mode);
  bytes[117] = static_cast<std::uint8_t>(record.guard_state);
  bytes[118] = record.power.valid ? 1U : 0U;
  bytes[120] = record.encoder.expected_sample_count;
  bytes[121] = record.encoder.actual_sample_count;
  bytes[122] = record.encoder.valid_sample_count;
  bytes[123] = record.encoder.repeated_sample_count;
  bytes[124] = record.encoder.skipped_sample_count;
  bytes[125] = record.encoder.invalid_sample_count;
  putU16(bytes.data() + 126U, record.encoder.flags);
  bytes[128] = static_cast<std::uint8_t>(record.zero_reference_kind);
  bytes[129] = static_cast<std::uint8_t>(record.run_kind);
  bytes[130] = record.qualification;
  putI32(bytes.data() + 132U, record.first_error);

  for (std::size_t slot = 0U;
       slot < kMaximumEncoderSamplesPerEpoch; ++slot) {
    if (!record.encoder.sample_present[slot])
      continue;
    const RawEncoderSample &sample = record.encoder.samples[slot];
    std::uint8_t *const destination = bytes.data() + 136U + slot * 36U;
    putU64(destination, sample.generation);
    putU64(destination + 8U, sample.scheduled_timestamp_us);
    putU64(destination + 16U, sample.capture_timestamp_us);
    putU16(destination + 24U, sample.angle_raw);
    putU16(destination + 26U, sample.diagnostic_flags);
    putI32(destination + 28U, sample.read_result_code);
    destination[32] = sample.valid ? 1U : 0U;
    destination[33] = static_cast<std::uint8_t>(slot);
  }
  putU32(bytes.data() + 316U, crc32(bytes.data(), 316U));
  return true;
}

bool encodeFooter(const LogFooterV5 &footer,
                  FooterBytes &bytes) noexcept {
  if (!knownCompletion(footer.completion) ||
      !knownUnsupported(footer.unsupported_reason) ||
      !validFooterState(footer.completion, footer.rate_supported,
                        footer.unsupported_reason) ||
      !footerFieldsValid(footer) ||
      (footer.completion == CompletionCode::Unsupported &&
       !hasUnsupportedEvidence(footer)))
    return false;
  bytes.fill(0U);
  std::copy(kFooterMagic.begin(), kFooterMagic.end(), bytes.begin());
  putU16(bytes.data() + 8U, kSchemaVersion);
  putU16(bytes.data() + 10U, kFooterBytes);
  bytes[12] = static_cast<std::uint8_t>(footer.completion);
  bytes[13] = footer.rate_supported ? 1U : 0U;
  bytes[14] = static_cast<std::uint8_t>(footer.unsupported_reason);
  putU64(bytes.data() + 16U, footer.total_records);
  putU64(bytes.data() + 24U, footer.first_sequence);
  putU64(bytes.data() + 32U, footer.last_sequence);
  const std::array<std::uint64_t, 11> counters{
      footer.statistics.trigger_coalesced_or_missed,
      footer.statistics.pre_epoch_samples,
      footer.statistics.repeated_samples,
      footer.statistics.skipped_samples,
      footer.statistics.invalid_samples,
      footer.statistics.late_after_release,
      footer.statistics.startup_incomplete_epochs,
      footer.statistics.steady_state_incomplete_epochs,
      footer.statistics.consumer_deadline_misses,
      footer.statistics.raw_queue_overflows,
      footer.statistics.writer_queue_overflows};
  for (std::size_t index = 0U; index < counters.size(); ++index)
    putU64(bytes.data() + 40U + index * 8U, counters[index]);
  putU64(bytes.data() + 128U,
         footer.statistics.encoder_transport_errors);
  putU64(bytes.data() + 136U, footer.statistics.encoder_status_faults);
  putU64(bytes.data() + 144U, footer.statistics.vbus_invalid_samples);
  putI32(bytes.data() + 152U, footer.statistics.first_error);
  putU32(bytes.data() + 156U, footer.qualification_valid_epochs);
  putU32(bytes.data() + 160U, footer.qualification_total_epochs);
  putU32(bytes.data() + 164U, footer.shutdown_step_mask);
  putU32(bytes.data() + 168U, footer.file_crc32);
  putU32(bytes.data() + 188U, crc32(bytes.data(), 188U));
  return true;
}

DecodeError decodeHeader(const std::uint8_t *data, std::size_t size,
                         LogHeaderV5 &header) noexcept {
  if (data == nullptr || size < kHeaderBytes)
    return DecodeError::Truncated;
  if (size > kHeaderBytes)
    return DecodeError::TrailingBytes;
  if (!magicMatches(data, kHeaderMagic))
    return DecodeError::Magic;
  if (getU16(data + 8U) != kSchemaVersion)
    return DecodeError::Schema;
  if (getU16(data + 10U) != kHeaderBytes ||
      getU16(data + 12U) != kRecordBytes ||
      getU16(data + 14U) != kFooterBytes ||
      getU16(data + 18U) != kConsumerRateHz ||
      getU32(data + 24U) != kProfileContractVersion)
    return DecodeError::Size;
  if (getU32(data + 252U) != crc32(data, 252U))
    return DecodeError::Crc;
  if (data[17] != 0U || !allZero(data + 60U, 4U))
    return DecodeError::Reserved;

  header = {};
  header.stage = static_cast<AssemblyStage>(data[16]);
  header.encoder_rate = static_cast<EncoderRate>(getU32(data + 20U));
  header.profile_seed = getU32(data + 28U);
  header.pwm_frequency_hz = getU32(data + 32U);
  header.reset_reason = getU32(data + 36U);
  header.total_reduction = getF32(data + 40U);
  header.physical_limit_deg = getF32(data + 44U);
  header.routine_guard_deg = getF32(data + 48U);
  header.hard_abort_deg = getF32(data + 52U);
  header.backlash_full_width_deg = getF32(data + 56U);
  header.epoch_zero_timestamp_us = getU64(data + 64U);
  std::memcpy(header.session_id.data(), data + 72U,
              header.session_id.size());
  std::memcpy(header.firmware_sha.data(), data + 104U,
              header.firmware_sha.size());
  std::memcpy(header.avi_esp_libs_sha.data(), data + 144U,
              header.avi_esp_libs_sha.size());
  std::memcpy(header.board_build_id.data(), data + 184U,
              header.board_build_id.size());
  const std::uint32_t flags = getU32(data + 248U);
  if (flags == 1U)
    header.run_kind = RunKind::RateCheck;
  else if (flags == 2U)
    header.run_kind = RunKind::Full;
  else
    return DecodeError::Enum;

  if (!isKnownStage(header.stage) ||
      !isSupportedEncoderRate(header.encoder_rate) ||
      !CampaignStateMachine::validSessionId(header.session_id) ||
      !sha40Valid(header.firmware_sha) ||
      !sha40Valid(header.avi_esp_libs_sha) ||
      !fixedTextValid(header.board_build_id, 0x21U) ||
      header.profile_seed == 0U || header.pwm_frequency_hz == 0U ||
      header.epoch_zero_timestamp_us == 0U ||
      !physicalMetadataValid(header))
    return DecodeError::Invariant;
  return DecodeError::None;
}

DecodeError decodeRecord(const std::uint8_t *data, std::size_t size,
                         ImmutableLogRecord &record) noexcept {
  if (data == nullptr || size < kRecordBytes)
    return DecodeError::Truncated;
  if (size > kRecordBytes)
    return DecodeError::TrailingBytes;
  if (!magicMatches(data, kRecordMagic))
    return DecodeError::Magic;
  if (getU32(data + 316U) != crc32(data, 316U))
    return DecodeError::Crc;
  if (data[119] != 0U || data[131] != 0U)
    return DecodeError::Reserved;

  record = {};
  record.sequence = getU64(data + 4U);
  record.encoder.epoch_index = getU64(data + 12U);
  record.encoder.epoch_start_timestamp_us = getU64(data + 20U);
  record.encoder.epoch_end_timestamp_us = getU64(data + 28U);
  record.encoder.release_timestamp_us = getU64(data + 36U);
  record.command.command_apply_timestamp_us = getU64(data + 44U);
  record.command.logger_snapshot_timestamp_us = getU64(data + 52U);
  record.power.capture_timestamp_us = getU64(data + 60U);
  record.command.command_generation = getU64(data + 68U);
  record.episode_index = getU32(data + 76U);
  record.target_fin_angle_millideg = getI32(data + 80U);
  record.fin_angle_millideg = getI32(data + 84U);
  record.fin_rate_millideg_s = getI32(data + 88U);
  record.encoder.consumer_lateness_us = getI32(data + 92U);
  record.command.apply_result_code = getI32(data + 96U);
  record.power.read_result = getI32(data + 100U);
  record.command.requested_command_permille = getI16(data + 104U);
  record.command.applied_command_permille = getI16(data + 106U);
  record.power.motor_millivolts = getU16(data + 108U);
  record.abort_reason =
      static_cast<AbortReason>(getU16(data + 110U));
  record.stage = static_cast<AssemblyStage>(data[112]);
  record.profile_phase = static_cast<ProfilePhase>(data[113]);
  record.approach_branch = static_cast<ApproachBranch>(data[114]);
  record.command.requested_motor_mode =
      static_cast<MotorMode>(data[115]);
  record.command.applied_motor_mode =
      static_cast<MotorMode>(data[116]);
  record.guard_state = static_cast<GuardState>(data[117]);
  if (data[118] > 1U)
    return DecodeError::Enum;
  record.power.valid = data[118] == 1U;
  record.encoder.expected_sample_count = data[120];
  record.encoder.actual_sample_count = data[121];
  record.encoder.observed_sample_count = data[121];
  record.encoder.valid_sample_count = data[122];
  record.encoder.repeated_sample_count = data[123];
  record.encoder.skipped_sample_count = data[124];
  record.encoder.invalid_sample_count = data[125];
  record.encoder.selected_sample_count =
      static_cast<std::uint8_t>(record.encoder.valid_sample_count +
                                record.encoder.invalid_sample_count);
  record.encoder.flags = getU16(data + 126U);
  record.zero_reference_kind =
      static_cast<ZeroReferenceKind>(data[128]);
  record.run_kind = static_cast<RunKind>(data[129]);
  record.qualification = data[130];
  record.first_error = getI32(data + 132U);

  switch (record.encoder.expected_sample_count) {
  case 1U:
    record.encoder_rate = EncoderRate::Hz1000;
    break;
  case 2U:
    record.encoder_rate = EncoderRate::Hz2000;
    break;
  case 5U:
    record.encoder_rate = EncoderRate::Hz5000;
    break;
  default:
    return DecodeError::Invariant;
  }

  for (std::size_t slot = 0U;
       slot < kMaximumEncoderSamplesPerEpoch; ++slot) {
    const std::uint8_t *const source = data + 136U + slot * 36U;
    if (!allZero(source + 34U, 2U))
      return DecodeError::Reserved;
    if (allZero(source, 36U))
      continue;
    if (source[32] > 1U || source[33] != slot)
      return DecodeError::Invariant;
    RawEncoderSample &sample = record.encoder.samples[slot];
    sample.generation = getU64(source);
    sample.scheduled_timestamp_us = getU64(source + 8U);
    sample.capture_timestamp_us = getU64(source + 16U);
    sample.angle_raw = getU16(source + 24U);
    sample.diagnostic_flags = getU16(source + 26U);
    sample.read_result_code = getI32(source + 28U);
    sample.valid = source[32] == 1U;
    sample.slot = source[33];
    record.encoder.sample_present[slot] = true;
  }

  if (record.zero_reference_kind != ZeroReferenceKind::Common &&
      record.zero_reference_kind != ZeroReferenceKind::M0)
    return DecodeError::Enum;
  if (!isKnownStage(record.stage) ||
      static_cast<std::uint8_t>(record.profile_phase) >
          static_cast<std::uint8_t>(ProfilePhase::Recenter) ||
      static_cast<std::uint8_t>(record.approach_branch) >
          static_cast<std::uint8_t>(ApproachBranch::FromNegative) ||
      static_cast<std::uint8_t>(record.command.requested_motor_mode) >
          static_cast<std::uint8_t>(MotorMode::Brake) ||
      static_cast<std::uint8_t>(record.command.applied_motor_mode) >
          static_cast<std::uint8_t>(MotorMode::Brake) ||
      static_cast<std::uint8_t>(record.guard_state) >
          static_cast<std::uint8_t>(GuardState::Abort) ||
      static_cast<std::uint16_t>(record.abort_reason) >
          static_cast<std::uint16_t>(AbortReason::StageError) ||
      record.qualification > 2U ||
      !knownRun(record.run_kind))
    return DecodeError::Enum;
  if (hasError(validateRecord(record)))
    return DecodeError::Invariant;
  return DecodeError::None;
}

DecodeError decodeFooter(const std::uint8_t *data, std::size_t size,
                         LogFooterV5 &footer) noexcept {
  if (data == nullptr || size < kFooterBytes)
    return DecodeError::Truncated;
  if (size > kFooterBytes)
    return DecodeError::TrailingBytes;
  if (!magicMatches(data, kFooterMagic))
    return DecodeError::Magic;
  if (getU16(data + 8U) != kSchemaVersion)
    return DecodeError::Schema;
  if (getU16(data + 10U) != kFooterBytes)
    return DecodeError::Size;
  if (getU32(data + 188U) != crc32(data, 188U))
    return DecodeError::Crc;
  if (data[13] > 1U || data[15] != 0U ||
      !allZero(data + 172U, 16U))
    return DecodeError::Reserved;

  footer = {};
  footer.completion = static_cast<CompletionCode>(data[12]);
  footer.rate_supported = data[13] == 1U;
  footer.unsupported_reason =
      static_cast<UnsupportedReason>(data[14]);
  footer.total_records = getU64(data + 16U);
  footer.first_sequence = getU64(data + 24U);
  footer.last_sequence = getU64(data + 32U);
  std::array<std::uint64_t *, 11> counters{
      &footer.statistics.trigger_coalesced_or_missed,
      &footer.statistics.pre_epoch_samples,
      &footer.statistics.repeated_samples,
      &footer.statistics.skipped_samples,
      &footer.statistics.invalid_samples,
      &footer.statistics.late_after_release,
      &footer.statistics.startup_incomplete_epochs,
      &footer.statistics.steady_state_incomplete_epochs,
      &footer.statistics.consumer_deadline_misses,
      &footer.statistics.raw_queue_overflows,
      &footer.statistics.writer_queue_overflows};
  for (std::size_t index = 0U; index < counters.size(); ++index)
    *counters[index] = getU64(data + 40U + index * 8U);
  footer.statistics.encoder_transport_errors = getU64(data + 128U);
  footer.statistics.encoder_status_faults = getU64(data + 136U);
  footer.statistics.vbus_invalid_samples = getU64(data + 144U);
  footer.statistics.first_error = getI32(data + 152U);
  footer.qualification_valid_epochs = getU32(data + 156U);
  footer.qualification_total_epochs = getU32(data + 160U);
  footer.shutdown_step_mask = getU32(data + 164U);
  footer.file_crc32 = getU32(data + 168U);
  if (!knownCompletion(footer.completion) ||
      !knownUnsupported(footer.unsupported_reason) ||
      !validFooterState(footer.completion, footer.rate_supported,
                        footer.unsupported_reason) ||
      !footerFieldsValid(footer) ||
      (footer.completion == CompletionCode::Unsupported &&
       !hasUnsupportedEvidence(footer)))
    return DecodeError::Enum;
  return DecodeError::None;
}

CaptureSummary validateCapture(const std::uint8_t *data,
                               std::size_t size) noexcept {
  CaptureSummary summary{};
  if (data == nullptr || size < kHeaderBytes + kFooterBytes) {
    summary.error = DecodeError::Truncated;
    return summary;
  }
  const std::size_t payload_size = size - kHeaderBytes - kFooterBytes;
  if (payload_size % kRecordBytes != 0U) {
    summary.error = DecodeError::TrailingBytes;
    return summary;
  }
  const std::uint64_t record_count = payload_size / kRecordBytes;
  LogHeaderV5 header{};
  summary.error = decodeHeader(data, kHeaderBytes, header);
  if (summary.error != DecodeError::None)
    return summary;

  std::uint64_t previous_sequence = 0U;
  std::uint64_t previous_epoch = 0U;
  std::uint64_t previous_raw_generation = 0U;
  std::uint64_t previous_raw_scheduled = 0U;
  std::uint64_t previous_raw_capture = 0U;
  ImmutableLogRecord previous_record{};
  std::uint64_t repeated_total = 0U;
  std::uint64_t skipped_total = 0U;
  std::uint64_t invalid_total = 0U;
  std::uint64_t late_flag_total = 0U;
  std::uint64_t startup_total = 0U;
  std::uint64_t steady_incomplete_total = 0U;
  std::uint64_t deadline_total = 0U;
  std::uint64_t vbus_invalid_total = 0U;
  std::uint64_t raw_transport_errors = 0U;
  std::uint64_t raw_status_errors = 0U;
  std::uint32_t aggregate_valid_total = 0U;
  std::int32_t record_first_error = 0;
  std::array<bool, 15> phases_seen{};
  bool record_error_present = false;
  for (std::uint64_t index = 0U; index < record_count; ++index) {
    ImmutableLogRecord record{};
    const std::size_t offset =
        kHeaderBytes + static_cast<std::size_t>(index) * kRecordBytes;
    summary.error =
        decodeRecord(data + offset, kRecordBytes, record);
    if (summary.error != DecodeError::None)
      return summary;
    if (record.stage != header.stage ||
        record.encoder_rate != header.encoder_rate ||
        record.run_kind != header.run_kind ||
        record.encoder.epoch_start_timestamp_us !=
            header.epoch_zero_timestamp_us +
                record.encoder.epoch_index * kEpochDurationUs ||
        (index != 0U &&
         (record.sequence != previous_sequence + 1U ||
          record.encoder.epoch_index != previous_epoch + 1U ||
          record.command.command_generation <
              previous_record.command.command_generation))) {
      summary.error = DecodeError::Sequence;
      return summary;
    }
    if (header.run_kind == RunKind::RateCheck &&
        (record.profile_phase != ProfilePhase::StationaryBaseline ||
         record.episode_index != 1U ||
         record.approach_branch != ApproachBranch::None)) {
      summary.error = DecodeError::Invariant;
      return summary;
    }
    phases_seen[static_cast<std::size_t>(record.profile_phase)] = true;
    if (record.first_error != 0) {
      if (record_first_error == 0)
        record_first_error = record.first_error;
      else if (record_first_error != record.first_error) {
        summary.error = DecodeError::Invariant;
        return summary;
      }
    }
    if (index != 0U &&
        record.command.command_generation ==
            previous_record.command.command_generation) {
      const bool stable_command =
          record.command.requested_command_permille ==
              previous_record.command.requested_command_permille &&
          record.command.requested_motor_mode ==
              previous_record.command.requested_motor_mode &&
          record.command.applied_command_permille ==
              previous_record.command.applied_command_permille &&
          record.command.applied_motor_mode ==
              previous_record.command.applied_motor_mode &&
          record.command.apply_result_code ==
              previous_record.command.apply_result_code &&
          record.command.command_apply_timestamp_us ==
              previous_record.command.command_apply_timestamp_us;
      if (!stable_command) {
        summary.error = DecodeError::Invariant;
        return summary;
      }
    }
    for (std::size_t slot = 0U;
         slot < kMaximumEncoderSamplesPerEpoch; ++slot) {
      if (!record.encoder.sample_present[slot])
        continue;
      const RawEncoderSample &sample = record.encoder.samples[slot];
      if ((previous_raw_generation != 0U &&
           (sample.generation <= previous_raw_generation ||
            sample.scheduled_timestamp_us <= previous_raw_scheduled ||
            sample.capture_timestamp_us <= previous_raw_capture))) {
        summary.error = DecodeError::Sequence;
        return summary;
      }
      previous_raw_generation = sample.generation;
      previous_raw_scheduled = sample.scheduled_timestamp_us;
      previous_raw_capture = sample.capture_timestamp_us;
      if (sample.read_result_code != 0)
        ++raw_transport_errors;
      if ((sample.diagnostic_flags & 0x0038U) != 0U)
        ++raw_status_errors;
    }
    repeated_total += record.encoder.repeated_sample_count;
    skipped_total += record.encoder.skipped_sample_count;
    invalid_total += record.encoder.invalid_sample_count;
    late_flag_total +=
        (record.encoder.flags & EpochLate) != 0U ? 1U : 0U;
    startup_total += (record.encoder.flags & EpochStartup) != 0U ? 1U : 0U;
    steady_incomplete_total +=
        (record.encoder.flags & EpochIncomplete) != 0U &&
                (record.encoder.flags & EpochStartup) == 0U
            ? 1U
            : 0U;
    deadline_total +=
        (record.encoder.flags & EpochDeadline) != 0U ? 1U : 0U;
    aggregate_valid_total +=
        (record.encoder.flags & EpochAggregateValid) != 0U ? 1U : 0U;
    vbus_invalid_total += record.power.valid ? 0U : 1U;
    record_error_present =
        record_error_present ||
        record.abort_reason != AbortReason::None ||
        record.first_error != 0 ||
        record.command.apply_result_code != 0;
    if (index == 0U)
      summary.first_sequence = record.sequence;
    previous_sequence = record.sequence;
    previous_epoch = record.encoder.epoch_index;
    previous_record = record;
  }
  summary.last_sequence =
      record_count == 0U ? 0U : previous_sequence;

  LogFooterV5 footer{};
  const std::size_t footer_offset = kHeaderBytes + payload_size;
  summary.error =
      decodeFooter(data + footer_offset, kFooterBytes, footer);
  if (summary.error != DecodeError::None)
    return summary;
  if (footer.total_records != record_count ||
      (record_count != 0U &&
       (footer.first_sequence != summary.first_sequence ||
        footer.last_sequence != summary.last_sequence))) {
    summary.error = DecodeError::Sequence;
    return summary;
  }
  const bool shutdown_prefix =
      footer.shutdown_step_mask <= 0x3FU &&
      (footer.shutdown_step_mask &
       (footer.shutdown_step_mask + 1U)) == 0U;
  const bool finished =
      footer.completion == CompletionCode::Normal ||
      footer.completion == CompletionCode::Unsupported;
  const bool rate_check = header.run_kind == RunKind::RateCheck;
  const bool footer_consistent =
      shutdown_prefix &&
      (!finished || footer.shutdown_step_mask == 0x3FU) &&
      (footer.completion != CompletionCode::Unsupported || rate_check) &&
      (footer.completion != CompletionCode::Unsupported ||
       hasUnsupportedEvidence(footer)) &&
      (footer.completion != CompletionCode::Unsupported ||
       footer.statistics.vbus_invalid_samples == 0U) &&
      footer.statistics.repeated_samples == repeated_total &&
      footer.statistics.skipped_samples == skipped_total &&
      footer.statistics.invalid_samples == invalid_total &&
      footer.statistics.late_after_release >= late_flag_total &&
      footer.statistics.startup_incomplete_epochs == startup_total &&
      footer.statistics.steady_state_incomplete_epochs ==
          steady_incomplete_total &&
      footer.statistics.consumer_deadline_misses >= deadline_total &&
      footer.statistics.vbus_invalid_samples >= vbus_invalid_total &&
      footer.statistics.encoder_transport_errors >= raw_transport_errors &&
      footer.statistics.encoder_status_faults >= raw_status_errors &&
      ((rate_check &&
        footer.qualification_total_epochs == record_count &&
        footer.qualification_valid_epochs == aggregate_valid_total) ||
       (!rate_check && footer.qualification_total_epochs == 0U &&
        footer.qualification_valid_epochs == 0U));
  if (!footer_consistent) {
    summary.error = DecodeError::Invariant;
    return summary;
  }
  if (footer.completion == CompletionCode::Normal) {
    const bool fatal =
        record_count == 0U || footer.statistics.first_error != 0 ||
        record_error_present ||
        footer.statistics.raw_queue_overflows != 0U ||
        footer.statistics.writer_queue_overflows != 0U ||
        footer.statistics.encoder_transport_errors != 0U ||
        footer.statistics.encoder_status_faults != 0U ||
        footer.statistics.vbus_invalid_samples != 0U ||
        footer.statistics.steady_state_incomplete_epochs != 0U ||
        footer.statistics.consumer_deadline_misses != 0U;
    if (fatal) {
      summary.error = DecodeError::Invariant;
      return summary;
    }
    if (!rate_check) {
      for (std::size_t phase = 1U; phase < phases_seen.size(); ++phase) {
        if (!phases_seen[phase]) {
          summary.error = DecodeError::Invariant;
          return summary;
        }
      }
    }
  }
  if (record_first_error != 0 &&
      footer.statistics.first_error != record_first_error) {
    summary.error = DecodeError::Invariant;
    return summary;
  }
  if (footer.file_crc32 != crc32(data, footer_offset)) {
    summary.error = DecodeError::Crc;
    return summary;
  }
  summary.records = record_count;
  return summary;
}

} // 名前空間 avi::characterization::wire_v5
