#include "characterization/campaign_state_machine.hpp"

#include <cstring>

namespace avi::characterization {
namespace {

constexpr std::uint32_t kFnvOffset = 2'166'136'261U;
constexpr std::uint32_t kFnvPrime = 16'777'619U;

void hashByte(std::uint32_t &hash, std::uint8_t value) noexcept {
  hash ^= value;
  hash *= kFnvPrime;
}

void hashU32(std::uint32_t &hash, std::uint32_t value) noexcept {
  for (std::uint32_t shift = 0; shift < 32U; shift += 8U)
    hashByte(hash, static_cast<std::uint8_t>(value >> shift));
}

} // 無名名前空間

bool CampaignStateMachine::validSessionId(
    const SessionId &session_id) noexcept {
  bool terminated = false;
  std::size_t length = 0U;
  for (const char value : session_id) {
    if (value == '\0') {
      terminated = true;
      continue;
    }
    if (terminated)
      return false;
    const auto byte = static_cast<unsigned char>(value);
    if (byte < 0x21U || byte > 0x7EU)
      return false;
    ++length;
  }
  return terminated && length != 0U &&
         length <= kSessionIdMaximumPrintableBytes;
}

void CampaignStateMachine::resetStageRuns() noexcept {
  rate_results_ = {};
  // M0完了前だけCharacterizationRuntime側に旧3-slot resolved checkが残る。
  // 5 kHzは新規取得対象外なので、M0ではlegacy slotを最初からunsupported扱いにする。
  if (stage_ == AssemblyStage::M0)
    rate_results_[2] = RateResult::Unsupported;
  full_completed_ = {};
  active_run_kind_ = RunKind::None;
}

CampaignStatus CampaignStateMachine::startNewSession(
    const SessionId &session_id, std::uint32_t profile_seed) noexcept {
  if (state_ != CampaignState::Idle)
    return CampaignStatus::InvalidState;
  if (!validSessionId(session_id) || profile_seed == 0U)
    return CampaignStatus::InvalidArgument;
  session_id_ = session_id;
  profile_seed_ = profile_seed;
  completed_stage_mask_ = 0U;
  stage_ = AssemblyStage::FV;
  resetStageRuns();
  state_ = CampaignState::AwaitingStageConfirmation;
  return CampaignStatus::Ok;
}

CampaignStatus
CampaignStateMachine::confirmStage(AssemblyStage stage) noexcept {
  if (state_ != CampaignState::AwaitingStageConfirmation)
    return CampaignStatus::InvalidState;
  if (stage != stage_ || stage == AssemblyStage::M0)
    return stage == AssemblyStage::M0
               ? CampaignStatus::FinRemovalConfirmationRequired
               : CampaignStatus::WrongStage;
  state_ = CampaignState::Ready;
  return CampaignStatus::Ok;
}

std::size_t
CampaignStateMachine::rateIndex(EncoderRate rate) noexcept {
  switch (rate) {
  case EncoderRate::Hz1000:
    return 0U;
  case EncoderRate::Hz2000:
    return 1U;
  case EncoderRate::Hz5000:
    return 2U;
  }
  return 3U;
}

CampaignStatus CampaignStateMachine::beginRun(EncoderRate rate,
                                               RunKind kind) noexcept {
  if (state_ != CampaignState::Ready)
    return CampaignStatus::InvalidState;
  const std::size_t index = rateIndex(rate);
  if (index >= rate_results_.size() ||
      (kind != RunKind::RateCheck && kind != RunKind::Full))
    return CampaignStatus::InvalidArgument;
  if (kind == RunKind::RateCheck &&
      rate_results_[index] != RateResult::Pending)
    return CampaignStatus::WrongRateOrder;
  if (kind == RunKind::Full &&
      rate_results_[index] != RateResult::Accepted)
    return CampaignStatus::RateNotQualified;
  active_rate_ = rate;
  active_run_kind_ = kind;
  state_ = CampaignState::Running;
  return CampaignStatus::Ok;
}

CampaignStatus CampaignStateMachine::finishRun(
    RunOutcome outcome, UnsupportedReason unsupported_reason) noexcept {
  if (state_ != CampaignState::Running)
    return CampaignStatus::InvalidState;
  const std::size_t index = rateIndex(active_rate_);
  if (index >= rate_results_.size())
    return CampaignStatus::InvalidArgument;

  if (active_run_kind_ == RunKind::RateCheck) {
    if (outcome == RunOutcome::Accepted) {
      rate_results_[index] = RateResult::Accepted;
    } else if (outcome == RunOutcome::Unsupported) {
      if (active_rate_ == EncoderRate::Hz1000) {
        state_ = CampaignState::Ready;
        active_run_kind_ = RunKind::None;
        return CampaignStatus::MandatoryRateUnsupported;
      }
      if (unsupported_reason == UnsupportedReason::None)
        return CampaignStatus::InvalidArgument;
      rate_results_[index] = RateResult::Unsupported;
    }
  } else if (active_run_kind_ == RunKind::Full &&
             outcome == RunOutcome::Accepted) {
    full_completed_[index] = true;
  }

  active_run_kind_ = RunKind::None;
  state_ = CampaignState::Ready;
  return CampaignStatus::Ok;
}

bool CampaignStateMachine::allRateChecksResolved() const noexcept {
  // AS5047Dの新規acquisitionは1 kHz / 2 kHzだけを対象とする。
  // index 2の5 kHz slotはV5の旧campaign互換のため保持するが、Stage完了条件には使わない。
  return rate_results_[0] != RateResult::Pending &&
         rate_results_[1] != RateResult::Pending;
}

CampaignStatus CampaignStateMachine::completeStage() noexcept {
  if (state_ != CampaignState::Ready || !allRateChecksResolved() ||
      !full_completed_[0])
    return CampaignStatus::InvalidState;
  completed_stage_mask_ =
      static_cast<std::uint8_t>(completed_stage_mask_ | stageBit(stage_));
  switch (stage_) {
  case AssemblyStage::FV:
    stage_ = AssemblyStage::FHPositive;
    resetStageRuns();
    state_ = CampaignState::AwaitingStageConfirmation;
    return CampaignStatus::Ok;
  case AssemblyStage::FHPositive:
    stage_ = AssemblyStage::FHNegative;
    resetStageRuns();
    state_ = CampaignState::AwaitingStageConfirmation;
    return CampaignStatus::Ok;
  case AssemblyStage::FHNegative:
    state_ = CampaignState::AwaitingM0Handoff;
    return CampaignStatus::Ok;
  case AssemblyStage::M0:
    state_ = CampaignState::Completed;
    return CampaignStatus::Ok;
  case AssemblyStage::None:
    return CampaignStatus::WrongStage;
  }
  return CampaignStatus::WrongStage;
}

std::uint32_t CampaignStateMachine::computeHandoffChecksum(
    const M0Handoff &handoff) noexcept {
  std::uint32_t hash = kFnvOffset;
  hashU32(hash, handoff.schema_version);
  for (const char value : handoff.session_id)
    hashByte(hash, static_cast<std::uint8_t>(value));
  hashU32(hash, handoff.profile_seed);
  hashByte(hash, handoff.completed_stage_mask);
  hashByte(hash, static_cast<std::uint8_t>(handoff.expected_stage));
  return hash;
}

bool CampaignStateMachine::validateHandoff(
    const M0Handoff &handoff) noexcept {
  const std::uint8_t required =
      static_cast<std::uint8_t>(stageBit(AssemblyStage::FV) |
                                stageBit(AssemblyStage::FHPositive) |
                                stageBit(AssemblyStage::FHNegative));
  if (handoff.schema_version != kSchemaVersion ||
      !validSessionId(handoff.session_id) || handoff.profile_seed == 0U ||
      handoff.expected_stage != AssemblyStage::M0 ||
      (handoff.completed_stage_mask & required) != required)
    return false;
  M0Handoff copy = handoff;
  copy.checksum = 0U;
  return handoff.checksum == computeHandoffChecksum(copy);
}

CampaignStatus
CampaignStateMachine::prepareM0(M0Handoff &handoff) noexcept {
  if (state_ != CampaignState::AwaitingM0Handoff ||
      stage_ != AssemblyStage::FHNegative)
    return CampaignStatus::InvalidState;
  handoff = {};
  handoff.session_id = session_id_;
  handoff.profile_seed = profile_seed_;
  handoff.completed_stage_mask = completed_stage_mask_;
  handoff.checksum = computeHandoffChecksum(handoff);
  state_ = CampaignState::PowerCycleRequired;
  return CampaignStatus::Ok;
}

CampaignStatus CampaignStateMachine::resumeM0(
    const M0Handoff &handoff, ResetKind reset_kind,
    bool rtc_cookie_survived) noexcept {
  if (state_ != CampaignState::Idle)
    return CampaignStatus::InvalidState;
  if (!validateHandoff(handoff))
    return CampaignStatus::InvalidHandoff;
  if (reset_kind != ResetKind::PowerOn)
    return CampaignStatus::PowerCycleRequired;
  if (rtc_cookie_survived)
    return CampaignStatus::ColdPowerCycleNotProven;
  session_id_ = handoff.session_id;
  profile_seed_ = handoff.profile_seed;
  completed_stage_mask_ = handoff.completed_stage_mask;
  stage_ = AssemblyStage::M0;
  resetStageRuns();
  state_ = CampaignState::AwaitingM0Confirmation;
  return CampaignStatus::Ok;
}

CampaignStatus CampaignStateMachine::resumeM0(
    const SessionId &confirmed_session_id,
    const char *literal_fin_removed) noexcept {
  if (state_ != CampaignState::AwaitingM0Confirmation)
    return CampaignStatus::InvalidState;
  if (confirmed_session_id != session_id_)
    return CampaignStatus::SessionIdMismatch;
  if (literal_fin_removed == nullptr ||
      std::strcmp(literal_fin_removed, "FIN_REMOVED") != 0)
    return CampaignStatus::FinRemovalConfirmationRequired;
  state_ = CampaignState::Ready;
  return CampaignStatus::Ok;
}

bool CampaignStateMachine::canArmMotor() const noexcept {
  return state_ == CampaignState::Ready && isKnownStage(stage_);
}

} // 名前空間 avi::characterization
