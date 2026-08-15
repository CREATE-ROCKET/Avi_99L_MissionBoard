#pragma once

#include "characterization/characterization_types.hpp"

#include <array>
#include <cstdint>

namespace avi::characterization {

class CampaignStateMachine {
public:
  [[nodiscard]] CampaignStatus
  startNewSession(const SessionId &session_id, std::uint32_t profile_seed)
      noexcept;
  [[nodiscard]] CampaignStatus confirmStage(AssemblyStage stage) noexcept;
  [[nodiscard]] CampaignStatus beginRun(EncoderRate rate,
                                        RunKind kind) noexcept;
  [[nodiscard]] CampaignStatus
  finishRun(RunOutcome outcome,
            UnsupportedReason unsupported_reason = UnsupportedReason::None)
      noexcept;
  [[nodiscard]] CampaignStatus completeStage() noexcept;
  [[nodiscard]] CampaignStatus prepareM0(M0Handoff &handoff) noexcept;
  [[nodiscard]] CampaignStatus
  resumeM0(const M0Handoff &handoff, ResetKind reset_kind,
           bool rtc_cookie_survived) noexcept;
  [[nodiscard]] CampaignStatus
  resumeM0(const SessionId &confirmed_session_id,
           const char *literal_fin_removed) noexcept;

  [[nodiscard]] bool canArmMotor() const noexcept;
  [[nodiscard]] CampaignState state() const noexcept { return state_; }
  [[nodiscard]] AssemblyStage stage() const noexcept { return stage_; }
  [[nodiscard]] const SessionId &sessionId() const noexcept {
    return session_id_;
  }
  [[nodiscard]] std::uint32_t profileSeed() const noexcept {
    return profile_seed_;
  }
  [[nodiscard]] std::uint8_t completedStageMask() const noexcept {
    return completed_stage_mask_;
  }
  [[nodiscard]] const std::array<RateResult, 3> &rateResults() const noexcept {
    return rate_results_;
  }
  [[nodiscard]] bool full1000Completed() const noexcept {
    return full_completed_[0];
  }

  void fault() noexcept { state_ = CampaignState::Faulted; }

  [[nodiscard]] static bool validSessionId(const SessionId &session_id)
      noexcept;
  [[nodiscard]] static std::uint32_t
  computeHandoffChecksum(const M0Handoff &handoff) noexcept;
  [[nodiscard]] static bool validateHandoff(const M0Handoff &handoff)
      noexcept;

private:
  [[nodiscard]] static std::size_t rateIndex(EncoderRate rate) noexcept;
  [[nodiscard]] bool allRateChecksResolved() const noexcept;
  void resetStageRuns() noexcept;

  SessionId session_id_{};
  CampaignState state_{CampaignState::Idle};
  AssemblyStage stage_{AssemblyStage::None};
  EncoderRate active_rate_{EncoderRate::Hz1000};
  RunKind active_run_kind_{RunKind::None};
  std::uint32_t profile_seed_{0};
  std::uint8_t completed_stage_mask_{0};
  std::array<RateResult, 3> rate_results_{};
  std::array<bool, 3> full_completed_{};
};

} // 名前空間 avi::characterization
