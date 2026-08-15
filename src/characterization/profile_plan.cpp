#include "characterization/profile_plan.hpp"

namespace avi::characterization {

std::array<ProfileEpisode, ProfilePlan::kCommonEpisodeCount>
ProfilePlan::makeCommonEpisodes() noexcept {
  constexpr std::int16_t motion = kInitialMotionCommandPermille;
  constexpr std::int16_t broadband = kBroadbandCommandPermille;
  constexpr std::int16_t polarity = kPolarityCheckCommandPermille;
  return {{{ProfilePhase::StationaryBaseline, ApproachBranch::None, 1U,
            1'000U, 0, false},
           // 動作済みZeroHoldの位置feedbackを基礎に、+/-1 degから0へ0.1 deg刻みで接近する。
           {ProfilePhase::ZeroApproach, ApproachBranch::FromPositive, 2U,
            18'000U, motion, false},
           {ProfilePhase::ZeroApproach, ApproachBranch::FromNegative, 3U,
            18'000U, motion, false},
           // 極性は別実機試験で設定済みであることがarm条件。ここでは短pulseで証跡だけ残す。
           {ProfilePhase::PolarityCheck, ApproachBranch::None, 4U, 800U,
            polarity, false},
           // 100..300 permilleを正負それぞれ静止側から短pulseしbreakawayを測る。
           {ProfilePhase::BreakawaySweep, ApproachBranch::None, 5U, 3'600U,
            motion, true},
           // 高dutyで運動を開始してから低dutyへ移し、sustained thresholdを方向別に測る。
           {ProfilePhase::SustainedMotionSweep, ApproachBranch::None, 6U,
            2'880U, motion, true},
           // 100..300 permilleの正負短pulseを各2回取得する。
           {ProfilePhase::BoundedPulseGrid, ApproachBranch::None, 7U, 4'000U,
            motion, true},
           // 各方向100 ms spin-up後、1 s coastを取得する。
           {ProfilePhase::Coast, ApproachBranch::None, 8U, 2'200U, motion,
            true},
           // 各方向100 ms spin-up後、250 ms short brakeを取得する。
           {ProfilePhase::ShortBrake, ApproachBranch::None, 9U, 2'000U,
            motion, true},
           {ProfilePhase::PositiveToNegative, ApproachBranch::None, 10U,
            800U, broadband, true},
           {ProfilePhase::NegativeToPositive, ApproachBranch::None, 11U,
            800U, broadband, true},
           {ProfilePhase::BoundedPrbs, ApproachBranch::None, 12U, 4'000U,
            broadband, true},
           {ProfilePhase::BandLimitedNoise, ApproachBranch::None, 13U,
            4'000U, broadband, true},
           {ProfilePhase::Chirp, ApproachBranch::None, 14U, 4'000U,
            broadband, true},
           {ProfilePhase::Recenter, ApproachBranch::None, 15U, 5'000U,
            motion, false},
           {ProfilePhase::StationaryBaseline, ApproachBranch::None, 16U,
            1'000U, 0, false}}};
}

ProfilePlan::ProfilePlan(AssemblyStage stage, EncoderRate rate,
                         std::uint32_t fixed_seed) noexcept
    : stage_(stage), rate_(rate), fixed_seed_(fixed_seed),
      common_episodes_(makeCommonEpisodes()) {}

bool ProfilePlan::valid() const noexcept {
  return isKnownStage(stage_) && isSupportedEncoderRate(rate_) &&
         fixed_seed_ != 0U;
}

ZeroReferenceKind ProfilePlan::zeroReferenceKind() const noexcept {
  return stage_ == AssemblyStage::M0 ? ZeroReferenceKind::M0
                                     : ZeroReferenceKind::Common;
}

bool ProfilePlan::commonComparableTo(const ProfilePlan &other) const noexcept {
  if (!valid() || !other.valid() || fixed_seed_ != other.fixed_seed_)
    return false;
  for (std::size_t index = 0; index < common_episodes_.size(); ++index) {
    const ProfileEpisode &left = common_episodes_[index];
    const ProfileEpisode &right = other.common_episodes_[index];
    if (left.phase != right.phase || left.branch != right.branch ||
        left.episode_index != right.episode_index ||
        left.duration_epochs != right.duration_epochs ||
        left.command_limit_permille != right.command_limit_permille ||
        left.include_in_identification_fit !=
            right.include_in_identification_fit) {
      return false;
    }
  }
  return true;
}

} // 名前空間 avi::characterization
