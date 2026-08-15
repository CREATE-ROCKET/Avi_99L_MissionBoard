#include "characterization/profile_plan.hpp"

namespace avi::characterization {

std::array<ProfileEpisode, ProfilePlan::kCommonEpisodeCount>
ProfilePlan::makeCommonEpisodes() noexcept {
  constexpr std::int16_t motion = kInitialMotionCommandPermille;
  return {{{ProfilePhase::StationaryBaseline, ApproachBranch::None, 1U,
            1'000U, 0, false},
           {ProfilePhase::ZeroApproach, ApproachBranch::FromPositive, 2U,
            11'000U, motion, false},
           {ProfilePhase::ZeroApproach, ApproachBranch::FromNegative, 3U,
            11'000U, motion, false},
           {ProfilePhase::PolarityCheck, ApproachBranch::None, 4U, 400U,
            motion, true},
           {ProfilePhase::BreakawaySweep, ApproachBranch::None, 5U, 1'200U,
            motion, true},
           {ProfilePhase::SustainedMotionSweep, ApproachBranch::None, 6U,
            1'200U, motion, true},
           {ProfilePhase::BoundedPulseGrid, ApproachBranch::None, 7U, 1'600U,
            motion, true},
           {ProfilePhase::Coast, ApproachBranch::None, 8U, 500U, 0, true},
           {ProfilePhase::ShortBrake, ApproachBranch::None, 9U, 200U, 0,
            true},
           {ProfilePhase::PositiveToNegative, ApproachBranch::None, 10U,
            600U, motion, true},
           {ProfilePhase::NegativeToPositive, ApproachBranch::None, 11U,
            600U, motion, true},
           {ProfilePhase::BoundedPrbs, ApproachBranch::None, 12U, 2'000U,
            motion, true},
           {ProfilePhase::BandLimitedNoise, ApproachBranch::None, 13U,
            2'000U, motion, true},
           {ProfilePhase::Chirp, ApproachBranch::None, 14U, 2'000U, motion,
            true},
           {ProfilePhase::Recenter, ApproachBranch::None, 15U, 2'000U,
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
