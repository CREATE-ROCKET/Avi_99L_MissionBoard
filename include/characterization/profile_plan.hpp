#pragma once

#include "characterization/characterization_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace avi::characterization {

struct ProfileEpisode {
  ProfilePhase phase{ProfilePhase::Idle};
  ApproachBranch branch{ApproachBranch::None};
  std::uint32_t episode_index{0};
  std::uint32_t duration_epochs{0};
  std::int16_t command_limit_permille{0};
  bool include_in_identification_fit{false};
};

class ProfilePlan {
public:
  static constexpr std::size_t kCommonEpisodeCount = 16U;
  // TODO(HW_TEST): 低出力極性試験後にepisode別上限を確定する。
  static constexpr std::int16_t kInitialMotionCommandPermille = 30;

  ProfilePlan(AssemblyStage stage, EncoderRate rate,
              std::uint32_t fixed_seed) noexcept;

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] AssemblyStage stage() const noexcept { return stage_; }
  [[nodiscard]] EncoderRate rate() const noexcept { return rate_; }
  [[nodiscard]] std::uint32_t seed() const noexcept { return fixed_seed_; }
  [[nodiscard]] ZeroReferenceKind zeroReferenceKind() const noexcept;
  [[nodiscard]] const std::array<ProfileEpisode, kCommonEpisodeCount> &
  commonEpisodes() const noexcept {
    return common_episodes_;
  }
  [[nodiscard]] bool commonComparableTo(const ProfilePlan &other) const
      noexcept;

private:
  static std::array<ProfileEpisode, kCommonEpisodeCount>
  makeCommonEpisodes() noexcept;

  AssemblyStage stage_{AssemblyStage::None};
  EncoderRate rate_{EncoderRate::Hz1000};
  std::uint32_t fixed_seed_{0};
  std::array<ProfileEpisode, kCommonEpisodeCount> common_episodes_{};
};

} // 名前空間 avi::characterization
