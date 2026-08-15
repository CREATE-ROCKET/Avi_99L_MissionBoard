#pragma once

#include "characterization/characterization_types.hpp"

#include <cstdint>

#ifndef AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED
#define AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED 0
#endif

#ifndef AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN
#define AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN 0
#endif

static_assert(AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED == 0 ||
              AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED == 1,
              "AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED must be 0 or 1");
static_assert(AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN >= -1 &&
                  AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN <= 1,
              "AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN must be -1, 0, "
              "or 1");

namespace avi::characterization {

// TODO(HW_TEST): polarity、PWM、encoder構成を人間が低出力試験で承認するまでdriveを禁止する。
inline constexpr bool kHardwareDriveApproved =
    AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED == 1;
inline constexpr std::int8_t kCommandToFinSign =
    AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN;
constexpr std::int16_t commandForFinError(
    std::int32_t fin_error_millideg, std::int16_t magnitude_permille,
    std::int8_t command_to_fin_sign = kCommandToFinSign) noexcept {
  if (magnitude_permille <= 0 ||
      magnitude_permille > kMaximumCommandPermille ||
      (command_to_fin_sign != -1 && command_to_fin_sign != 1))
    return 0;
  const std::int32_t fin_direction =
      fin_error_millideg > 0 ? 1 : (fin_error_millideg < 0 ? -1 : 0);
  return static_cast<std::int16_t>(fin_direction * command_to_fin_sign *
                                   magnitude_permille);
}
// TODO(HW_TEST): 実機coast停止距離から保守的な予測horizonを確定する。
inline constexpr std::int32_t kPredictedStoppingRateDivisor = 50;

} // 名前空間 avi::characterization
