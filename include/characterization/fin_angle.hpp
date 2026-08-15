#pragma once

#include <cstdint>

namespace avi::characterization {

// multi-turnのQ16 encoder countを再wrapせずfin-equivalent角へ変換する。
[[nodiscard]] bool finAngleMilliDegreesFromUnwrappedQ16(
    std::int64_t mean_unwrapped_counts_q16, std::uint16_t zero_raw,
    std::int32_t &fin_angle_millideg) noexcept;

} // 名前空間 avi::characterization
