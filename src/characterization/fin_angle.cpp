#include "characterization/fin_angle.hpp"

#include "characterization/characterization_types.hpp"

#include <cmath>
#include <limits>

namespace avi::characterization {

bool finAngleMilliDegreesFromUnwrappedQ16(
    std::int64_t mean_unwrapped_counts_q16, std::uint16_t zero_raw,
    std::int32_t &fin_angle_millideg) noexcept {
  constexpr long double kCountsPerMotorTurn = 16'384.0L;
  constexpr long double kQ16Scale = 65'536.0L;
  const long double unwrapped_counts =
      static_cast<long double>(mean_unwrapped_counts_q16) / kQ16Scale;
  const long double delta_counts =
      unwrapped_counts - static_cast<long double>(zero_raw);
  const long double angle =
      delta_counts * 360'000.0L /
      (kCountsPerMotorTurn * static_cast<long double>(kTotalReduction));
  if (!std::isfinite(angle))
    return false;
  const long double rounded = std::round(angle);
  if (rounded <
          static_cast<long double>(std::numeric_limits<std::int32_t>::min()) ||
      rounded >
          static_cast<long double>(std::numeric_limits<std::int32_t>::max()))
    return false;
  fin_angle_millideg = static_cast<std::int32_t>(rounded);
  return true;
}

} // 名前空間 avi::characterization
