#include "control/fin_overtravel_guard.hpp"

#include <atomic>
#include <cmath>

#include "config/control_config.hpp"

namespace control {
namespace {

std::atomic<bool> fault_latched{false};
std::atomic<bool> sample_valid{false};
std::atomic<bool> inside_fault_limit{false};

} // 無名名前空間

void observeFinOvertravel(double angle_rad, bool valid_sample) {
  const bool finite_angle = std::isfinite(angle_rad);
  const bool usable_sample = valid_sample && finite_angle;
  const bool inside =
      usable_sample && std::abs(angle_rad) <= board::kFinOvertravelFaultLimitRad;

  sample_valid.store(usable_sample, std::memory_order_release);
  inside_fault_limit.store(inside, std::memory_order_release);

  // invalid/stale/zero未設定相当のsampleは別faultで扱い、overtravelを
  // 新規成立させない。既存overtravelの解除にも利用しない。
  if (usable_sample && !inside)
    fault_latched.store(true, std::memory_order_release);
}

FinOvertravelStatus finOvertravelStatus() {
  return {fault_latched.load(std::memory_order_acquire),
          sample_valid.load(std::memory_order_acquire),
          inside_fault_limit.load(std::memory_order_acquire)};
}

bool finOvertravelFaultLatched() {
  return fault_latched.load(std::memory_order_acquire);
}

bool clearFinOvertravelIfRecoverable() {
  const auto status = finOvertravelStatus();
  if (!status.fault_latched)
    return true;
  if (!status.sample_valid || !status.inside_fault_limit)
    return false;
  fault_latched.store(false, std::memory_order_release);
  return true;
}

void clearFinOvertravelFault() {
  fault_latched.store(false, std::memory_order_release);
}

} // 名前空間 control
