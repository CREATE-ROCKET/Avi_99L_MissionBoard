#include "sensors/power_presence.hpp"

#include <algorithm>
#include <cmath>

namespace sensors {

void PowerPresenceDetector::reset() {
  last_valid_timestamp_us_ = 0;
  assert_count_ = 0;
  clear_count_ = 0;
  present_ = false;
}

void PowerPresenceDetector::update(uint64_t timestamp_us, double voltage_v,
                                   bool sample_valid) {
  if (!config_.configured || !sample_valid || timestamp_us == 0 ||
      !std::isfinite(voltage_v))
    return;
  last_valid_timestamp_us_ = timestamp_us;
  if (!present_) {
    clear_count_ = 0;
    if (voltage_v >= config_.present_on_voltage_v)
      assert_count_ = std::min<uint16_t>(assert_count_ + 1, config_.assert_samples);
    else
      assert_count_ = 0;
    if (config_.assert_samples != 0 && assert_count_ >= config_.assert_samples) {
      present_ = true;
      assert_count_ = 0;
    }
    return;
  }

  assert_count_ = 0;
  if (voltage_v <= config_.present_off_voltage_v)
    clear_count_ = std::min<uint16_t>(clear_count_ + 1, config_.clear_samples);
  else
    clear_count_ = 0;
  if (config_.clear_samples != 0 && clear_count_ >= config_.clear_samples) {
    present_ = false;
    clear_count_ = 0;
  }
}

bool PowerPresenceDetector::present(uint64_t now_us) const {
  if (!config_.configured || !present_ || last_valid_timestamp_us_ == 0 ||
      now_us < last_valid_timestamp_us_)
    return false;
  return config_.stale_timeout_us == 0 ||
         now_us - last_valid_timestamp_us_ <= config_.stale_timeout_us;
}

} // namespace sensors
