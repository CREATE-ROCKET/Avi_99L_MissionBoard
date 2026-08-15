#include "runtime/flight_log.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>

int main() {
  runtime::flight_log::Sample sample{};
  sample.monotonic_us = 0x0102'0304'0506'0708ULL;
  sample.flight_elapsed_us = 123'456;
  sample.flight_epoch = 7;
  sample.sd_drop_count = 3;
  sample.flash_drop_count = 4;
  sample.flight_status = 0xA55A;
  sample.state = 3;
  sample.config_flags = 0x6D;
  sample.fin_mode = 5;
  sample.para_mode = 1;
  sample.lps_temperature_raw = 0x46;
  sample.airspeed_raw = 0x3D;
  sample.fin_angle_raw = 0x78;
  sample.lps_pressure_raw = 0x042A;
  sample.roll_raw = 0x1234;
  sample.roll_rate_raw = 0x5678;
  sample.fin_rate_raw = 0x9ABC;
  sample.requested_torque_raw = 0x0F85;
  sample.control_roll_reference_raw = 760;
  sample.roll_deviation_raw = static_cast<uint16_t>(-1440);
  sample.control_roll_flags = 0x07;
  sample.reference_capture_event_sequence = 0x2A;
  sample.gain_clamp_flags = 1;
  sample.lps_valid = true;
  sample.airspeed_valid = true;
  sample.deployment_power_cutoff = false;
  sample.control_reference_valid = true;
  sample.fin_zero_configured = true;
  sample.reference_capture_tick = 42;
  sample.reference_estimator_timestamp_us = 987'654;
  sample.roll_estimate_rad = 1.25F;
  sample.control_reference_rad = 1.5F;
  sample.roll_deviation_rad = -0.25F;
  sample.static_pressure_pa = 101'320.0F;
  sample.ssc_temperature_celsius = 20.5F;
  sample.airspeed_mps = 80.0F;
  sample.pitot_coefficient_assumed = 0.92F;
  sample.pitot_coefficient_diagnostic_min = 0.60F;
  sample.pitot_coefficient_diagnostic_max = 1.20F;

  auto record = runtime::flight_log::serialize(sample);
  assert(record.size() == runtime::flight_log::kSerializedRecordBytes);
  assert(record[0] == '9' && record[1] == '9' && record[2] == 'L' &&
         record[3] == 'G');
  assert(record[4] == runtime::flight_log::kSchemaVersion);
  assert(record[5] == runtime::flight_log::kSerializedRecordBytes);
  assert(runtime::flight_log::validate(record));
  assert(!runtime::flight_log::erased(record));

  auto corrupted = record;
  corrupted[80] ^= 0x01;
  assert(!runtime::flight_log::validate(corrupted));

  runtime::flight_log::SerializedRecord erased{};
  erased.fill(0xFF);
  assert(runtime::flight_log::erased(erased));
  assert(!runtime::flight_log::validate(erased));

  const auto second = runtime::flight_log::serialize(sample);
  assert(std::equal(record.begin(), record.end(), second.begin()));
  return 0;
}
