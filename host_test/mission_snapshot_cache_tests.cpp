#include <cassert>

#include "runtime/mission_snapshot_cache.hpp"

int main() {
  runtime::MissionSnapshotCache cache;

  const auto initial = cache.value();
  assert(initial.state == protocol::MissionState::unknown);
  assert(initial.fin == mission::FinDirective::brake);

  mission::MissionSnapshot control{};
  control.state = protocol::MissionState::control;
  control.flight_epoch = 42;
  control.liftoff_time_valid = true;
  control.elapsed_us = 12'345'000;
  control.deployment_started = true;
  control.fin = mission::FinDirective::roll_control;
  cache.update(control);

  const auto retained = cache.value();
  assert(retained.state == protocol::MissionState::control);
  assert(retained.flight_epoch == 42);
  assert(retained.elapsed_us == 12'345'000);
  assert(retained.deployment_started);
  assert(retained.fin == mission::FinDirective::roll_control);

  // mutex競合を模擬してupdateしない場合もdefaultへ戻らない。
  const auto after_contention = cache.value();
  assert(after_contention.state == protocol::MissionState::control);
  assert(after_contention.flight_epoch == 42);
  assert(after_contention.fin == mission::FinDirective::roll_control);

  mission::MissionSnapshot descent = retained;
  descent.state = protocol::MissionState::descent;
  descent.deployment_power_cutoff_latched = true;
  descent.fin = mission::FinDirective::brake;
  cache.update(descent);

  const auto updated = cache.value();
  assert(updated.state == protocol::MissionState::descent);
  assert(updated.deployment_power_cutoff_latched);
  assert(updated.fin == mission::FinDirective::brake);
  return 0;
}
