from pathlib import Path


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: target count={count}, expected=1")
    path.write_text(text.replace(old, new, 1))


runtime = Path("src/runtime/production_runtime.cpp")
replace_once(
    runtime,
    '#include "runtime/flight_storage.hpp"\n',
    '#include "runtime/flight_storage.hpp"\n#include "runtime/mission_snapshot_cache.hpp"\n',
    "production runtime snapshot cache include",
)
replace_once(
    runtime,
    """  uint32_t timestamp_epoch = 1;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
""",
    """  uint32_t timestamp_epoch = 1;
  MissionSnapshotCache mission_snapshot_cache;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
""",
    "production runtime snapshot cache storage",
)
replace_once(
    runtime,
    """    protocol::MissionState detector_state =
        protocol::MissionState::command_receive;
    uint32_t current_epoch = 0;
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      const auto snapshot = state_machine.snapshot();
      detector_state = snapshot.state;
      current_epoch = snapshot.flight_epoch;
      xSemaphoreGive(state_mutex);
    }
""",
    """    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      mission_snapshot_cache.update(state_machine.snapshot());
      xSemaphoreGive(state_mutex);
    }
    // mutex競合時はCommandReceiveへ偽装せず、最後のcoherent snapshotを使う。
    const auto detector_snapshot = mission_snapshot_cache.value();
    const protocol::MissionState detector_state = detector_snapshot.state;
    const uint32_t current_epoch = detector_snapshot.flight_epoch;
""",
    "MissionRealtime detector snapshot fallback",
)
replace_once(
    runtime,
    """    RuntimeStatus status{};
    bool power_cutoff = false;
    bool deployment_started = false;
    uint32_t flight_epoch = 0;
    mission::MissionSnapshot mission_snapshot{};
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      mission_snapshot = state_machine.snapshot();
      status.state = mission_snapshot.state;
      status.fin_mode = mission_snapshot.fin == mission::FinDirective::zero_hold
                            ? protocol::FinMode::zero_hold
                            : (mission_snapshot.fin == mission::FinDirective::roll_control
                                   ? protocol::FinMode::roll_control
                                   : protocol::FinMode::brake);
      status.para_mode = para_mode_actual.load(std::memory_order_acquire);
      power_cutoff = mission_snapshot.deployment_power_cutoff_latched;
      deployment_started = mission_snapshot.deployment_started;
      flight_epoch = mission_snapshot.flight_epoch;
      xSemaphoreGive(state_mutex);
    }
""",
    """    RuntimeStatus status{};
    bool power_cutoff = false;
    bool deployment_started = false;
    uint32_t flight_epoch = 0;
    if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
      mission_snapshot_cache.update(state_machine.snapshot());
      xSemaphoreGive(state_mutex);
    }
    // 取得失敗時にdefault構築値をpublish/制御へ流さず、一つ前の正常値を使う。
    const auto mission_snapshot = mission_snapshot_cache.value();
    status.state = mission_snapshot.state;
    status.fin_mode = mission_snapshot.fin == mission::FinDirective::zero_hold
                          ? protocol::FinMode::zero_hold
                          : (mission_snapshot.fin == mission::FinDirective::roll_control
                                 ? protocol::FinMode::roll_control
                                 : protocol::FinMode::brake);
    // Para modeは独立atomicなのでstate mutexの成否に依存させない。
    status.para_mode = para_mode_actual.load(std::memory_order_acquire);
    power_cutoff = mission_snapshot.deployment_power_cutoff_latched;
    deployment_started = mission_snapshot.deployment_started;
    flight_epoch = mission_snapshot.flight_epoch;
""",
    "MissionRealtime control/status snapshot fallback",
)

cache_header = Path("src/runtime/mission_snapshot_cache.hpp")
if not cache_header.exists():
    cache_header.write_text(
        """#pragma once

#include "mission/mission_state.hpp"

namespace runtime {

class MissionSnapshotCache {
public:
  MissionSnapshotCache() {
    // 最初の取得すら競合した場合はCommandReceiveへ誤遷移せず安全側で待つ。
    snapshot_.state = protocol::MissionState::unknown;
    snapshot_.fin = mission::FinDirective::brake;
    snapshot_.parachute = mission::ParaDirective::hold;
  }

  void update(const mission::MissionSnapshot &snapshot) { snapshot_ = snapshot; }

  [[nodiscard]] const mission::MissionSnapshot &value() const {
    return snapshot_;
  }

private:
  mission::MissionSnapshot snapshot_{};
};

} // namespace runtime
"""
    )

test_file = Path("host_test/mission_snapshot_cache_tests.cpp")
if not test_file.exists():
    test_file.write_text(
        """#include <cassert>

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
"""
    )

cmake = Path("host_test/CMakeLists.txt")
replace_once(
    cmake,
    """add_test(NAME mission_state_safety_regression_tests
         COMMAND mission_state_safety_regression_tests)

add_executable(flight_log_tests
""",
    """add_test(NAME mission_state_safety_regression_tests
         COMMAND mission_state_safety_regression_tests)

add_executable(mission_snapshot_cache_tests
  mission_snapshot_cache_tests.cpp
)
target_include_directories(mission_snapshot_cache_tests PRIVATE ../src)
target_compile_features(mission_snapshot_cache_tests PRIVATE cxx_std_17)
target_compile_options(mission_snapshot_cache_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_test(NAME mission_snapshot_cache_tests COMMAND mission_snapshot_cache_tests)

add_executable(flight_log_tests
""",
    "host test snapshot cache target",
)

print("MissionBoard mission snapshot cache patch applied")
