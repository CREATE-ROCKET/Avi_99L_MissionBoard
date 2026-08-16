#pragma once

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
