#pragma once

namespace control {

struct FinOvertravelStatus {
  bool fault_latched{};
  bool sample_valid{};
  bool inside_fault_limit{};
};

// 1 kHzの動翼観測からovertravel状態を更新する。
// fault成立は有限な角度だけで判定し、解除可否にはvalid sampleを要求する。
void observeFinOvertravel(double angle_rad, bool sample_valid);

[[nodiscard]] FinOvertravelStatus finOvertravelStatus();
[[nodiscard]] bool finOvertravelFaultLatched();

// CommandReceiveでのみ呼び出し、validな角度がfault limit内へ戻った場合に解除する。
[[nodiscard]] bool clearFinOvertravelIfRecoverable();

// SetFinZero正常完了およびtestの明示reset専用。
void clearFinOvertravelFault();

} // 名前空間 control
