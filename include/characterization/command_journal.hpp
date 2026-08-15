#pragma once

#include "characterization/characterization_types.hpp"
#include "characterization/platform_compat.hpp"

#include <cstdint>

namespace avi::characterization {

using MotorApplyCallback =
    esp_err_t (*)(void *context, const MotorCommandRequest &request,
                 std::uint64_t &completed_at_us);

// commandと実出力証跡はrealtime taskだけが更新する。
class CommandJournal {
public:
  CommandJournal() = default;
  CommandJournal(MotorApplyCallback callback, void *context) noexcept
      : callback_(callback), context_(context) {}

  void bind(MotorApplyCallback callback, void *context) noexcept {
    callback_ = callback;
    context_ = context;
  }
  [[nodiscard]] esp_err_t arm() noexcept;
  [[nodiscard]] esp_err_t disarm(std::uint64_t now_us) noexcept;
  [[nodiscard]] esp_err_t stopForError(esp_err_t cause,
                                       std::uint64_t now_us) noexcept;
  [[nodiscard]] bool armed() const noexcept { return armed_; }

  [[nodiscard]] esp_err_t apply(const MotorCommandRequest &request,
                                std::uint64_t now_us) noexcept;
  [[nodiscard]] MotorCommandApplied currentApplied() const noexcept {
    return current_applied_;
  }
  [[nodiscard]] ImmutableCommandEvidence
  snapshot(std::uint64_t logger_snapshot_us) const noexcept;
  [[nodiscard]] ImmutableCommandEvidence
  snapshotForLogger(std::uint64_t logger_snapshot_us) const noexcept {
    return snapshot(logger_snapshot_us);
  }

private:
  [[nodiscard]] esp_err_t applyCoast(std::uint64_t now_us) noexcept;

  MotorApplyCallback callback_{nullptr};
  void *context_{nullptr};
  std::uint64_t next_generation_{1U};
  ImmutableCommandEvidence current_{};
  MotorCommandApplied current_applied_{};
  bool armed_{false};
};

} // 名前空間 avi::characterization
