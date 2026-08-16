#pragma once

#include <cstdint>

namespace runtime::recovery_persistence {

enum class LatchState : uint8_t { inactive, active, unreadable };

[[nodiscard]] LatchState latchState();
[[nodiscard]] bool ensureActive();
void setBootEvidence(bool persistent_evidence, bool rtc_marker_valid);
[[nodiscard]] bool powerOnResetUnrecoverable();
void armExitRequest(uint8_t transaction_id, bool valid_arguments);
[[nodiscard]] bool exitRequestPending(uint8_t transaction_id);
[[nodiscard]] bool prepareExit(uint8_t transaction_id);
void clearExitRequest(uint8_t transaction_id);
[[noreturn]] void onExitStatusTransmitted();

} // namespace runtime::recovery_persistence
