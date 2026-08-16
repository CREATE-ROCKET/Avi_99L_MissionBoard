#include "runtime/recovery_persistence.hpp"

#include <atomic>

#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "runtime/recovery_boot.hpp"

namespace runtime::recovery_persistence {
namespace {

constexpr char kNamespace[] = "recovery";
constexpr char kActiveKey[] = "active_v1";
std::atomic<bool> por_unrecoverable{false};
std::atomic<uint8_t> exit_transaction{0};
std::atomic<bool> exit_arguments_valid{false};
std::atomic<bool> exit_prepared{false};

} // namespace

LatchState latchState() {
  if (nvs_flash_init() != ESP_OK)
    return LatchState::unreadable;
  nvs_handle_t handle{};
  const esp_err_t opened = nvs_open(kNamespace, NVS_READONLY, &handle);
  if (opened == ESP_ERR_NVS_NOT_FOUND)
    return LatchState::inactive;
  if (opened != ESP_OK)
    return LatchState::unreadable;
  uint8_t active = 0;
  const esp_err_t result = nvs_get_u8(handle, kActiveKey, &active);
  nvs_close(handle);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    return LatchState::inactive;
  if (result != ESP_OK || active != 1)
    return LatchState::unreadable;
  return LatchState::active;
}

bool ensureActive() {
  if (latchState() == LatchState::active)
    return true;
  if (nvs_flash_init() != ESP_OK)
    return false;
  nvs_handle_t handle{};
  if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK)
    return false;
  esp_err_t result = nvs_set_u8(handle, kActiveKey, 1);
  if (result == ESP_OK)
    result = nvs_commit(handle);
  nvs_close(handle);
  return result == ESP_OK && latchState() == LatchState::active;
}

void setBootEvidence(bool persistent_evidence, bool rtc_marker_valid) {
  por_unrecoverable.store(persistent_evidence && !rtc_marker_valid,
                          std::memory_order_release);
}

bool powerOnResetUnrecoverable() {
  return por_unrecoverable.load(std::memory_order_acquire);
}

void armExitRequest(uint8_t transaction_id, bool valid_arguments) {
  if (transaction_id == 0)
    return;
  exit_arguments_valid.store(valid_arguments, std::memory_order_release);
  exit_transaction.store(transaction_id, std::memory_order_release);
  exit_prepared.store(false, std::memory_order_release);
}

bool exitRequestPending(uint8_t transaction_id) {
  return transaction_id != 0 &&
         exit_transaction.load(std::memory_order_acquire) == transaction_id;
}

bool exitArgumentsValid(uint8_t transaction_id) {
  return exitRequestPending(transaction_id) &&
         exit_arguments_valid.load(std::memory_order_acquire);
}

bool prepareExit(uint8_t transaction_id) {
  if (!exitArgumentsValid(transaction_id))
    return false;
  if (exit_prepared.load(std::memory_order_acquire))
    return true;
  if (nvs_flash_init() != ESP_OK)
    return false;
  nvs_handle_t handle{};
  const esp_err_t opened = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (opened != ESP_OK && opened != ESP_ERR_NVS_NOT_FOUND)
    return false;
  esp_err_t result = ESP_OK;
  if (opened == ESP_OK) {
    result = nvs_erase_key(handle, kActiveKey);
    if (result == ESP_ERR_NVS_NOT_FOUND)
      result = ESP_OK;
    if (result == ESP_OK)
      result = nvs_commit(handle);
    nvs_close(handle);
  }
  if (result != ESP_OK || latchState() != LatchState::inactive)
    return false;
  recovery_boot::clearMarker();
  por_unrecoverable.store(false, std::memory_order_release);
  exit_prepared.store(true, std::memory_order_release);
  return true;
}

void clearExitRequest(uint8_t transaction_id) {
  if (!exitRequestPending(transaction_id))
    return;
  exit_transaction.store(0, std::memory_order_release);
  exit_arguments_valid.store(false, std::memory_order_release);
  exit_prepared.store(false, std::memory_order_release);
}

[[noreturn]] void onExitStatusTransmitted() {
  esp_restart();
  for (;;) {
  }
}

} // namespace runtime::recovery_persistence
