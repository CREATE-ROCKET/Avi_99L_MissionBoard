#include "characterization/command_journal.hpp"

#include <cstdlib>

namespace avi::characterization {

esp_err_t CommandJournal::arm() noexcept {
  if (armed_ || callback_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  armed_ = true;
  return ESP_OK;
}

esp_err_t CommandJournal::applyCoast(std::uint64_t now_us) noexcept {
  const MotorCommandRequest coast{0, MotorMode::Coast};
  std::uint64_t completed_at_us = now_us;
  const esp_err_t result =
      callback_ == nullptr ? ESP_ERR_INVALID_STATE
                           : callback_(context_, coast, completed_at_us);
  current_applied_ = {0, MotorMode::Coast, result, completed_at_us};
  return result;
}

esp_err_t CommandJournal::disarm(std::uint64_t now_us) noexcept {
  const esp_err_t result = applyCoast(now_us);
  armed_ = false;
  return result;
}

esp_err_t CommandJournal::stopForError(esp_err_t cause,
                                       std::uint64_t now_us) noexcept {
  if (cause == ESP_OK)
    return ESP_ERR_INVALID_ARG;
  // 安全停止の成否にかかわらず、診断では最初の原因を保持する。
  (void)disarm(now_us);
  return cause;
}

esp_err_t CommandJournal::apply(const MotorCommandRequest &request,
                                std::uint64_t now_us) noexcept {
  const bool valid_request =
      motorModeMatchesCommand(request.command_permille, request.mode) &&
      std::abs(static_cast<int>(request.command_permille)) <=
          kMaximumCommandPermille;
  const bool needs_arm =
      request.command_permille != 0 || request.mode == MotorMode::Brake;

  esp_err_t result = ESP_OK;
  if (!valid_request)
    result = ESP_ERR_INVALID_ARG;
  else if (needs_arm && !armed_)
    result = ESP_ERR_INVALID_STATE;
  else if (callback_ == nullptr)
    result = ESP_ERR_INVALID_STATE;
  else {
    std::uint64_t completed_at_us = now_us;
    result = callback_(context_, request, completed_at_us);
    current_applied_ = {request.command_permille, request.mode, result,
                        completed_at_us};
  }

  current_ = {};
  current_.command_generation = next_generation_++;
  current_.requested_command_permille = request.command_permille;
  current_.requested_motor_mode = request.mode;
  current_.logger_snapshot_timestamp_us = now_us;

  const esp_err_t requested_result = result;
  if (requested_result != ESP_OK) {
    // fallback coastの実状態はcurrent_applied_へ残し、要求失敗を上書きしない。
    (void)applyCoast(now_us);
    armed_ = false;
  }

  current_.applied_command_permille =
      current_applied_.applied_command_permille;
  current_.applied_motor_mode = current_applied_.applied_mode;
  current_.apply_result_code = requested_result;
  current_.command_apply_timestamp_us =
      current_applied_.command_apply_timestamp_us;
  return requested_result;
}

ImmutableCommandEvidence CommandJournal::snapshot(
    std::uint64_t logger_snapshot_us) const noexcept {
  ImmutableCommandEvidence evidence = current_;
  evidence.logger_snapshot_timestamp_us = logger_snapshot_us;
  return evidence;
}

} // 名前空間 avi::characterization
