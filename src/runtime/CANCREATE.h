#pragma once

#include_next "CANCREATE.h"

#include <atomic>
#include <cstdint>

#include "esp_timer.h"
#include "runtime/recovery_persistence.hpp"
#include "sensors/display_attitude_runtime.hpp"
#include "sensors/power_presence_runtime.hpp"

namespace mission_runtime_can_detail {
// Kinematics 0x100のrollはCommandReceive中だけ表示用rollへ差し替える。
// MissionEventとMissionStatusの両方から状態を追跡し、遷移直後の100 Hz frameでも
// 旧状態を長く保持しないようにする。
inline std::atomic<uint8_t> mission_state_raw{0};
} // namespace mission_runtime_can_detail

class MissionRuntimeCan final : public CANCREATE {
public:
  using CANCREATE::write;

  [[nodiscard]] esp_err_t read(
      Frame &frame, avi::Timeout timeout = avi::Timeout::noWait()) {
    const esp_err_t result = CANCREATE::read(frame, timeout);
    if (result != ESP_OK)
      return result;

    if (frame.identifier == 0x008 && frame.data_length == 8 &&
        (frame.data[0] & 0x0FU) == 4U) {
      const bool valid_arguments =
          ((frame.data[0] >> 4U) & 0x01U) == 0U && frame.data[1] != 0 &&
          frame.data[2] == 0 && frame.data[3] == 0 && frame.data[4] == 0 &&
          frame.data[5] == 0 && frame.data[6] == 0 && frame.data[7] == 0;
      runtime::recovery_persistence::armExitRequest(frame.data[1],
                                                     valid_arguments);
      // 現行runtimeへはWakeとして渡し、既存Recovery owner taskに応答を生成させる。
      frame.data[0] = static_cast<uint8_t>((frame.data[0] & 0x10U) | 1U);
    }
    return result;
  }

  [[nodiscard]] esp_err_t write(
      const Frame &input, avi::Timeout timeout = avi::Timeout::noWait()) {
    Frame frame = input;
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

    if (frame.identifier == 0x011 && frame.data_length == 8 &&
        frame.data[1] == 0x30) {
      const uint8_t phase = frame.data[2];
      const uint8_t reason = frame.data[3];
      if (phase == 0 && reason == 0)
        sensors::display_attitude_runtime::calibrationAccepted(frame.data[0]);
      else if (phase == 1 || phase == 2 || phase == 3)
        sensors::display_attitude_runtime::calibrationFinished(
            frame.data[0], phase == 1 && reason == 0);
    }

    if (frame.identifier == 0x020 && frame.data_length == 8)
      mission_runtime_can_detail::mission_state_raw.store(
          frame.data[3], std::memory_order_release);

    if (frame.identifier == 0x102 && frame.data_length == 8) {
      mission_runtime_can_detail::mission_state_raw.store(
          frame.data[1], std::memory_order_release);
      uint16_t status = static_cast<uint16_t>(frame.data[2]) |
                        (static_cast<uint16_t>(frame.data[3]) << 8U);
      if (sensors::power_presence_runtime::logicPresent(now_us))
        status |= 1U << 5U;
      else
        status &= static_cast<uint16_t>(~(1U << 5U));
      if (sensors::power_presence_runtime::motorPresent(now_us))
        status |= 1U << 6U;
      else
        status &= static_cast<uint16_t>(~(1U << 6U));
      if ((status & (1U << 9U)) != 0)
        sensors::display_attitude_runtime::invalidateForDataLoss();
      if ((status & (1U << 14U)) != 0)
        sensors::display_attitude_runtime::invalidateForReset();
      frame.data[2] = static_cast<uint8_t>(status);
      frame.data[3] = static_cast<uint8_t>(status >> 8U);
    }

    if (frame.identifier == 0x100 && frame.data_length == 8 &&
        mission_runtime_can_detail::mission_state_raw.load(
            std::memory_order_acquire) == 0U) {
      const auto attitude = sensors::display_attitude_runtime::wireTelemetry(now_us);
      frame.data[1] = static_cast<uint8_t>(attitude.roll_raw);
      frame.data[2] = static_cast<uint8_t>(attitude.roll_raw >> 8U);
    }

    if (frame.identifier == 0x103 && frame.data_length == 8) {
      sensors::power_presence_runtime::observeRaw(frame.data[1], frame.data[2],
                                                  now_us);
      if (runtime::recovery_persistence::powerOnResetUnrecoverable()) {
        frame.data[5] = 0xFD;
        frame.data[6] = 0xFF;
      }
    }

    if (frame.identifier == 0x107 && frame.data_length == 3) {
      const auto attitude = sensors::display_attitude_runtime::wireTelemetry(now_us);
      const uint16_t packed =
          static_cast<uint16_t>(attitude.magnitude_raw & 0x7FU) |
          static_cast<uint16_t>((attitude.direction_raw & 0x01FFU) << 7U);
      frame.data[1] = static_cast<uint8_t>(packed);
      frame.data[2] = static_cast<uint8_t>(packed >> 8U);
    }

    if (frame.identifier == 0x014 && frame.data_length == 3) {
      (void)runtime::recovery_persistence::ensureActive();
      if (runtime::recovery_persistence::powerOnResetUnrecoverable())
        frame.data[2] = 3; // ResetRecovery
    }

    bool exit_complete = false;
    uint8_t exit_transaction = 0;
    if (frame.identifier == 0x105 && frame.data_length == 8 &&
        frame.data[0] == 1U &&
        runtime::recovery_persistence::exitRequestPending(frame.data[1])) {
      exit_transaction = frame.data[1];
      frame.data[0] = 4U;
      frame.data[3] = 0U;
      frame.data[4] = 0;
      frame.data[5] = 0;
      frame.data[6] = 0;
      frame.data[7] = 0;
      if (frame.data[2] == 0U) {
        if (!runtime::recovery_persistence::exitArgumentsValid(exit_transaction)) {
          frame.data[2] = 5U; // InvalidArgument
        } else if (runtime::recovery_persistence::prepareExit(exit_transaction)) {
          frame.data[2] = 2U; // Complete
          exit_complete = true;
        } else {
          frame.data[2] = 7U; // IoError
        }
      }
    }

    const esp_err_t result = CANCREATE::write(frame, timeout);
    if (result == ESP_OK && exit_transaction != 0) {
      if (exit_complete)
        runtime::recovery_persistence::onExitStatusTransmitted();
      runtime::recovery_persistence::clearExitRequest(exit_transaction);
    }
    return result;
  }
};

#define CANCREATE MissionRuntimeCan
