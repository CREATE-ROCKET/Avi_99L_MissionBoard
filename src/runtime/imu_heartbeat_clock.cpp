#include "bringup/imu_bringup.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(AVI_99L_IMU_HEARTBEAT_CLOCK) && AVI_99L_IMU_HEARTBEAT_CLOCK

#include <cstdint>

namespace {

// 1 kHz ICM42688 FIFO watermark INTを通常周期源にする。
// 3 ms以内に来なければMissionRealtimeTaskを起こし、既存のstale/recovery判定へ渡す。
constexpr std::uint32_t kHeartbeatTimeoutMs = 3U;

} // 無名名前空間

extern "C" BaseType_t
__real_xTaskDelayUntil(TickType_t *const previous_wake,
                       const TickType_t increment);

extern "C" BaseType_t
__wrap_xTaskDelayUntil(TickType_t *const previous_wake,
                       const TickType_t increment) {
  if (previous_wake == nullptr)
    return __real_xTaskDelayUntil(previous_wake, increment);

  const bringup::ImuHeartbeatWaitResult heartbeat =
      bringup::waitRegisteredImuHeartbeat(kHeartbeatTimeoutMs);
  if (heartbeat == bringup::ImuHeartbeatWaitResult::unavailable)
    return __real_xTaskDelayUntil(previous_wake, increment);

  // 登録ownerはImuBringup::begin()を実行したMissionRealtimeTaskだけである。
  // heartbeatが成功した場合はFIFO threshold GPIO ISR由来で起床済み。
  // timeout/error時も追加のtick待ちを入れず、現在時刻へphaseを再同期して
  // runtime本体のIMU stale/error/restart処理を直ちに実行させる。
  *previous_wake = xTaskGetTickCount();
  return heartbeat == bringup::ImuHeartbeatWaitResult::ready ? pdTRUE
                                                              : pdFALSE;
}

#endif
