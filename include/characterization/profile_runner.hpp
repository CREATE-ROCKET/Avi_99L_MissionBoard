#pragma once

#include "characterization/absolute_periodic_timer.hpp"
#include "characterization/campaign_state_machine.hpp"
#include "characterization/command_journal.hpp"
#include "characterization/encoder_sampler.hpp"
#include "characterization/log_writer_v5.hpp"
#include "characterization/power_sampler.hpp"
#include "characterization/profile_plan.hpp"
#include "characterization/zero_approach.hpp"

#if defined(ESP_PLATFORM)
#include "actuators/production_motor.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#endif

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace avi::characterization {

class ProfileRunner {
public:
#if defined(ESP_PLATFORM)
  ProfileRunner(CampaignStateMachine &campaign,
                actuators::ProductionMotorDriver &motor,
                EncoderSampler &sampler, LogWriterV5 &writer) noexcept;
#endif

  [[nodiscard]] esp_err_t arm(const SessionId &session_id) noexcept;
  [[nodiscard]] esp_err_t disarm() noexcept;
  [[nodiscard]] esp_err_t
  captureZero(ZeroReferenceKind reference_kind) noexcept;
  [[nodiscard]] esp_err_t runRateCheck(EncoderRate rate,
                                       std::uint32_t seconds);
  [[nodiscard]] esp_err_t runFullProfile(EncoderRate rate);
  void requestStop() noexcept;

  [[nodiscard]] bool busy() const noexcept { return busy_.load(); }
  [[nodiscard]] bool armed() const noexcept { return journal_.armed(); }
  [[nodiscard]] AbortReason lastAbortReason() const noexcept {
    return last_abort_reason_;
  }

private:
#if defined(ESP_PLATFORM)
  struct ConsumerTick {
    std::uint64_t alarm_value_us{0U};
    std::uint32_t alarm_lateness_us{0U};
  };

  // 正常tick + 1 epoch catch-upに加えて、さらに遅れた状態をqueue上で検出するため4件持つ。
  // policy上許可するbacklogは2 tickまでで、3 tick以上はfatal。
  static constexpr std::size_t kConsumerTickQueueDepth = 4U;

  static esp_err_t motorApply(void *context,
                              const MotorCommandRequest &request,
                              std::uint64_t &completed_at_us);
  static bool consumerTimerCallback(
      gptimer_handle_t timer, const gptimer_alarm_event_data_t *event,
      void *context);
  [[nodiscard]] esp_err_t startConsumerTimer(
      std::uint64_t epoch_zero_us);
  [[nodiscard]] esp_err_t stopConsumerTimer() noexcept;
  [[nodiscard]] esp_err_t run(EncoderRate rate, RunKind run_kind,
                              std::uint32_t epochs);
  [[nodiscard]] MotorCommandRequest
  commandForEpisode(const ProfileEpisode &episode,
                    std::uint32_t local_epoch,
                    std::int32_t fin_angle_millideg);
  [[nodiscard]] LogHeaderV5 makeHeader(EncoderRate rate, RunKind run_kind,
                                      std::uint64_t epoch_zero_us) const;
  CampaignStateMachine &campaign_;
  actuators::ProductionMotorDriver &motor_;
  EncoderSampler &sampler_;
  LogWriterV5 &writer_;
  PowerSampler power_sampler_{};
  AbsolutePeriodicTimer consumer_timer_{};
  StaticQueue_t consumer_tick_queue_control_{};
  std::array<std::uint8_t,
             kConsumerTickQueueDepth * sizeof(ConsumerTick)>
      consumer_tick_queue_storage_{};
  QueueHandle_t consumer_tick_queue_{nullptr};
  std::atomic<bool> consumer_tick_queue_overflow_{false};
  std::atomic<TaskHandle_t> consumer_task_{nullptr};
  std::atomic<bool> consumer_timer_running_{false};
#endif
  CommandJournal journal_{};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> busy_{false};
  AbortReason last_abort_reason_{AbortReason::None};
  std::uint16_t common_zero_raw_{0U};
  std::uint16_t m0_zero_raw_{0U};
  bool common_zero_valid_{false};
  bool m0_zero_valid_{false};
  std::uint32_t pseudo_random_state_{1U};
};

} // 名前空間 avi::characterization
