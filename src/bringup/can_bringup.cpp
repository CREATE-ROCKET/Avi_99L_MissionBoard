#include "bringup/can_bringup.hpp"

#include <cinttypes>
#include <cstdio>

#include "CANCREATE.h"
#include "avi_esp_libs/timeout.h"
#include "bringup/safe_outputs.hpp"
#include "config/board_config.hpp"
#include "esp_idf_version.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if ESP_IDF_VERSION_MAJOR >= 6
#include "esp_attr.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#endif

namespace bringup {
namespace {

class BusyGuard {
public:
  explicit BusyGuard(std::atomic<bool> &busy) : busy_(busy) {
    bool expected = false;
    acquired_ = busy_.compare_exchange_strong(expected, true);
  }
  ~BusyGuard() {
    if (acquired_)
      busy_.store(false);
  }
  [[nodiscard]] bool acquired() const { return acquired_; }

private:
  std::atomic<bool> &busy_;
  bool acquired_{false};
};

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

CANCREATE::Config config(bool allow_diagnostic_frames) {
  CANCREATE::Config value{};
  value.tx = board::kCanTx;
  value.rx = board::kCanRx;
  value.bitrate = CANCREATE::Bitrate::kbps125;
  value.mode = CANCREATE::Mode::normal;
  value.rx_queue_depth = 16;
  value.allow_diagnostic_frames = allow_diagnostic_frames;
  if (allow_diagnostic_frames) {
    value.filter.identifier = board::kCanLoadTestIdentifier;
    value.filter.mask = 0x7FF;
    value.filter.extended = false;
    value.filter.enabled = true;
  }
  return value;
}

esp_err_t forceSafeOutputs() {
  const esp_err_t motor_result = safe_outputs::motorCoast();
  const esp_err_t aux_result = safe_outputs::setAux5v(false);
  const esp_err_t para_result = safe_outputs::setParaPower(false);
  std::printf("CAN safe outputs: motor=%s AUX5V=%s PARA=%s\n",
              esp_err_to_name(motor_result), esp_err_to_name(aux_result),
              esp_err_to_name(para_result));
  esp_err_t result = motor_result;
  rememberFirst(aux_result, result);
  rememberFirst(para_result, result);
  return result;
}

const char *stateName(CANCREATE::State state) {
  switch (state) {
  case CANCREATE::State::stopped:
    return "stopped";
  case CANCREATE::State::running:
    return "running";
  case CANCREATE::State::bus_off:
    return "bus_off";
  case CANCREATE::State::recovering:
    return "recovering";
  }
  return "unknown";
}

const char *testStateName(CANCREATE::TestState state) {
  switch (state) {
  case CANCREATE::TestState::success:
    return "success";
  case CANCREATE::TestState::no_peer_response:
    return "no_peer_response";
  case CANCREATE::TestState::controller_failure:
    return "controller_failure";
  }
  return "unknown";
}

void printStatus(const CANCREATE::Status &status) {
  std::printf("CAN status: state=%s pending_tx=%" PRIu32
              " pending_rx=%" PRIu32 " tx_error=%" PRIu32
              " rx_error=%" PRIu32 " bus_error=%" PRIu32
              " dropped_rx=%" PRIu32 " tx_success=%" PRIu32
              " tx_failed=%" PRIu32 " completion_valid=%d\n",
              stateName(status.state), status.pending_tx, status.pending_rx,
              status.tx_error_count, status.rx_error_count,
              status.bus_error_count, status.dropped_rx_count,
              status.successful_tx_count, status.failed_tx_count,
              status.tx_completion_counts_valid);
}

void waitUntil(int64_t target_us) {
  while (true) {
    const int64_t remaining = target_us - esp_timer_get_time();
    if (remaining <= 0)
      return;
    if (remaining >= static_cast<int64_t>(portTICK_PERIOD_MS) * 1'000) {
      vTaskDelay(1);
    } else {
      esp_rom_delay_us(static_cast<uint32_t>(remaining));
    }
  }
}

#if ESP_IDF_VERSION_MAJOR >= 6 && defined(MISSION_CAN_UNSAFE_DIAG) &&          \
    MISSION_CAN_UNSAFE_DIAG
struct RawTwaiCallbackContext {
  uint32_t tx_done{};
  uint32_t tx_success{};
};

RawTwaiCallbackContext raw_twai_callback_context;

bool IRAM_ATTR rawTwaiTransmitDone(twai_node_handle_t,
                                   const twai_tx_done_event_data_t *event,
                                   void *context) {
  // SAFETY: contextは固定storageでnode削除後も生存する。ISRではheap確保、
  // printf、blocking APIを使わず、callback引数をatomic flagへ写すだけにする。
  auto *state = static_cast<RawTwaiCallbackContext *>(context);
  __atomic_store_n(&state->tx_success, event->is_tx_success ? 1U : 0U,
                   __ATOMIC_RELAXED);
  __atomic_store_n(&state->tx_done, 1U, __ATOMIC_RELEASE);
  return false;
}
#endif

} // 無名名前空間

esp_err_t CanBringup::lifecycleTest(uint32_t count) {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  if (count == 0 || count > 1'000)
    return ESP_ERR_INVALID_ARG;

  esp_err_t result = forceSafeOutputs();
  if (result != ESP_OK)
    return result;

  uint32_t begin_ok = 0;
  uint32_t initialized_ok = 0;
  uint32_t status_ok = 0;
  uint32_t read_no_data = 0;
  uint32_t end_ok = 0;
  uint32_t completed = 0;
  for (uint32_t iteration = 1; iteration <= count; ++iteration) {
    esp_err_t begin_result = ESP_ERR_INVALID_STATE;
    esp_err_t status_result = ESP_ERR_INVALID_STATE;
    esp_err_t read_result = ESP_ERR_INVALID_STATE;
    esp_err_t end_result = ESP_ERR_INVALID_STATE;
    CANCREATE::Status status{};
    bool initialized = false;
    esp_err_t iteration_result = ESP_OK;
    {
      CANCREATE can;
      begin_result = can.begin(config(false));
      initialized = can.initialized();
      if (begin_result == ESP_OK)
        ++begin_ok;
      rememberFirst(begin_result, iteration_result);
      if (begin_result == ESP_OK && initialized) {
        ++initialized_ok;
      } else if (begin_result == ESP_OK) {
        rememberFirst(ESP_ERR_INVALID_STATE, iteration_result);
      }

      if (initialized) {
        status_result = can.getStatus(status);
        if (status_result == ESP_OK)
          ++status_ok;
        rememberFirst(status_result, iteration_result);
        if (status_result == ESP_OK && status.state != CANCREATE::State::running)
          rememberFirst(ESP_ERR_INVALID_STATE, iteration_result);

        CANCREATE::Frame frame{};
        read_result = can.read(frame, avi::Timeout::noWait());
        if (read_result == ESP_ERR_NOT_FINISHED) {
          ++read_no_data;
        } else {
          rememberFirst(read_result == ESP_OK ? ESP_ERR_INVALID_RESPONSE
                                              : read_result,
                        iteration_result);
        }

        end_result = can.end();
        if (end_result == ESP_OK)
          ++end_ok;
        rememberFirst(end_result, iteration_result);
      }
    }

    if (iteration_result == ESP_OK)
      ++completed;
    std::printf(
        "CAN lifecycle: iteration=%" PRIu32 " begin=%s initialized=%d "
        "status=%s state=%s read=%s end=%s completed=%" PRIu32 "/%" PRIu32
        "\n",
        iteration, esp_err_to_name(begin_result), initialized,
        esp_err_to_name(status_result), stateName(status.state),
        esp_err_to_name(read_result), esp_err_to_name(end_result), completed,
        count);

    // object破棄後に最低1 tick譲り、deferred処理を滞留させない。
    vTaskDelay(1);
    if (iteration_result != ESP_OK) {
      rememberFirst(iteration_result, result);
      break;
    }
  }

  std::printf(
      "CAN lifecycle summary: requested=%" PRIu32 " begin_ok=%" PRIu32
      " initialized_ok=%" PRIu32 " status_ok=%" PRIu32
      " read_no_data=%" PRIu32 " end_ok=%" PRIu32 " completed=%" PRIu32
      "\n",
      count, begin_ok, initialized_ok, status_ok, read_no_data, end_ok,
      completed);
  return result;
}

esp_err_t CanBringup::test() {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

#if !defined(MISSION_CAN_UNSAFE_DIAG) || !MISSION_CAN_UNSAFE_DIAG
  std::printf("CAN test: can_diag専用build以外では未対応\n");
  return ESP_ERR_NOT_SUPPORTED;
#endif

  esp_err_t result = forceSafeOutputs();
  if (result != ESP_OK)
    return result;

  std::printf(
      "CAN test semantics: standard ID 0x7FFをnormal/single-shotで送信し、"
      "ACK成功ならsuccess。ACKが無ければno-ack/self-loopbackを行い、"
      "loopback成功ならno_peer_response、失敗ならcontroller_failure。\n"
      "判定がFAILでも診断手順が完了すればdriver APIはESP_OKを返す。"
      "test中はbackend/queueを再生成し、最後に元Configを復元する。\n");

  CANCREATE can;
  result = can.begin(config(true));
  if (result != ESP_OK) {
    std::printf("CAN begin: %s\n", esp_err_to_name(result));
    return result;
  }

  CANCREATE::TestResult diagnostic{};
  const esp_err_t test_result = can.test(diagnostic);
  if (test_result == ESP_OK) {
    std::printf("CAN test: api=%s state=%s restored=%s\n",
                esp_err_to_name(test_result), testStateName(diagnostic.state),
                diagnostic.restored ? "true" : "false");
  } else {
    std::printf("CAN test: api=%s state=not_evaluated "
                "restored=not_evaluated\n",
                esp_err_to_name(test_result));
  }
  rememberFirst(test_result, result);
  if (test_result == ESP_OK && !diagnostic.restored)
    rememberFirst(ESP_FAIL, result);
  if (test_result == ESP_OK &&
      diagnostic.state == CANCREATE::TestState::controller_failure)
    rememberFirst(ESP_FAIL, result);

  if (can.initialized()) {
    CANCREATE::Status status{};
    const esp_err_t status_result = can.getStatus(status);
    if (status_result == ESP_OK)
      printStatus(status);
    else
      std::printf("CAN getStatus: %s\n", esp_err_to_name(status_result));
    rememberFirst(status_result, result);

    CANCREATE::Frame received{};
    const esp_err_t read_result =
        can.read(received, avi::Timeout::noWait());
    if (read_result == ESP_OK) {
      std::printf("CAN read: ESP_OK id=0x%03" PRIX32 " length=%u\n",
                  received.identifier, received.data_length);
    } else {
      std::printf("CAN read no-data path: %s\n",
                  esp_err_to_name(read_result));
      if (read_result != ESP_ERR_NOT_FINISHED)
        rememberFirst(read_result, result);
    }

    const esp_err_t end_result = can.end();
    std::printf("CAN end: %s\n", esp_err_to_name(end_result));
    rememberFirst(end_result, result);
  }
  return result;
}

esp_err_t CanBringup::idfLifecycleTest() {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

#if !defined(MISSION_CAN_UNSAFE_DIAG) || !MISSION_CAN_UNSAFE_DIAG
  std::printf("raw TWAI lifecycle: can_diag専用build以外では未対応\n");
  return ESP_ERR_NOT_SUPPORTED;
#elif ESP_IDF_VERSION_MAJOR < 6
  std::printf("raw TWAI lifecycle: ESP-IDF 6以上でのみ対応\n");
  return ESP_ERR_NOT_SUPPORTED;
#else
  esp_err_t result = forceSafeOutputs();
  if (result != ESP_OK)
    return result;

  __atomic_store_n(&raw_twai_callback_context.tx_done, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&raw_twai_callback_context.tx_success, 0U,
                   __ATOMIC_RELAXED);

  twai_node_handle_t node = nullptr;
  twai_onchip_node_config_t node_config{};
  node_config.io_cfg.tx = board::kCanTx;
  node_config.io_cfg.rx = board::kCanRx;
  node_config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  node_config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  node_config.bit_timing.bitrate = board::kCanBitrate;
  node_config.fail_retry_cnt = 0;
  node_config.tx_queue_depth = 1;
  node_config.flags.enable_self_test = true;
  node_config.flags.enable_loopback = true;

  std::printf("raw TWAI lifecycle: phase=new_node start\n");
  result = twai_new_node_onchip(&node_config, &node);
  std::printf("raw TWAI lifecycle: phase=new_node result=%s\n",
              esp_err_to_name(result));

  if (result == ESP_OK) {
    twai_event_callbacks_t callbacks{};
    callbacks.on_tx_done = rawTwaiTransmitDone;
    const esp_err_t callback_result = twai_node_register_event_callbacks(
        node, &callbacks, &raw_twai_callback_context);
    std::printf("raw TWAI lifecycle: phase=register_callback result=%s\n",
                esp_err_to_name(callback_result));
    rememberFirst(callback_result, result);
  }

  bool enabled = false;
  if (result == ESP_OK) {
    const esp_err_t enable_result = twai_node_enable(node);
    enabled = enable_result == ESP_OK;
    std::printf("raw TWAI lifecycle: phase=enable result=%s\n",
                esp_err_to_name(enable_result));
    rememberFirst(enable_result, result);
  }

  uint8_t payload = 0x5A;
  twai_frame_t frame{};
  frame.header.id = 0x123;
  frame.header.dlc = 1;
  frame.buffer = &payload;
  frame.buffer_len = sizeof(payload);
  if (result == ESP_OK) {
    const esp_err_t transmit_result = twai_node_transmit(node, &frame, 100);
    std::printf("raw TWAI lifecycle: phase=transmit result=%s\n",
                esp_err_to_name(transmit_result));
    rememberFirst(transmit_result, result);
  }

  if (result == ESP_OK) {
    const int64_t deadline_us = esp_timer_get_time() + 100'000;
    // TX callbackはatomic flagだけで通知し、待機は有限時間で打ち切る。
    while (__atomic_load_n(&raw_twai_callback_context.tx_done,
                           __ATOMIC_ACQUIRE) == 0U &&
           esp_timer_get_time() < deadline_us) {
    }
    const bool tx_done = __atomic_load_n(&raw_twai_callback_context.tx_done,
                                         __ATOMIC_ACQUIRE) != 0U;
    const bool tx_success =
        __atomic_load_n(&raw_twai_callback_context.tx_success,
                        __ATOMIC_RELAXED) != 0U;
    const esp_err_t wait_result =
        !tx_done ? ESP_ERR_TIMEOUT : (tx_success ? ESP_OK : ESP_FAIL);
    std::printf(
        "raw TWAI lifecycle: phase=tx_done result=%s done=%d success=%d\n",
        esp_err_to_name(wait_result), tx_done, tx_success);
    rememberFirst(wait_result, result);
  }

  if (enabled) {
    const esp_err_t disable_result = twai_node_disable(node);
    std::printf("raw TWAI lifecycle: phase=disable result=%s\n",
                esp_err_to_name(disable_result));
    rememberFirst(disable_result, result);
  }

  if (node != nullptr) {
    std::printf("raw TWAI lifecycle: phase=delete start\n");
    const esp_err_t delete_result = twai_node_delete(node);
    std::printf("raw TWAI lifecycle: phase=delete result=%s\n",
                esp_err_to_name(delete_result));
    rememberFirst(delete_result, result);
  }

  std::printf("raw TWAI lifecycle: phase=deferred_wait 50ms\n");
  vTaskDelay(pdMS_TO_TICKS(50));
  std::printf("raw TWAI lifecycle: result=%s\n", esp_err_to_name(result));
  return result;
#endif
}

esp_err_t CanBringup::loadTest(uint32_t frequency_hz,
                               uint32_t duration_seconds) {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  if (frequency_hz == 0 || frequency_hz > 1'000 || duration_seconds == 0)
    return ESP_ERR_INVALID_ARG;

#if ESP_IDF_VERSION_MAJOR >= 6
  // TX完了直後のnode削除も同じTWAI lifecycle問題を踏むためfail-closedにする。
  std::printf("CAN load: ESP-IDF 6以上はTWAI lifecycleの既知safe releaseを"
              "未確認のため未対応\n");
  return ESP_ERR_NOT_SUPPORTED;
#endif

  esp_err_t result = forceSafeOutputs();
  if (result != ESP_OK)
    return result;

  CANCREATE can;
  result = can.begin(config(true));
  if (result != ESP_OK) {
    std::printf("CAN load begin: %s\n", esp_err_to_name(result));
    return result;
  }

  const uint64_t requested =
      static_cast<uint64_t>(frequency_hz) * duration_seconds;
  uint64_t queued = 0;
  uint64_t write_errors = 0;
  uint64_t status_errors = 0;
  uint64_t bus_off_count = 0;
  uint64_t recovery_attempts = 0;
  uint64_t recovery_successes = 0;
  uint64_t latency_sum_us = 0;
  uint64_t latency_max_us = 0;
  uint64_t received_frames = 0;
  uint64_t read_errors = 0;
  uint64_t sequence_errors = 0;
  uint32_t expected_sequence = 0;
  bool have_sequence = false;
  bool bus_off_seen = false;

  std::printf("CAN load: id=0x%03" PRIX32
              " standard rate=%" PRIu32 "Hz duration=%" PRIu32
              "s requested=%" PRIu64 "\n",
              board::kCanLoadTestIdentifier, frequency_hz, duration_seconds,
              requested);
  std::printf(
      "注意: queuedはCAN driverへの投入成功であり、物理送信完了やACK成功数ではない。\n");

  const int64_t interval_us =
      static_cast<int64_t>(1'000'000ULL / frequency_hz);
  const int64_t started_us = esp_timer_get_time();
  int64_t next_send_us = started_us;
  for (uint64_t sequence = 0; sequence < requested; ++sequence) {
    waitUntil(next_send_us);

    CANCREATE::Frame frame{};
    frame.identifier = board::kCanLoadTestIdentifier;
    frame.data_length = 4;
    for (uint8_t byte = 0; byte < 4; ++byte)
      frame.data[byte] = static_cast<uint8_t>(sequence >> (byte * 8));

    const int64_t before = esp_timer_get_time();
    const esp_err_t write_result =
        can.write(frame, avi::Timeout::milliseconds(5));
    const uint64_t latency =
        static_cast<uint64_t>(esp_timer_get_time() - before);
    latency_sum_us += latency;
    if (latency > latency_max_us)
      latency_max_us = latency;
    if (write_result == ESP_OK) {
      ++queued;
    } else {
      ++write_errors;
      rememberFirst(write_result, result);
    }

    CANCREATE::Status status{};
    const esp_err_t status_result = can.getStatus(status);
    if (status_result != ESP_OK) {
      ++status_errors;
      rememberFirst(status_result, result);
    } else {
      const bool faulted = status.state == CANCREATE::State::bus_off ||
                           status.state == CANCREATE::State::recovering;
      if (faulted && !bus_off_seen)
        ++bus_off_count;
      bus_off_seen = faulted;
      if (faulted) {
        ++recovery_attempts;
        const esp_err_t recovery =
            can.recover(avi::Timeout::milliseconds(1'000));
        if (recovery == ESP_OK) {
          ++recovery_successes;
          bus_off_seen = false;
        } else {
          rememberFirst(recovery, result);
        }
      }
    }

    for (;;) {
      CANCREATE::Frame received{};
      const esp_err_t read_result =
          can.read(received, avi::Timeout::noWait());
      if (read_result == ESP_ERR_NOT_FINISHED)
        break;
      if (read_result != ESP_OK) {
        ++read_errors;
        rememberFirst(read_result, result);
        break;
      }
      ++received_frames;
      if (received.identifier != board::kCanLoadTestIdentifier ||
          received.extended || received.remote ||
          received.data_length < sizeof(uint32_t)) {
        ++sequence_errors;
        continue;
      }
      uint32_t received_sequence = 0;
      for (uint8_t byte = 0; byte < sizeof(uint32_t); ++byte)
        received_sequence |=
            static_cast<uint32_t>(received.data[byte]) << (byte * 8U);
      if (have_sequence && received_sequence != expected_sequence)
        ++sequence_errors;
      expected_sequence = received_sequence + 1U;
      have_sequence = true;
    }

    // 復旧待ちなどで周期を超過しても、追従のためのburst送信は行わない。
    next_send_us += interval_us;
    const int64_t now_us = esp_timer_get_time();
    if (next_send_us < now_us)
      next_send_us = now_us + interval_us;
  }

  const int64_t elapsed_us = esp_timer_get_time() - started_us;

  CANCREATE::Status final_status{};
  esp_err_t final_status_result = ESP_OK;
  const int64_t completion_deadline_us = esp_timer_get_time() + 20'000;
  do {
    final_status_result = can.getStatus(final_status);
    if (final_status_result != ESP_OK || final_status.pending_tx == 0)
      break;
    vTaskDelay(pdMS_TO_TICKS(1));
  } while (esp_timer_get_time() < completion_deadline_us);
  if (final_status_result == ESP_OK)
    printStatus(final_status);
  else
    ++status_errors;
  rememberFirst(final_status_result, result);

  const double average_latency = requested == 0
                                     ? 0.0
                                     : static_cast<double>(latency_sum_us) /
                                           static_cast<double>(requested);
  std::printf(
      "CAN load summary: requested=%" PRIu64 " queued=%" PRIu64
      " write_error=%" PRIu64 " status_error=%" PRIu64
      " received=%" PRIu64 " read_error=%" PRIu64
      " sequence_error=%" PRIu64
      " physical_tx_success=%" PRIu32 " physical_tx_failed=%" PRIu32
      " completion_valid=%d pending_tx=%" PRIu32
      " bus_off=%" PRIu64 " recovery=%" PRIu64 "/%" PRIu64
      " enqueue_latency_avg_us=%.2f max_us=%" PRIu64
      " elapsed_s=%.3f\n",
      requested, queued, write_errors, status_errors, received_frames,
      read_errors, sequence_errors, final_status.successful_tx_count,
      final_status.failed_tx_count, final_status.tx_completion_counts_valid,
      final_status.pending_tx, bus_off_count, recovery_successes,
      recovery_attempts, average_latency, latency_max_us,
      static_cast<double>(elapsed_us) / 1'000'000.0);

  if (final_status_result == ESP_OK &&
      (!final_status.tx_completion_counts_valid ||
       final_status.successful_tx_count != requested ||
       final_status.failed_tx_count != 0 || final_status.pending_tx != 0 ||
       final_status.tx_error_count != 0 || final_status.rx_error_count != 0 ||
       final_status.bus_error_count != 0 ||
       final_status.dropped_rx_count != 0 || read_errors != 0 ||
       sequence_errors != 0))
    rememberFirst(ESP_FAIL, result);

  const esp_err_t end_result = can.end();
  std::printf("CAN load end: %s\n", esp_err_to_name(end_result));
  rememberFirst(end_result, result);
  return result;
}

} // 名前空間 bringup
