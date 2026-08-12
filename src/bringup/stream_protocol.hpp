#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace bringup {

enum class StreamRecordType : uint8_t {
  encoder = 1,
  imu = 2,
  adc = 3,
  motor = 4,
  calibration = 5,
};

class StreamPayload {
public:
  static constexpr std::size_t kCapacity = 96;

  [[nodiscard]] bool u8(uint8_t value);
  [[nodiscard]] bool i16(int16_t value);
  [[nodiscard]] bool u16(uint16_t value);
  [[nodiscard]] bool i32(int32_t value);
  [[nodiscard]] bool u32(uint32_t value);
  [[nodiscard]] bool u64(uint64_t value);
  [[nodiscard]] bool f32(float value);
  [[nodiscard]] const uint8_t *data() const { return bytes_.data(); }
  [[nodiscard]] std::size_t size() const { return size_; }

private:
  [[nodiscard]] bool append(uint8_t value);
  std::array<uint8_t, kCapacity> bytes_{};
  std::size_t size_{0};
};

class StreamProtocol {
public:
  StreamProtocol() = default;
  StreamProtocol(const StreamProtocol &) = delete;
  StreamProtocol &operator=(const StreamProtocol &) = delete;

  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] esp_err_t beginCapture();
  [[nodiscard]] esp_err_t send(StreamRecordType type,
                               const StreamPayload &payload);
  [[nodiscard]] esp_err_t finish(uint32_t timeout_ms = 2'000);
  [[nodiscard]] uint32_t droppedFrames() const { return dropped_.load(); }
  [[nodiscard]] uint32_t outputErrors() const { return output_errors_.load(); }
  [[nodiscard]] bool active() const { return active_; }

private:
  static constexpr std::size_t kFrameCapacity = 112;
  static constexpr std::size_t kQueueDepth = 64;
  static constexpr std::size_t kWriterStackWords = 3'072;

  struct QueueItem {
    uint16_t length{0};
    std::array<uint8_t, kFrameCapacity> bytes{};
  };

  static void writerEntry(void *context);
  void writerLoop();

  StaticQueue_t queue_storage_{};
  std::array<uint8_t, kQueueDepth * sizeof(QueueItem)> queue_bytes_{};
  QueueHandle_t queue_{nullptr};
  StaticTask_t task_storage_{};
  std::array<StackType_t, kWriterStackWords> task_stack_{};
  TaskHandle_t task_{nullptr};
  std::atomic<uint32_t> dropped_{0};
  std::atomic<uint32_t> output_errors_{0};
  std::atomic<uint32_t> outstanding_{0};
  uint32_t sequence_{0};
  bool active_{false};
};

} // 名前空間 bringup
