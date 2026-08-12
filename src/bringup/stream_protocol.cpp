#include "bringup/stream_protocol.hpp"

#include <cstring>

#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"

namespace bringup {
namespace {
constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0x5A;
constexpr uint8_t kVersion = 1;

constexpr uint16_t crc16(const uint8_t *data, std::size_t length) {
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = static_cast<uint16_t>((crc & 0x8000U) != 0
                                      ? (crc << 1) ^ 0x1021U
                                      : crc << 1);
  }
  return crc;
}

constexpr uint8_t kCrcCheck[] = {'1', '2', '3', '4', '5',
                                 '6', '7', '8', '9'};
static_assert(crc16(kCrcCheck, sizeof(kCrcCheck)) == 0x29B1);
static_assert(sizeof(float) == sizeof(uint32_t));
} // 無名名前空間

bool StreamPayload::append(uint8_t value) {
  if (size_ == bytes_.size())
    return false;
  bytes_[size_++] = value;
  return true;
}

bool StreamPayload::u8(uint8_t value) { return append(value); }

bool StreamPayload::u16(uint16_t value) {
  return append(static_cast<uint8_t>(value)) &&
         append(static_cast<uint8_t>(value >> 8));
}

bool StreamPayload::i16(int16_t value) {
  return u16(static_cast<uint16_t>(value));
}

bool StreamPayload::u32(uint32_t value) {
  return u16(static_cast<uint16_t>(value)) &&
         u16(static_cast<uint16_t>(value >> 16));
}

bool StreamPayload::i32(int32_t value) {
  return u32(static_cast<uint32_t>(value));
}

bool StreamPayload::u64(uint64_t value) {
  return u32(static_cast<uint32_t>(value)) &&
         u32(static_cast<uint32_t>(value >> 32));
}

bool StreamPayload::f32(float value) {
  uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  return u32(bits);
}

esp_err_t StreamProtocol::initialize() {
  if (queue_ != nullptr)
    return ESP_OK;
  queue_ = xQueueCreateStatic(kQueueDepth, sizeof(QueueItem),
                              queue_bytes_.data(), &queue_storage_);
  if (queue_ == nullptr)
    return ESP_ERR_NO_MEM;
  task_ = xTaskCreateStatic(writerEntry, "bringup_stream", kWriterStackWords,
                            this, 5, task_stack_.data(), &task_storage_);
  return task_ == nullptr ? ESP_ERR_NO_MEM : ESP_OK;
}

esp_err_t StreamProtocol::beginCapture() {
  if (queue_ == nullptr || active_ || outstanding_.load() != 0)
    return ESP_ERR_INVALID_STATE;
  sequence_ = 0;
  dropped_.store(0);
  output_errors_.store(0);
  active_ = true;
  return ESP_OK;
}

esp_err_t StreamProtocol::send(StreamRecordType type,
                               const StreamPayload &payload) {
  if (!active_ || payload.size() > StreamPayload::kCapacity)
    return ESP_ERR_INVALID_STATE;

  QueueItem item{};
  std::size_t offset = 0;
  item.bytes[offset++] = kMagic0;
  item.bytes[offset++] = kMagic1;
  item.bytes[offset++] = kVersion;
  item.bytes[offset++] = static_cast<uint8_t>(type);
  item.bytes[offset++] = static_cast<uint8_t>(payload.size());
  item.bytes[offset++] = static_cast<uint8_t>(payload.size() >> 8);
  const uint32_t sequence = sequence_++;
  for (uint8_t shift = 0; shift < 32; shift += 8)
    item.bytes[offset++] = static_cast<uint8_t>(sequence >> shift);
  std::memcpy(item.bytes.data() + offset, payload.data(), payload.size());
  offset += payload.size();
  const uint16_t crc = crc16(item.bytes.data() + 2, offset - 2);
  item.bytes[offset++] = static_cast<uint8_t>(crc);
  item.bytes[offset++] = static_cast<uint8_t>(crc >> 8);
  item.length = static_cast<uint16_t>(offset);

  // writerがqueueから取得した直後も未完了として数えられるよう、enqueue前に加算する。
  outstanding_.fetch_add(1);
  if (xQueueSend(queue_, &item, 0) != pdTRUE) {
    outstanding_.fetch_sub(1);
    dropped_.fetch_add(1);
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

esp_err_t StreamProtocol::finish(uint32_t timeout_ms) {
  if (!active_)
    return ESP_ERR_INVALID_STATE;
  active_ = false;
  const int64_t deadline = esp_timer_get_time() +
                           static_cast<int64_t>(timeout_ms) * 1'000;
  while (outstanding_.load() != 0 && esp_timer_get_time() < deadline)
    vTaskDelay(pdMS_TO_TICKS(1));
  if (outstanding_.load() != 0)
    return ESP_ERR_TIMEOUT;
  return output_errors_.load() == 0 ? ESP_OK : ESP_FAIL;
}

void StreamProtocol::writerEntry(void *context) {
  static_cast<StreamProtocol *>(context)->writerLoop();
}

void StreamProtocol::writerLoop() {
  QueueItem item{};
  while (true) {
    if (xQueueReceive(queue_, &item, portMAX_DELAY) != pdTRUE)
      continue;
    std::size_t written = 0;
    while (written < item.length) {
      const int result = usb_serial_jtag_write_bytes(
          item.bytes.data() + written,
          static_cast<std::size_t>(item.length) - written,
          pdMS_TO_TICKS(20));
      if (result <= 0) {
        output_errors_.fetch_add(1);
        break;
      }
      written += static_cast<std::size_t>(result);
    }
    outstanding_.fetch_sub(1);
  }
}

} // 名前空間 bringup
