#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "bringup/calibration_bringup.hpp"
#include "bringup/can_bringup.hpp"
#include "bringup/encoder_bringup.hpp"
#include "bringup/i2c_bringup.hpp"
#include "bringup/imu_bringup.hpp"
#include "bringup/motor_bringup.hpp"
#include "bringup/spi_bringup.hpp"
#include "bringup/sts_bringup.hpp"
#include "bringup/stream_protocol.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace bringup {

class BringupShell {
public:
  explicit BringupShell(StreamProtocol &stream)
      : stream_(stream), motor_(spi_, encoder_, imu_, stream_),
        calibration_(spi_, imu_, stream_) {}

  [[nodiscard]] esp_err_t initialize();
  [[noreturn]] void run();

private:
  static constexpr std::size_t kLineCapacity = 96;
  static constexpr std::size_t kWorkerStackBytes = 16 * 1'024;
  static_assert(kWorkerStackBytes % sizeof(StackType_t) == 0);

  struct Command {
    std::array<char, kLineCapacity> text{};
  };

  static void workerEntry(void *context);
  void workerLoop();
  [[nodiscard]] esp_err_t dispatch(char *line);
  void acceptLine();
  void printHelp() const;
  void printStatus() const;

  StreamProtocol &stream_;
  SpiBringup spi_{};
  EncoderBringup encoder_{};
  ImuBringup imu_{};
  CanBringup can_{};
  I2cBringup i2c_{};
  StsBringup sts_{};
  MotorBringup motor_;
  CalibrationBringup calibration_;

  StaticQueue_t queue_storage_{};
  std::array<uint8_t, sizeof(Command)> queue_bytes_{};
  QueueHandle_t queue_{nullptr};
  StaticTask_t worker_storage_{};
  std::array<StackType_t, kWorkerStackBytes / sizeof(StackType_t)>
      worker_stack_{};
  TaskHandle_t worker_{nullptr};
  std::array<char, kLineCapacity> line_{};
  std::size_t line_length_{0};
  bool discarding_line_{false};
  std::atomic<bool> command_running_{false};
  esp_err_t power_initialize_result_{ESP_ERR_INVALID_STATE};
  esp_err_t motor_initialize_result_{ESP_ERR_INVALID_STATE};
  esp_err_t sts_initialize_result_{ESP_ERR_INVALID_STATE};
};

} // 名前空間 bringup
