#include <cstdio>

#ifndef AVI_99L_CHARACTERIZATION
#define AVI_99L_CHARACTERIZATION 0
#endif

#if !AVI_99L_CHARACTERIZATION

#include "bringup/bringup_shell.hpp"
#include "bringup/safe_outputs.hpp"
#include "bringup/stream_protocol.hpp"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime/production_runtime.hpp"
#include "runtime/recovery_boot.hpp"

#ifndef MISSION_BRINGUP_SHELL
#define MISSION_BRINGUP_SHELL 0
#endif

namespace {
#if MISSION_BRINGUP_SHELL
bringup::StreamProtocol stream;
bringup::BringupShell shell{stream};
#else
runtime::ProductionRuntime *production_runtime{};
#endif

esp_err_t initializeConsole() {
  if (!usb_serial_jtag_is_driver_installed()) {
    usb_serial_jtag_driver_config_t config{};
    config.tx_buffer_size = 8'192;
    config.rx_buffer_size = 1'024;
    const esp_err_t result = usb_serial_jtag_driver_install(&config);
    if (result != ESP_OK)
      return result;
  }
  usb_serial_jtag_vfs_use_driver();
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
  return ESP_OK;
}

bool printMemoryInfo() {
  uint32_t flash_bytes{};
  const esp_err_t flash_result =
      esp_flash_get_physical_size(nullptr, &flash_bytes);
  const bool psram_ready = esp_psram_is_initialized();
  const std::size_t psram_bytes = psram_ready ? esp_psram_get_size() : 0;
  std::printf("flash=%lu bytes result=%s, psram=%u bytes initialized=%s\n",
              static_cast<unsigned long>(flash_bytes),
              esp_err_to_name(flash_result),
              static_cast<unsigned>(psram_bytes), psram_ready ? "yes" : "no");
  const bool valid = flash_result == ESP_OK && flash_bytes == 16U * 1024U * 1024U &&
                     psram_ready && psram_bytes == 8U * 1024U * 1024U;
  std::printf("memory capacity validation: %s\n", valid ? "PASS" : "FAIL");
  return valid;
}
} // 無名名前空間

extern "C" void app_main() {
  const esp_err_t safe_result = bringup::safe_outputs::initialize();
  if (safe_result != ESP_OK) {
    // 安全GPIOを確立できない限り、他のdriverやtaskを開始しない。
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1'000));
  }

  const esp_err_t console_result = initializeConsole();
  if (console_result != ESP_OK) {
    (void)bringup::safe_outputs::motorCoast();
    (void)bringup::safe_outputs::setAux5v(false);
    (void)bringup::safe_outputs::setParaPower(false);
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1'000));
  }
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  std::printf("\nC-99L Mission Board %s\n",
#if MISSION_BRINGUP_SHELL
              "bring-up"
#else
              "production"
#endif
  );
  std::printf("safe_outputs=%s console=%s\n", esp_err_to_name(safe_result),
              esp_err_to_name(console_result));
  const bool memory_valid = printMemoryInfo();
  if (!memory_valid) {
    (void)bringup::safe_outputs::motorCoast();
    (void)bringup::safe_outputs::setAux5v(false);
    (void)bringup::safe_outputs::setParaPower(false);
    std::printf("容量検証失敗のためtaskを起動せず安全状態を維持します。\n");
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1'000));
  }

#if MISSION_BRINGUP_SHELL
  const esp_err_t stream_result = stream.initialize();
  const esp_err_t shell_result = shell.initialize();
  std::printf("stream=%s shell=%s\n", esp_err_to_name(stream_result),
              esp_err_to_name(shell_result));
  std::printf("actuatorは自動実行されません。helpでcommand一覧を表示します。\n");
  if (stream_result != ESP_OK || shell_result != ESP_OK) {
    (void)bringup::safe_outputs::motorCoast();
    (void)bringup::safe_outputs::setAux5v(false);
    (void)bringup::safe_outputs::setParaPower(false);
    std::printf("初期化または容量検証失敗のためshellを起動せず"
                "安全状態を維持します。\n");
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1'000));
  }
  shell.run();
#else
  const bool marker_valid = runtime::recovery_boot::markerValid();
  const bool wake_cause_valid = runtime::recovery_boot::wakeCauseValid();
  const bool recovery_only = marker_valid || wake_cause_valid;
  static runtime::ProductionRuntime runtime{
      recovery_only, marker_valid && wake_cause_valid};
  production_runtime = &runtime;
  const esp_err_t runtime_result = production_runtime->start();
  std::printf("runtime=%s flight_enabled=%s\n",
              esp_err_to_name(runtime_result),
              production_runtime->flightEnabled() ? "true" : "false");
  if (runtime_result != ESP_OK) {
    (void)bringup::safe_outputs::motorCoast();
    (void)bringup::safe_outputs::setAux5v(false);
    (void)bringup::safe_outputs::setParaPower(false);
    std::printf("production task起動失敗のため安全状態を維持します。\n");
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1'000));
  }
  // runtime taskへ所有権を移したためapp_main taskは終了する。
  vTaskDelete(nullptr);
#endif
}

#endif
