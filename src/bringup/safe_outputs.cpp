#include "bringup/safe_outputs.hpp"

#include <array>

#include "config/board_config.hpp"
#include "driver/gpio.h"

namespace bringup::safe_outputs {
namespace {
bool ready = false;

void rememberFirst(esp_err_t next, esp_err_t &first) {
  if (first == ESP_OK && next != ESP_OK)
    first = next;
}

esp_err_t configureLow(gpio_num_t pin) {
  esp_err_t result = gpio_reset_pin(pin);
  if (result == ESP_OK)
    result = gpio_set_level(pin, 0);
  if (result == ESP_OK)
    // statusでpad levelも読めるよう入力回路を有効にしたまま出力する。
    result = gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
  if (result == ESP_OK)
    result = gpio_set_level(pin, 0);
  return result;
}
} // 無名名前空間

esp_err_t initialize() {
  esp_err_t first = ESP_OK;
  for (const gpio_num_t pin :
       std::array{board::kAux5vEnable, board::kParaEnable,
                  board::kMotorIn2, board::kMotorIn1})
    rememberFirst(configureLow(pin), first);
  ready = first == ESP_OK;
  return first;
}

esp_err_t motorCoast() {
  esp_err_t first = gpio_set_level(board::kMotorIn1, 0);
  rememberFirst(gpio_set_level(board::kMotorIn2, 0), first);
  return first;
}

esp_err_t setAux5v(bool enabled) {
  if (enabled && !ready)
    return ESP_ERR_INVALID_STATE;
  return gpio_set_level(board::kAux5vEnable, enabled ? 1 : 0);
}

esp_err_t setParaPower(bool enabled) {
  if (enabled && !ready)
    return ESP_ERR_INVALID_STATE;
  return gpio_set_level(board::kParaEnable, enabled ? 1 : 0);
}

bool initialized() { return ready; }

} // 名前空間 bringup::safe_outputs
