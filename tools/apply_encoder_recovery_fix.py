#!/usr/bin/env python3
from pathlib import Path

path = Path("src/bringup/encoder_bringup.cpp")
text = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected 1 match, got {count}")
    text = text.replace(old, new, 1)

replace_once(
    """    result = sensors::as5047d_health::validateStatus(
        encoder_.getStatus(status), status);
  }
  if (result == ESP_OK)
""",
    """    result = sensors::as5047d_health::validateStatus(
        encoder_.getStatus(status), status);
    if (result == ESP_OK && sensors::as5047d_health::statusFaulted(status))
      result = ESP_ERR_INVALID_RESPONSE;
  }
  if (result == ESP_OK)
""",
    "recovery health gate",
)

replace_once(
    """esp_err_t EncoderBringup::begin(SpiBringup &spi) {
  ExclusiveGuard guard{busy_};
  return guard.acquired() ? beginImpl(spi) : ESP_ERR_INVALID_STATE;
}
""",
    """esp_err_t EncoderBringup::begin(SpiBringup &spi) {
  ExclusiveGuard guard{busy_};
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;
  const esp_err_t result = beginImpl(spi);
  if (result != ESP_OK)
    scheduleRecovery();
  return result;
}
""",
    "initial begin recovery",
)

replace_once(
    """  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

  if (recovery_required_) {
""",
    """  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

  // readPipelined()を呼ぶruntimeはpipeline継続を要求している。
  // boot時begin/status失敗後もこの要求を記録して1秒retryへ入る。
  pipeline_requested_ = true;
  if (recovery_required_) {
""",
    "pipelined demand after boot failure",
)

path.write_text(text)
print("encoder recovery fix applied")
