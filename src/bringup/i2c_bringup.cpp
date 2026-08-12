#include "bringup/i2c_bringup.hpp"

#include <array>
#include <cinttypes>
#include <cstdio>

#include "I2CCREATE.h"
#include "LPS25HB.h"
#include "SSCDRRN005PD2A5.h"
#include "avi_esp_libs/timeout.h"
#include "config/board_config.hpp"
#include "esp_timer.h"

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

struct ProbeTarget {
  const char *name;
  uint8_t address;
  esp_err_t result{ESP_FAIL};
};

bool missing(const ProbeTarget &target) {
  return target.result == ESP_ERR_NOT_FOUND;
}

esp_err_t verifyLpsCleanup(I2CCREATE &bus, LPS25HB::Address address,
                           const ProbeTarget &target) {
  const std::size_t before = bus.deviceCount();
  esp_err_t operation = ESP_OK;
  {
    LPS25HB sensor;
    const esp_err_t begin_result = sensor.begin(bus, address);
    if (begin_result == ESP_OK) {
      std::printf("%s driver begin: PASS device_count=%zu\n", target.name,
                  bus.deviceCount());
      if (bus.deviceCount() != before + 1)
        rememberFirst(ESP_FAIL, operation);
      const esp_err_t end_result = sensor.end();
      std::printf("%s driver end: %s\n", target.name,
                  esp_err_to_name(end_result));
      rememberFirst(end_result, operation);
    } else if (missing(target) &&
               (begin_result == ESP_ERR_NOT_FOUND ||
                begin_result == ESP_ERR_INVALID_RESPONSE)) {
      std::printf("%s driver begin: SKIP(no device), result=%s\n", target.name,
                  esp_err_to_name(begin_result));
    } else {
      std::printf("%s driver begin: FAIL result=%s\n", target.name,
                  esp_err_to_name(begin_result));
      rememberFirst(begin_result, operation);
    }
  }
  if (bus.deviceCount() != before) {
    std::printf("%s cleanup: FAIL before=%zu after=%zu\n", target.name, before,
                bus.deviceCount());
    rememberFirst(ESP_FAIL, operation);
  } else {
    std::printf("%s cleanup: PASS device_count=%zu\n", target.name, before);
  }
  return operation;
}

esp_err_t verifySscCleanup(I2CCREATE &bus, const ProbeTarget &target) {
  const std::size_t before = bus.deviceCount();
  esp_err_t operation = ESP_OK;
  {
    SSCDRRN005PD2A5 sensor;
    const esp_err_t begin_result = sensor.begin(bus);
    if (begin_result == ESP_OK) {
      std::printf("SSC driver begin: PASS device_count=%zu\n",
                  bus.deviceCount());
      if (bus.deviceCount() != before + 1)
        rememberFirst(ESP_FAIL, operation);
      const esp_err_t end_result = sensor.end();
      std::printf("SSC driver end: %s\n", esp_err_to_name(end_result));
      rememberFirst(end_result, operation);
    } else if (missing(target) && begin_result == ESP_ERR_NOT_FOUND) {
      std::printf("SSC driver begin: SKIP(no device), result=%s\n",
                  esp_err_to_name(begin_result));
    } else {
      std::printf("SSC driver begin: FAIL result=%s\n",
                  esp_err_to_name(begin_result));
      rememberFirst(begin_result, operation);
    }
  }
  if (bus.deviceCount() != before) {
    std::printf("SSC cleanup: FAIL before=%zu after=%zu\n", before,
                bus.deviceCount());
    rememberFirst(ESP_FAIL, operation);
  } else {
    std::printf("SSC cleanup: PASS device_count=%zu\n", before);
  }
  return operation;
}

} // 無名名前空間

esp_err_t I2cBringup::probe() {
  BusyGuard guard(busy_);
  if (!guard.acquired())
    return ESP_ERR_INVALID_STATE;

  I2CCREATE::Config config{};
  config.port = board::kAirDataI2cPort;
  config.sda = board::kAirDataSda;
  config.scl = board::kAirDataScl;
  config.frequency_hz = board::kAirDataI2cFrequencyHz;
  config.enable_internal_pullups = false;
  config.lock_timeout = avi::Timeout::noWait();
  config.operation_timeout =
      avi::Timeout::milliseconds(board::kAirDataOperationTimeoutMs);

  I2CCREATE bus;
  esp_err_t result = bus.begin(config);
  std::printf("I2C begin: %s, frequency=%" PRIu32
              "Hz pullup=external operation_timeout=%" PRIu32 "ms\n",
              esp_err_to_name(result), config.frequency_hz,
              board::kAirDataOperationTimeoutMs);
  if (result != ESP_OK)
    return result;

  std::array<ProbeTarget, 3> targets{{
      {"SSC", 0x28},
      {"LPS-low", 0x5C},
      {"LPS-high", 0x5D},
  }};
  for (auto &target : targets) {
    const int64_t before = esp_timer_get_time();
    target.result = bus.probe(target.address);
    const int64_t latency = esp_timer_get_time() - before;
    const char *classification = target.result == ESP_OK
                                     ? "PASS"
                                     : (missing(target) ? "SKIP(no device)"
                                                        : "FAIL");
    std::printf("I2C probe %s[0x%02X]: %s result=%s latency_us=%" PRId64
                "\n",
                target.name, target.address, classification,
                esp_err_to_name(target.result), latency);
    if (target.result != ESP_OK && !missing(target))
      rememberFirst(target.result, result);
  }

  uint32_t repeated = 0;
  uint32_t repeated_errors = 0;
  int64_t maximum_latency_us = 0;
  for (const auto &target : targets) {
    if (!missing(target))
      continue;
    for (uint32_t attempt = 0; attempt < 100; ++attempt) {
      const int64_t before = esp_timer_get_time();
      const esp_err_t probe_result = bus.probe(target.address);
      const int64_t latency = esp_timer_get_time() - before;
      ++repeated;
      if (latency > maximum_latency_us)
        maximum_latency_us = latency;
      if (probe_result != ESP_ERR_NOT_FOUND) {
        ++repeated_errors;
        rememberFirst(probe_result == ESP_OK ? ESP_FAIL : probe_result, result);
      }
    }
  }
  std::printf("I2C no-device repeat: accesses=%" PRIu32
              " errors=%" PRIu32 " max_latency_us=%" PRId64
              " device_count=%zu\n",
              repeated, repeated_errors, maximum_latency_us,
              bus.deviceCount());
  if (repeated == 0)
    std::printf("I2C no-device repeat: SKIP（全対象addressでdeviceを検出）\n");

  rememberFirst(verifySscCleanup(bus, targets[0]), result);
  rememberFirst(verifyLpsCleanup(bus, LPS25HB::Address::low, targets[1]),
                result);
  rememberFirst(verifyLpsCleanup(bus, LPS25HB::Address::high, targets[2]),
                result);

  if (bus.deviceCount() != 0) {
    std::printf("I2C final device_count: FAIL count=%zu\n", bus.deviceCount());
    rememberFirst(ESP_FAIL, result);
  }
  const esp_err_t end_result = bus.end();
  std::printf("I2C end: %s\n", esp_err_to_name(end_result));
  rememberFirst(end_result, result);
  return result;
}

} // 名前空間 bringup
