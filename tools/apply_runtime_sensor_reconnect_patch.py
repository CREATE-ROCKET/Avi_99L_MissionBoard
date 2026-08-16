#!/usr/bin/env python3
from __future__ import annotations

import hashlib
from pathlib import Path

RUNTIME = Path("src/runtime/production_runtime.cpp")
EXPECTED_BLOB = "05012383d815429c6a33fcbce8a8adb2c8fad032"


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, got {count}")
    return text.replace(old, new, 1)


data = RUNTIME.read_bytes()
actual = git_blob_sha(data)
if actual != EXPECTED_BLOB:
    raise SystemExit(f"runtime blob changed: expected {EXPECTED_BLOB}, got {actual}")
text = data.decode()

text = replace_once(
    text,
    """    if (imu_data_loss_latched || imu_stale) {
      imu_ready.store(false, std::memory_order_release);
      if (attitude.state().valid)
        attitude.invalidateForReset();
      if (imu.initialized() &&
          imu_now_us - last_imu_recovery_attempt_us >= 100'000) {
        last_imu_recovery_attempt_us = imu_now_us;
        (void)imu.end();
        ++timestamp_epoch;
        if (timestamp_epoch == 0)
          timestamp_epoch = 1;
        timestamp_offset_valid = false;
        const esp_err_t restart = imu.begin(spi, true);
        imu_ready.store(restart == ESP_OK, std::memory_order_release);
        if (restart == ESP_OK) {
          imu_data_loss_latched = false;
          last_imu_host_sample_us = imu_now_us;
        }
      }
    } else {
""",
    """    if (imu_data_loss_latched || imu_stale || !imu.initialized()) {
      imu_ready.store(false, std::memory_order_release);
      if (attitude.state().valid)
        attitude.invalidateForReset();
      const uint64_t imu_retry_interval_us =
          detector_state == protocol::MissionState::command_receive
              ? static_cast<uint64_t>(
                    flight_config::kCommandReceiveDeviceReconnectMs) *
                    1'000ULL
              : 100'000ULL;
      if (imu_now_us >= last_imu_recovery_attempt_us &&
          imu_now_us - last_imu_recovery_attempt_us >=
              imu_retry_interval_us) {
        last_imu_recovery_attempt_us = imu_now_us;
        if (imu.initialized())
          (void)imu.end();
        ++timestamp_epoch;
        if (timestamp_epoch == 0)
          timestamp_epoch = 1;
        timestamp_offset_valid = false;
        const esp_err_t restart = imu.begin(spi, true);
        imu_ready.store(restart == ESP_OK, std::memory_order_release);
        if (restart == ESP_OK) {
          imu_data_loss_latched = false;
          last_imu_host_sample_us = imu_now_us;
        }
      }
    } else {
""",
    "ICM initial/reconnect retry",
)

text = replace_once(
    text,
    """  const esp_err_t bus_result = bus.begin(config);
  LPS25HB lps;
  SSCDRRN005PD2A5 ssc;
  esp_err_t lps_result = ESP_ERR_NOT_FOUND;
  esp_err_t ssc_result = ESP_ERR_NOT_FOUND;
  if (bus_result == ESP_OK) {
    LPS25HB::Config lps_config{};
    lps_config.odr = LPS25HB::Odr::hz25;
    lps_config.pressure_average = LPS25HB::PressureAverage::samples8;
    lps_config.temperature_average = LPS25HB::TemperatureAverage::samples8;
""",
    """  esp_err_t bus_result = bus.begin(config);
  LPS25HB lps;
  SSCDRRN005PD2A5 ssc;
  LPS25HB::Config lps_config{};
  lps_config.odr = LPS25HB::Odr::hz25;
  lps_config.pressure_average = LPS25HB::PressureAverage::samples8;
  lps_config.temperature_average = LPS25HB::TemperatureAverage::samples8;
  esp_err_t lps_result = ESP_ERR_NOT_FOUND;
  esp_err_t ssc_result = ESP_ERR_NOT_FOUND;
  if (bus_result == ESP_OK) {
""",
    "make AirData config reusable",
)

text = replace_once(
    text,
    """  uint64_t last_ssc_us = 0;
  uint64_t last_lps_us = 0;
  uint32_t calibration_generation = 0;
""",
    """  uint64_t last_ssc_us = 0;
  uint64_t last_lps_us = 0;
  uint64_t last_reconnect_us = 0;
  uint32_t calibration_generation = 0;
  uint8_t lps_consecutive_errors = 0;
  uint8_t ssc_consecutive_errors = 0;
""",
    "AirData reconnect state",
)

text = replace_once(
    text,
    """  auto setInitialSscError = [&](esp_err_t result) {
""",
    """  auto reconnectAirData = [&](uint64_t now_us) {
    if (now_us < last_reconnect_us ||
        now_us - last_reconnect_us <
            static_cast<uint64_t>(
                flight_config::kCommandReceiveDeviceReconnectMs) *
                1'000ULL)
      return;
    last_reconnect_us = now_us;

    if (!bus.initialized())
      bus_result = bus.begin(config);
    if (!bus.initialized()) {
      lps_ready.store(false, std::memory_order_release);
      ssc_ready.store(false, std::memory_order_release);
      return;
    }

    if (!lps.initialized()) {
      if (bus.probe(0x5C) == ESP_OK)
        lps_result = lps.begin(bus, LPS25HB::Address::low, lps_config);
      else if (bus.probe(0x5D) == ESP_OK)
        lps_result = lps.begin(bus, LPS25HB::Address::high, lps_config);
      else
        lps_result = ESP_ERR_NOT_FOUND;
      lps_ready.store(lps_result == ESP_OK, std::memory_order_release);
      if (lps_result == ESP_OK)
        lps_consecutive_errors = 0;
    }

    if (!ssc.initialized()) {
      ssc_result = bus.probe(0x28) == ESP_OK ? ssc.begin(bus)
                                             : ESP_ERR_NOT_FOUND;
      ssc_ready.store(ssc_result == ESP_OK, std::memory_order_release);
      if (ssc_result == ESP_OK) {
        ssc_consecutive_errors = 0;
        // 再接続後に古い差圧zeroを新device sampleへ暗黙適用しない。
        pressure_conditioner.reset();
        snapshot.ssc_zero_valid = false;
        snapshot.airspeed_valid = false;
      }
    }
  };

  auto setInitialSscError = [&](esp_err_t result) {
""",
    "AirData reconnect helper",
)

text = replace_once(
    text,
    """  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const bool calibration_active =
""",
    """  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    updateMissionSnapshot();
    // LPS/SSC recovery itself may continue in flight. Control loss remains
    // one-shot inhibited by MissionStateMachine and does not re-enter after recovery.
    reconnectAirData(now_us);
    const bool calibration_active =
""",
    "invoke AirData reconnect",
)

text = replace_once(
    text,
    """        if (result == ESP_OK) {
          snapshot.ssc_monotonic_us = now_us;
""",
    """        if (result == ESP_OK) {
          ssc_consecutive_errors = 0;
          snapshot.ssc_monotonic_us = now_us;
""",
    "reset SSC error streak",
)

text = replace_once(
    text,
    """        } else {
          setInitialSscError(result);
        }
""",
    """        } else {
          setInitialSscError(result);
          if (ssc_consecutive_errors != 0xFFU)
            ++ssc_consecutive_errors;
          if (ssc_consecutive_errors >= 3) {
            (void)ssc.end();
            ssc_ready.store(false, std::memory_order_release);
            snapshot.ssc_valid = false;
            snapshot.ssc_zero_valid = false;
            snapshot.airspeed_valid = false;
            pressure_conditioner.reset();
          }
        }
""",
    "SSC disconnect transition",
)

text = replace_once(
    text,
    """      if (read_result == ESP_OK) {
        snapshot.lps_monotonic_us = now_us;
""",
    """      if (read_result == ESP_OK) {
        lps_consecutive_errors = 0;
        snapshot.lps_monotonic_us = now_us;
""",
    "reset LPS error streak",
)

text = replace_once(
    text,
    """      } else {
        setInitialLpsError(read_result);
      }
      snapshot.flight = flight_logic.update(
""",
    """      } else {
        setInitialLpsError(read_result);
        if (lps_consecutive_errors != 0xFFU)
          ++lps_consecutive_errors;
        if (lps_consecutive_errors >= 3) {
          (void)lps.end();
          lps_ready.store(false, std::memory_order_release);
          snapshot.lps_valid = false;
        }
      }
      snapshot.flight = flight_logic.update(
""",
    "LPS disconnect transition",
)

RUNTIME.write_text(text)
print("runtime sensor reconnect patch applied")
