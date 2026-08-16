from pathlib import Path

board_path = Path("src/config/board_config.hpp")
runtime_path = Path("src/runtime/production_runtime.cpp")
board = board_path.read_text()
runtime = runtime_path.read_text()

board_done = "kSscRestartConsecutiveFaults" in board
runtime_done = "struct SscDiagnostics" in runtime
if board_done != runtime_done:
    raise SystemExit("SSC reliability patch is only partially applied")
if board_done and runtime_done:
    print("SSC reliability patch already applied")
    raise SystemExit(0)


def once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: replacement count={count}, expected=1")
    return text.replace(old, new, 1)


board = once(
    board,
    "// TODO(HW_TEST): 400 Hz取得時のI2C operation timeoutを実測で確定する\n"
    "constexpr uint32_t kAirDataOperationTimeoutMs = 2;\n",
    "// 400 Hz AirData試験コードと同じ10 ms上限を使用する。正常transactionは\n"
    "// timeoutまで待たず完了するため、2.5 ms取得周期そのものは変更しない。\n"
    "// TODO(HW_TEST): 10 ms上限での連続運転結果を記録し最終値を確定する\n"
    "constexpr uint32_t kAirDataOperationTimeoutMs = 10;\n"
    "// 単発I2C errorやstaleではfreshな直前sampleを即座に捨てず、連続fault時だけ\n"
    "// SSC device handleを再生成する。共有bus/LPSはresetしない。\n"
    "constexpr uint32_t kSscRestartConsecutiveFaults = 8;\n"
    "constexpr uint32_t kSscReconnectIntervalMs = 250;\n",
    "air-data timing constants",
)

runtime = once(
    runtime,
    "  uint64_t last_ssc_us = 0;\n"
    "  uint64_t last_lps_us = 0;\n"
    "  uint32_t calibration_generation = 0;\n",
    "  uint64_t last_ssc_us = 0;\n"
    "  uint64_t last_lps_us = 0;\n"
    "  uint32_t calibration_generation = 0;\n"
    "  struct SscDiagnostics {\n"
    "    uint32_t success{};\n"
    "    uint32_t stale{};\n"
    "    uint32_t command_mode{};\n"
    "    uint32_t diagnostic_fault{};\n"
    "    uint32_t timeout{};\n"
    "    uint32_t i2c_error{};\n"
    "    uint32_t reconnect_attempts{};\n"
    "    uint32_t reconnect_success{};\n"
    "    uint32_t max_read_us{};\n"
    "    uint32_t consecutive_restartable_faults{};\n"
    "  } ssc_diagnostics;\n"
    "  uint64_t next_ssc_reconnect_us =\n"
    "      ssc_result == ESP_OK\n"
    "          ? 0\n"
    "          : static_cast<uint64_t>(esp_timer_get_time()) +\n"
    "                static_cast<uint64_t>(board::kSscReconnectIntervalMs) *\n"
    "                    1'000ULL;\n"
    "  esp_err_t last_ssc_reconnect_error = ssc_result;\n"
    "  bool ssc_recovery_pending = ssc_result != ESP_OK;\n",
    "SSC diagnostics state",
)

runtime = once(
    runtime,
    "  auto setInitialSscError = [&](esp_err_t result) {\n"
    "    if (snapshot.ssc_monotonic_us != 0)\n"
    "      return;\n"
    "    snapshot.airspeed_raw = static_cast<uint8_t>(\n"
    "        result == ESP_ERR_TIMEOUT\n"
    "            ? protocol::quantization::AirspeedError::ssc_i2c_timeout\n"
    "            : protocol::quantization::AirspeedError::ssc_i2c_error);\n"
    "  };\n",
    "  auto setInitialSscError = [&](esp_err_t result) {\n"
    "    if (snapshot.ssc_monotonic_us != 0)\n"
    "      return;\n"
    "    protocol::quantization::AirspeedError error =\n"
    "        protocol::quantization::AirspeedError::ssc_i2c_error;\n"
    "    if (result == ESP_ERR_TIMEOUT)\n"
    "      error = protocol::quantization::AirspeedError::ssc_i2c_timeout;\n"
    "    else if (result == ESP_ERR_NOT_FINISHED)\n"
    "      error = protocol::quantization::AirspeedError::ssc_stale;\n"
    "    else if (result == ESP_ERR_INVALID_STATE)\n"
    "      error = protocol::quantization::AirspeedError::ssc_command_mode;\n"
    "    else if (result == ESP_ERR_INVALID_RESPONSE)\n"
    "      error = protocol::quantization::AirspeedError::ssc_diagnostic_fault;\n"
    "    snapshot.airspeed_raw = static_cast<uint8_t>(error);\n"
    "  };\n",
    "initial SSC error classification",
)

old_helpers_anchor = '''  auto setInitialLpsError = [&](esp_err_t result) {
    if (snapshot.lps_monotonic_us != 0)
      return;
    snapshot.pressure_raw = static_cast<uint16_t>(
        result == ESP_ERR_TIMEOUT
            ? protocol::quantization::LpsPressureError::i2c_timeout
            : protocol::quantization::LpsPressureError::i2c_bus_error);
    snapshot.temperature_raw = static_cast<uint8_t>(
        result == ESP_ERR_TIMEOUT
            ? protocol::quantization::LpsTemperatureError::i2c_timeout
            : protocol::quantization::LpsTemperatureError::i2c_bus_error);
  };

  for (;;) {
'''
new_helpers_anchor = '''  auto setInitialLpsError = [&](esp_err_t result) {
    if (snapshot.lps_monotonic_us != 0)
      return;
    snapshot.pressure_raw = static_cast<uint16_t>(
        result == ESP_ERR_TIMEOUT
            ? protocol::quantization::LpsPressureError::i2c_timeout
            : protocol::quantization::LpsPressureError::i2c_bus_error);
    snapshot.temperature_raw = static_cast<uint8_t>(
        result == ESP_ERR_TIMEOUT
            ? protocol::quantization::LpsTemperatureError::i2c_timeout
            : protocol::quantization::LpsTemperatureError::i2c_bus_error);
  };
  auto incrementSaturated = [](uint32_t &value) {
    if (value != std::numeric_limits<uint32_t>::max())
      ++value;
  };
  auto printSscDiagnostics = [&](const char *prefix, esp_err_t result) {
    std::printf(
        "SSC %s result=%s success=%lu stale=%lu command=%lu diagnostic=%lu "
        "timeout=%lu i2c=%lu reconnect=%lu/%lu consecutive=%lu "
        "max_read_us=%lu\\n",
        prefix, esp_err_to_name(result),
        static_cast<unsigned long>(ssc_diagnostics.success),
        static_cast<unsigned long>(ssc_diagnostics.stale),
        static_cast<unsigned long>(ssc_diagnostics.command_mode),
        static_cast<unsigned long>(ssc_diagnostics.diagnostic_fault),
        static_cast<unsigned long>(ssc_diagnostics.timeout),
        static_cast<unsigned long>(ssc_diagnostics.i2c_error),
        static_cast<unsigned long>(ssc_diagnostics.reconnect_success),
        static_cast<unsigned long>(ssc_diagnostics.reconnect_attempts),
        static_cast<unsigned long>(
            ssc_diagnostics.consecutive_restartable_faults),
        static_cast<unsigned long>(ssc_diagnostics.max_read_us));
  };
  auto recordSscRead = [&](esp_err_t result, uint32_t elapsed_us) {
    if (elapsed_us > ssc_diagnostics.max_read_us)
      ssc_diagnostics.max_read_us = elapsed_us;
    bool restartable = false;
    if (result == ESP_OK) {
      incrementSaturated(ssc_diagnostics.success);
      ssc_diagnostics.consecutive_restartable_faults = 0;
    } else if (result == ESP_ERR_NOT_FINISHED) {
      incrementSaturated(ssc_diagnostics.stale);
      // staleはI2C transportが応答した結果なのでdevice再生成の根拠にしない。
      ssc_diagnostics.consecutive_restartable_faults = 0;
    } else if (result == ESP_ERR_INVALID_STATE) {
      incrementSaturated(ssc_diagnostics.command_mode);
      restartable = true;
    } else if (result == ESP_ERR_INVALID_RESPONSE) {
      incrementSaturated(ssc_diagnostics.diagnostic_fault);
      restartable = true;
    } else if (result == ESP_ERR_TIMEOUT) {
      incrementSaturated(ssc_diagnostics.timeout);
      restartable = true;
    } else {
      incrementSaturated(ssc_diagnostics.i2c_error);
      restartable = true;
    }
    if (restartable)
      incrementSaturated(ssc_diagnostics.consecutive_restartable_faults);
    return restartable &&
           ssc_diagnostics.consecutive_restartable_faults >=
               board::kSscRestartConsecutiveFaults;
  };
  auto attemptSscReconnect = [&](uint64_t now_us, bool restart_initialized) {
    if (bus_result != ESP_OK || now_us < next_ssc_reconnect_us)
      return false;
    if (ssc.initialized() && !restart_initialized)
      return false;
    if (mission_snapshot.deployment_power_cutoff_latched ||
        recovery_requested.load(std::memory_order_acquire))
      return false;

    // SSCだけをbusからdetachする。共有I2C busとLPS25HBは維持する。
    ssc_ready.store(false, std::memory_order_release);
    ssc_recovery_pending = true;
    incrementSaturated(ssc_diagnostics.reconnect_attempts);
    esp_err_t cleanup = ssc.end();
    if (cleanup == ESP_ERR_INVALID_STATE)
      cleanup = ESP_OK;
    esp_err_t reconnect = cleanup;
    if (reconnect == ESP_OK)
      reconnect = ssc.begin(bus);
    next_ssc_reconnect_us =
        now_us + static_cast<uint64_t>(board::kSscReconnectIntervalMs) *
                     1'000ULL;
    if (reconnect == ESP_OK) {
      incrementSaturated(ssc_diagnostics.reconnect_success);
      ssc_diagnostics.consecutive_restartable_faults = 0;
      last_ssc_reconnect_error = ESP_OK;
      // begin/probe成功だけではdataのnormal statusを保証しない。次の正常readまで
      // ssc_ready=falseを維持し、古いairspeedをControlへ再接続しない。
      return true;
    }
    if (reconnect != last_ssc_reconnect_error ||
        ssc_diagnostics.reconnect_attempts == 1)
      printSscDiagnostics("reconnect failed", reconnect);
    last_ssc_reconnect_error = reconnect;
    return false;
  };

  for (;;) {
'''
runtime = once(
    runtime,
    old_helpers_anchor,
    new_helpers_anchor,
    "SSC recovery helpers",
)

runtime = once(
    runtime,
    '      updateMissionSnapshot();\n      if (ssc.initialized()) {\n',
    '      updateMissionSnapshot();\n'
    '      if (!ssc.initialized())\n'
    '        (void)attemptSscReconnect(now_us, false);\n'
    '      if (ssc.initialized()) {\n',
    "SSC reconnect entry",
)

runtime = once(
    runtime,
    '        SSCDRRN005PD2A5::Data data{};\n'
    '        const esp_err_t result = ssc.read(data);\n'
    '        if (result == ESP_OK) {\n'
    '          snapshot.ssc_monotonic_us = now_us;\n',
    '        SSCDRRN005PD2A5::Data data{};\n'
    '        const int64_t read_started_us = esp_timer_get_time();\n'
    '        const esp_err_t result = ssc.read(data);\n'
    '        const int64_t read_elapsed_us =\n'
    '            esp_timer_get_time() - read_started_us;\n'
    '        const uint32_t read_duration_us = static_cast<uint32_t>(\n'
    '            read_elapsed_us > 0 ? read_elapsed_us : 0);\n'
    '        const bool restart_requested =\n'
    '            recordSscRead(result, read_duration_us);\n'
    '        if (result == ESP_OK) {\n'
    '          const bool recovered = ssc_recovery_pending;\n'
    '          ssc_ready.store(true, std::memory_order_release);\n'
    '          ssc_recovery_pending = false;\n'
    '          next_ssc_reconnect_us = 0;\n'
    '          last_ssc_reconnect_error = ESP_OK;\n'
    '          if (recovered)\n'
    '            printSscDiagnostics("recovered", ESP_OK);\n'
    '          snapshot.ssc_monotonic_us = now_us;\n',
    "SSC read timing and recovery completion",
)

runtime = once(
    runtime,
    '        } else {\n'
    '          setInitialSscError(result);\n'
    '        }\n'
    '      } else {\n'
    '        setInitialSscError(ESP_ERR_INVALID_STATE);\n'
    '      }\n'
    '      (void)xQueueOverwrite(air_data_queue, &snapshot);\n',
    '        } else {\n'
    '          setInitialSscError(result);\n'
    '          if (restart_requested) {\n'
    '            ssc_ready.store(false, std::memory_order_release);\n'
    '            ssc_recovery_pending = true;\n'
    '            const uint64_t reconnect_now_us =\n'
    '                static_cast<uint64_t>(esp_timer_get_time());\n'
    '            if (reconnect_now_us >= next_ssc_reconnect_us) {\n'
    '              printSscDiagnostics("restart requested", result);\n'
    '              (void)attemptSscReconnect(reconnect_now_us, true);\n'
    '            }\n'
    '          }\n'
    '        }\n'
    '      } else {\n'
    '        ssc_ready.store(false, std::memory_order_release);\n'
    '        ssc_recovery_pending = true;\n'
    '        setInitialSscError(ESP_ERR_INVALID_STATE);\n'
    '      }\n'
    '      (void)xQueueOverwrite(air_data_queue, &snapshot);\n',
    "SSC read failure handling",
)

runtime = once(
    runtime,
    '  std::printf("AirDataTask bus=%s lps=%s ssc=%s%s\\n",\n'
    '              esp_err_to_name(bus_result), esp_err_to_name(lps_result),\n'
    '              esp_err_to_name(ssc_result),\n'
    '              ssc_result == ESP_ERR_NOT_FOUND ? " (未接続は継続可能)" : "");\n',
    '  std::printf("AirDataTask bus=%s lps=%s ssc=%s timeout_ms=%lu%s\\n",\n'
    '              esp_err_to_name(bus_result), esp_err_to_name(lps_result),\n'
    '              esp_err_to_name(ssc_result),\n'
    '              static_cast<unsigned long>(board::kAirDataOperationTimeoutMs),\n'
    '              ssc_result == ESP_ERR_NOT_FOUND ? " (未接続は継続可能)" : "");\n',
    "AirData startup diagnostics",
)

board_path.write_text(board)
runtime_path.write_text(runtime)
print("applied SSC timeout, diagnostics, and isolated reconnect policy")
