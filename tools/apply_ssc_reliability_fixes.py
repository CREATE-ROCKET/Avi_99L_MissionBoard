from pathlib import Path

path = Path("src/runtime/production_runtime.cpp")
text = path.read_text()

old = '''      } else {
        ssc_ready.store(false, std::memory_order_release);
        ssc_recovery_pending = true;
        setInitialSscError(ESP_ERR_INVALID_STATE);
      }
'''
new = '''      } else {
        ssc_ready.store(false, std::memory_order_release);
        ssc_recovery_pending = true;
        // begin/reconnect失敗の原因をcommand_modeへ潰さず、そのままtelemetryへ残す。
        const esp_err_t unavailable_result =
            last_ssc_reconnect_error == ESP_OK ? ESP_ERR_INVALID_STATE
                                               : last_ssc_reconnect_error;
        setInitialSscError(unavailable_result);
      }
'''

if new in text:
    print("SSC reconnect error telemetry fix already applied")
    raise SystemExit(0)
count = text.count(old)
if count != 1:
    raise SystemExit(f"SSC unavailable block count={count}, expected=1")
path.write_text(text.replace(old, new, 1))
print("preserved SSC begin/reconnect failure reason in telemetry")
