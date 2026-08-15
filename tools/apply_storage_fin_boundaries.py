from pathlib import Path


def replace_once(old: str, new: str) -> None:
    path = Path("src/runtime/production_runtime.cpp")
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected one match, got {count}: {old[:100]!r}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_once(
    "          fin_zero_available = true;\n"
    "          fin_angle_rad = 0.0;\n"
    "          fin_zero_configured.store(true, std::memory_order_release);\n"
    "          fin_zero_hold_valid.store(false, std::memory_order_release);\n"
    "          zero_hold_controller.resetValidity();",
    "          fin_zero_available = true;\n"
    "          fin_angle_rad = 0.0;\n"
    "          fin_rate_valid = false;\n"
    "          fin_velocity.reset();\n"
    "          fin_zero_configured.store(true, std::memory_order_release);\n"
    "          fin_zero_hold_valid.store(false, std::memory_order_release);\n"
    "          zero_hold_controller.resetValidity();",
)

replace_once(
    "        if (fin_zero_available)\n"
    "          fin_angle_rad = unwrapped_fin_rad - fin_zero_reference_rad;\n"
    "        fin_rate_valid = fin_velocity.update(sample.host_timestamp_us,\n"
    "                                             fin_angle_rad,\n"
    "                                             fin_rate_rad_s);",
    "        if (fin_zero_available) {\n"
    "          fin_angle_rad = unwrapped_fin_rad - fin_zero_reference_rad;\n"
    "          fin_rate_valid = fin_velocity.update(sample.host_timestamp_us,\n"
    "                                               fin_angle_rad,\n"
    "                                               fin_rate_rad_s);\n"
    "        } else {\n"
    "          // zero設定前の仮の0 rad系列を速度推定historyへ混ぜない。\n"
    "          fin_rate_valid = false;\n"
    "          fin_velocity.reset();\n"
    "        }",
)

replace_once(
    "  RecoveryDumpCursor sd_dump{};\n"
    "  for (;;) {\n"
    "    StorageExportRequest export_request{};\n"
    "    while (xQueueReceive(storage_export_queue, &export_request, 0) == pdTRUE) {\n"
    "      protocol::CommandReason reason = protocol::CommandReason::none;\n"
    "      esp_err_t export_result = ESP_ERR_INVALID_STATE;\n"
    "      if (!sd_flight_log.ready() || !internal_flash_log.ready()) {\n"
    "        reason = protocol::CommandReason::device_unavailable;\n"
    "      } else {\n"
    "        export_result =\n"
    "            sd_flight_log.exportRawFlashAndErase(internal_flash_log);\n"
    "        if (export_result != ESP_OK)\n"
    "          reason = protocol::CommandReason::persistence_error;\n"
    "      }\n"
    "      if (reason == protocol::CommandReason::none)\n"
    "        flash_log_ready.store(true, std::memory_order_release);\n"
    "      if (xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {\n"
    "        const auto result = command_executor.finish(\n"
    "            export_request.transaction_id,\n"
    "            reason == protocol::CommandReason::none\n"
    "                ? protocol::CommandPhase::completed\n"
    "                : protocol::CommandPhase::failed,\n"
    "            reason, static_cast<uint32_t>(export_result));\n"
    "        xSemaphoreGive(executor_mutex);\n"
    "        enqueueResult(result, false);\n"
    "      } else {\n"
    "        (void)xQueueSendToFront(storage_export_queue, &export_request, 0);\n"
    "        break;\n"
    "      }\n"
    "    }",
    "  RecoveryDumpCursor sd_dump{};\n"
    "  struct StorageExportCompletion {\n"
    "    bool pending{};\n"
    "    uint8_t transaction_id{};\n"
    "    protocol::CommandReason reason{protocol::CommandReason::none};\n"
    "    uint32_t detail{};\n"
    "  } export_completion;\n"
    "  for (;;) {\n"
    "    if (export_completion.pending &&\n"
    "        xSemaphoreTake(executor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {\n"
    "      const auto result = command_executor.finish(\n"
    "          export_completion.transaction_id,\n"
    "          export_completion.reason == protocol::CommandReason::none\n"
    "              ? protocol::CommandPhase::completed\n"
    "              : protocol::CommandPhase::failed,\n"
    "          export_completion.reason, export_completion.detail);\n"
    "      xSemaphoreGive(executor_mutex);\n"
    "      enqueueResult(result, false);\n"
    "      export_completion = {};\n"
    "    }\n\n"
    "    if (!export_completion.pending) {\n"
    "      StorageExportRequest export_request{};\n"
    "      if (xQueueReceive(storage_export_queue, &export_request, 0) == pdTRUE) {\n"
    "        protocol::CommandReason reason = protocol::CommandReason::none;\n"
    "        esp_err_t export_result = ESP_ERR_INVALID_STATE;\n"
    "        if (!sd_flight_log.ready() || !internal_flash_log.ready()) {\n"
    "          reason = protocol::CommandReason::device_unavailable;\n"
    "        } else {\n"
    "          export_result =\n"
    "              sd_flight_log.exportRawFlashAndErase(internal_flash_log);\n"
    "          if (export_result != ESP_OK)\n"
    "            reason = protocol::CommandReason::persistence_error;\n"
    "        }\n"
    "        if (reason == protocol::CommandReason::none)\n"
    "          flash_log_ready.store(true, std::memory_order_release);\n"
    "        // export/eraseは一度だけ実行し、result通知だけを必要ならretryする。\n"
    "        export_completion =\n"
    "            {true, export_request.transaction_id, reason,\n"
    "             static_cast<uint32_t>(export_result)};\n"
    "      }\n"
    "    }",
)

print("storage/FinZero boundary fixes applied")
