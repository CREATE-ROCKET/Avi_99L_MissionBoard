from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "src/runtime/production_runtime.cpp"


def replace_once(old: str, new: str) -> None:
    text = PATH.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected one match, got {count}: {old[:120]!r}")
    PATH.write_text(text.replace(old, new, 1), encoding="utf-8")


old_loader = r'''esp_err_t loadParachuteEndpoint(nvs_handle_t handle,
                                actuators::ParachuteEndpoint endpoint,
                                bool &valid, uint16_t &count,
                                bool &corrupted) {
  valid = false;
  corrupted = false;
  std::size_t size = 0;
  esp_err_t result =
      nvs_get_blob(handle, parachuteNvsKey(endpoint), nullptr, &size);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    return ESP_OK;
  if (result != ESP_OK)
    return result;
  if (size != actuators::kParachuteEndpointBlobSize) {
    corrupted = true;
    return ESP_OK;
  }
  actuators::ParachuteEndpointBlob blob{};
  result = nvs_get_blob(handle, parachuteNvsKey(endpoint), blob.data(), &size);
  if (result != ESP_OK)
    return result;
  const auto decoded =
      actuators::decodeParachuteEndpoint(blob.data(), size, endpoint);
  if (!decoded.valid()) {
    corrupted = true;
    return ESP_OK;
  }
  valid = true;
  count = decoded.angle->count();
  return ESP_OK;
}

bool verifyParachuteEndpoint(nvs_handle_t handle,
                             actuators::ParachuteEndpoint endpoint,
                             uint16_t expected_count) {
  bool valid = false;
  bool corrupted = false;
  uint16_t count = 0;
  return loadParachuteEndpoint(handle, endpoint, valid, count, corrupted) ==
             ESP_OK &&
         valid && !corrupted && count == expected_count;
}'''
new_loader = r'''uint16_t parachuteCorruptionDetail(
    actuators::ParachuteEndpoint endpoint,
    actuators::ParachuteBlobError reason) {
  const uint16_t endpoint_bit =
      endpoint == actuators::ParachuteEndpoint::close ? uint16_t{1U << 8U}
                                                       : uint16_t{0};
  return static_cast<uint16_t>(endpoint_bit |
                               static_cast<uint8_t>(reason));
}

uint16_t persistenceRuntimeDetail(esp_err_t error) {
  return static_cast<uint16_t>(
      0x8000U | (static_cast<uint32_t>(error) & 0x7FFFU));
}

esp_err_t loadParachuteEndpoint(
    nvs_handle_t handle, actuators::ParachuteEndpoint endpoint, bool &valid,
    uint16_t &count, actuators::ParachuteBlobError &corruption_reason) {
  valid = false;
  corruption_reason = actuators::ParachuteBlobError::none;
  std::size_t size = 0;
  esp_err_t result =
      nvs_get_blob(handle, parachuteNvsKey(endpoint), nullptr, &size);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    return ESP_OK;
  if (result != ESP_OK)
    return result;
  if (size != actuators::kParachuteEndpointBlobSize) {
    corruption_reason = actuators::ParachuteBlobError::wrong_size;
    return ESP_OK;
  }
  actuators::ParachuteEndpointBlob blob{};
  result = nvs_get_blob(handle, parachuteNvsKey(endpoint), blob.data(), &size);
  if (result != ESP_OK)
    return result;
  const auto decoded =
      actuators::decodeParachuteEndpoint(blob.data(), size, endpoint);
  if (!decoded.valid()) {
    corruption_reason = decoded.error;
    return ESP_OK;
  }
  valid = true;
  count = decoded.angle->count();
  return ESP_OK;
}

bool verifyParachuteEndpoint(nvs_handle_t handle,
                             actuators::ParachuteEndpoint endpoint,
                             uint16_t expected_count) {
  bool valid = false;
  auto corruption_reason = actuators::ParachuteBlobError::none;
  uint16_t count = 0;
  return loadParachuteEndpoint(handle, endpoint, valid, count,
                               corruption_reason) == ESP_OK &&
         valid && corruption_reason == actuators::ParachuteBlobError::none &&
         count == expected_count;
}'''
replace_once(old_loader, new_loader)

old_load_block = r'''  const esp_err_t nvs_result = nvs_flash_init();
  load_response.persistence_ready = nvs_result == ESP_OK;
  if (nvs_result == ESP_OK) {
    nvs_handle_t handle{};
    const esp_err_t open_result =
        nvs_open(kParachuteNvsNamespace, NVS_READONLY, &handle);
    if (open_result == ESP_OK) {
      bool open_corrupted = false;
      bool close_corrupted = false;
      const esp_err_t open_load = loadParachuteEndpoint(
          handle, actuators::ParachuteEndpoint::open,
          load_response.open_valid, load_response.open_count,
          open_corrupted);
      const esp_err_t close_load = loadParachuteEndpoint(
          handle, actuators::ParachuteEndpoint::close,
          load_response.close_valid, load_response.close_count,
          close_corrupted);
      nvs_close(handle);
      load_response.corruption_detected =
          open_corrupted || close_corrupted;
      if (open_corrupted)
        std::printf("parachute NVS open_v1 corrupted\n");
      if (close_corrupted)
        std::printf("parachute NVS close_v1 corrupted\n");
      if (open_load != ESP_OK)
        std::printf("parachute NVS open_v1 load failed: %s\n",
                    esp_err_to_name(open_load));
      if (close_load != ESP_OK)
        std::printf("parachute NVS close_v1 load failed: %s\n",
                    esp_err_to_name(close_load));
      load_response.persistence_ready =
          open_load == ESP_OK && close_load == ESP_OK;
    } else if (open_result != ESP_ERR_NVS_NOT_FOUND) {
      load_response.persistence_ready = false;
    }
  }
  load_response.success = load_response.persistence_ready;
  if (!load_response.persistence_ready ||
      load_response.corruption_detected) {
    enqueueEvent(protocol::eventFlag(
                     protocol::MissionEventFlag::persistence_error),
                 protocol::MissionState::command_receive, 0,
                 static_cast<uint16_t>(nvs_result));
  }'''
new_load_block = r'''  const esp_err_t nvs_result = nvs_flash_init();
  esp_err_t persistence_runtime_error = nvs_result;
  load_response.persistence_ready = nvs_result == ESP_OK;
  if (nvs_result == ESP_OK) {
    nvs_handle_t handle{};
    const esp_err_t open_result =
        nvs_open(kParachuteNvsNamespace, NVS_READONLY, &handle);
    if (open_result == ESP_OK) {
      auto open_corruption = actuators::ParachuteBlobError::none;
      auto close_corruption = actuators::ParachuteBlobError::none;
      const esp_err_t open_load = loadParachuteEndpoint(
          handle, actuators::ParachuteEndpoint::open,
          load_response.open_valid, load_response.open_count,
          open_corruption);
      const esp_err_t close_load = loadParachuteEndpoint(
          handle, actuators::ParachuteEndpoint::close,
          load_response.close_valid, load_response.close_count,
          close_corruption);
      nvs_close(handle);
      load_response.corruption_detected =
          open_corruption != actuators::ParachuteBlobError::none ||
          close_corruption != actuators::ParachuteBlobError::none;
      if (open_corruption != actuators::ParachuteBlobError::none) {
        std::printf("parachute NVS open_v1 corrupted: reason=%u\n",
                    static_cast<unsigned>(open_corruption));
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::persistence_error),
                     protocol::MissionState::command_receive, 0,
                     parachuteCorruptionDetail(
                         actuators::ParachuteEndpoint::open,
                         open_corruption));
      }
      if (close_corruption != actuators::ParachuteBlobError::none) {
        std::printf("parachute NVS close_v1 corrupted: reason=%u\n",
                    static_cast<unsigned>(close_corruption));
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::persistence_error),
                     protocol::MissionState::command_receive, 0,
                     parachuteCorruptionDetail(
                         actuators::ParachuteEndpoint::close,
                         close_corruption));
      }
      if (open_load != ESP_OK) {
        std::printf("parachute NVS open_v1 load failed: %s\n",
                    esp_err_to_name(open_load));
        persistence_runtime_error = open_load;
      }
      if (close_load != ESP_OK) {
        std::printf("parachute NVS close_v1 load failed: %s\n",
                    esp_err_to_name(close_load));
        if (persistence_runtime_error == ESP_OK)
          persistence_runtime_error = close_load;
      }
      load_response.persistence_ready =
          open_load == ESP_OK && close_load == ESP_OK;
    } else if (open_result != ESP_ERR_NVS_NOT_FOUND) {
      load_response.persistence_ready = false;
      persistence_runtime_error = open_result;
    }
  }
  load_response.success = load_response.persistence_ready;
  if (!load_response.persistence_ready) {
    enqueueEvent(protocol::eventFlag(
                     protocol::MissionEventFlag::persistence_error),
                 protocol::MissionState::command_receive, 0,
                 persistenceRuntimeDetail(persistence_runtime_error));
  }'''
replace_once(old_load_block, new_load_block)

replace_once(
    r'''      if (!saved) {
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::persistence_error),
                     protocol::MissionState::command_receive, 0,
                     rollback_failed ? uint16_t{2} : uint16_t{1});
      }''',
    r'''      if (!saved) {
        // bit15=1はblob corruptionではなくpersistence runtime failureを表す。
        enqueueEvent(protocol::eventFlag(
                         protocol::MissionEventFlag::persistence_error),
                     protocol::MissionState::command_receive, 0,
                     rollback_failed ? uint16_t{0x8002}
                                     : uint16_t{0x8001});
      }''',
)

# Descent statusは既存13 bit fieldの予約bitだけを使う。
replace_once(
    r'''          const uint16_t failure = static_cast<uint16_t>(
              parachute_deployment_failure.load(std::memory_order_acquire) & 0x0FU);
          const protocol::DescentCoreTelemetry descent{
              sequences.next(protocol::CanId::descent_core_telemetry),
              static_cast<uint16_t>(0x1F00U | failure),
              static_cast<uint8_t>(
                  protocol::quantization::ParachuteAngleError::unavailable)};''',
    r'''          const uint16_t failure = static_cast<uint16_t>(
              parachute_deployment_failure.load(std::memory_order_acquire) &
              0x0FU);
          const uint16_t persistence_corrupt =
              parachute_persistence_corrupt.load(std::memory_order_acquire)
                  ? uint16_t{1U << 4U}
                  : uint16_t{0};
          const protocol::DescentCoreTelemetry descent{
              sequences.next(protocol::CanId::descent_core_telemetry),
              static_cast<uint16_t>(failure | persistence_corrupt),
              static_cast<uint8_t>(
                  protocol::quantization::ParachuteAngleError::unavailable)};''',
)

# READMEにwire上のreason保持を追記する。
readme = ROOT / "README.md"
text = readme.read_text(encoding="utf-8")
needle = "- `PowerTimeTelemetry.persistence_flags`のbit0/1/2でparachute NVS load完了/runtime ready/corruptを公開します。\n"
addition = needle + "- parachute endpoint corruptionはCRC/schema/size等の`ParachuteBlobError` reasonをMissionEvent detailへ保持し、Descent statusの予約bitには最初のdeployment failureとcorrupt latchを載せます。\n"
if text.count(needle) != 1:
    raise RuntimeError("README persistence reason insertion point not found")
readme.write_text(text.replace(needle, addition, 1), encoding="utf-8")

print("ForceStart Stage 2 persistence reasons and descent status applied")
