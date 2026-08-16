from pathlib import Path


def replace_once(path: Path, old: str, new: str, label: str) -> bool:
    text = path.read_text()
    if new in text:
        print(f"{label}: already applied")
        return False
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: insertion point count={count}, expected=1")
    path.write_text(text.replace(old, new, 1))
    print(f"{label}: applied")
    return True


changed = False

storage = Path("src/runtime/flight_storage.hpp")
old_storage = '''  static constexpr std::size_t kWriteBatchRecords = 64U;
  static constexpr std::size_t kWriteBatchBytes =
      flight_log::kSerializedRecordBytes * kWriteBatchRecords;
  static constexpr std::size_t kPsramReserveBytes = 512U * 1024U;
  static constexpr std::size_t kMaxPsramStagingBytes = 8U * 1024U * 1024U;

  static_assert(kWriteBatchBytes == 8192U);
'''
new_storage = '''  // 8 KiBをSD writeの上限目標とし、serialized recordを途中で分断しない。
  // schema v1 (128 B)では64 records = 8192 B、schema v2 (192 B)では
  // 42 records = 8064 Bとなる。record size変更時もcompile-timeで追従する。
  static constexpr std::size_t kWriteBatchTargetBytes = 8192U;
  static constexpr std::size_t kWriteBatchRecords =
      kWriteBatchTargetBytes / flight_log::kSerializedRecordBytes;
  static constexpr std::size_t kWriteBatchBytes =
      flight_log::kSerializedRecordBytes * kWriteBatchRecords;
  static constexpr std::size_t kPsramReserveBytes = 512U * 1024U;
  static constexpr std::size_t kMaxPsramStagingBytes = 8U * 1024U * 1024U;

  static_assert(kWriteBatchRecords > 0U);
  static_assert(kWriteBatchBytes <= kWriteBatchTargetBytes);
  static_assert(kWriteBatchTargetBytes - kWriteBatchBytes <
                flight_log::kSerializedRecordBytes);
'''
changed |= replace_once(storage, old_storage, new_storage,
                        "psram SD batch sizing")

runtime = Path("src/runtime/production_runtime.cpp")
old_state = '''    if (mission_snapshot.state == protocol::MissionState::command_receive &&
        command_fin_auto_hold_allowed && command_fin_mode == CommandFinMode::free &&
        fin_observation_valid &&
'''
new_state = '''    if (detector_state == protocol::MissionState::command_receive &&
        command_fin_auto_hold_allowed && command_fin_mode == CommandFinMode::free &&
        fin_observation_valid &&
'''
changed |= replace_once(runtime, old_state, new_state,
                        "CommandReceive fin auto-hold state")

old_unused = '''bool estimatePreflightGyroBias(const sensors::GyroHistoryRing &history,
                               uint64_t liftoff_time_us, double &bias) {
  constexpr uint64_t kBiasWindowUs = 1'000'000;
  const uint64_t first_time = liftoff_time_us > kBiasWindowUs
                                  ? liftoff_time_us - kBiasWindowUs
                                  : 0;
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t index = 0; index < history.size(); ++index) {
    const auto &sample = history.at(index);
    if (sample.timestamp_us < first_time ||
        sample.timestamp_us > liftoff_time_us || !sample.valid ||
        sample.saturated || sample.format_fault || sample.lost_packets != 0)
      continue;
    sum += sample.roll_rate_rad_s;
    ++count;
  }
  // TODO(SIMULATION): 1秒/500 sample条件をpreflight noiseで再評価する。
  if (count < 500)
    return false;
  bias = sum / static_cast<double>(count);
  return true;
}

'''
text = runtime.read_text()
if old_unused in text:
    runtime.write_text(text.replace(old_unused, "", 1))
    changed = True
    print("unused preflight gyro-bias helper: removed")
elif "estimatePreflightGyroBias(" in text:
    raise SystemExit("unused preflight gyro-bias helper: unexpected remaining form")
else:
    print("unused preflight gyro-bias helper: already absent")

if not changed:
    print("no source changes required")
