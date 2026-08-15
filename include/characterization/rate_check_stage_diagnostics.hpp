#pragma once

#include "characterization/characterization_types.hpp"

#include <cstdint>

namespace avi::characterization {

enum class RateCheckStage : std::uint8_t {
  PowerLatest = 0,
  EncoderDrain,
  AssemblerRelease,
  AngleConvert,
  RecordValidate,
  WriterEnqueue,
  EncoderRead,
};

struct RateCheckStageDiagnostics {
  bool active{false};
  EncoderRate rate{EncoderRate::Hz1000};
  std::uint32_t power_latest_max_us{0U};
  std::uint32_t encoder_drain_max_us{0U};
  std::uint32_t assembler_release_max_us{0U};
  std::uint32_t angle_convert_max_us{0U};
  std::uint32_t record_validate_max_us{0U};
  std::uint32_t writer_enqueue_max_us{0U};
  std::uint32_t encoder_read_max_us{0U};
};

// consumer側diagnostic stateはchar_runtimeだけが更新・取得する。
// EncoderReadだけはEncoderSampler task内で最大値を保持し、stop後にrecordする。
void beginRateCheckStageDiagnostics(EncoderRate rate,
                                    RunKind run_kind) noexcept;
void endRateCheckStageDiagnostics() noexcept;
[[nodiscard]] bool rateCheckStageDiagnosticsActive() noexcept;
[[nodiscard]] std::uint64_t rateCheckStageNowUs() noexcept;
void recordRateCheckStageDuration(RateCheckStage stage,
                                  std::uint64_t duration_us) noexcept;
[[nodiscard]] RateCheckStageDiagnostics
rateCheckStageDiagnosticsSnapshot() noexcept;

class RateCheckStageScope {
public:
  explicit RateCheckStageScope(RateCheckStage stage) noexcept;
  ~RateCheckStageScope();

  RateCheckStageScope(const RateCheckStageScope &) = delete;
  RateCheckStageScope &operator=(const RateCheckStageScope &) = delete;

private:
  RateCheckStage stage_;
  std::uint64_t started_us_{0U};
  bool enabled_{false};
};

} // 名前空間 avi::characterization
