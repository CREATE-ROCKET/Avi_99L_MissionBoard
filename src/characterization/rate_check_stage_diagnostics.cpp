#include "characterization/rate_check_stage_diagnostics.hpp"

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#endif

#include <algorithm>
#include <limits>

namespace avi::characterization {
namespace {

RateCheckStageDiagnostics g_diagnostics{};

std::uint32_t saturateU32(std::uint64_t value) noexcept {
  return static_cast<std::uint32_t>(std::min<std::uint64_t>(
      value, std::numeric_limits<std::uint32_t>::max()));
}

std::uint32_t &stageMaximum(RateCheckStageDiagnostics &diagnostics,
                            RateCheckStage stage) noexcept {
  switch (stage) {
  case RateCheckStage::PowerLatest:
    return diagnostics.power_latest_max_us;
  case RateCheckStage::EncoderDrain:
    return diagnostics.encoder_drain_max_us;
  case RateCheckStage::AssemblerRelease:
    return diagnostics.assembler_release_max_us;
  case RateCheckStage::AngleConvert:
    return diagnostics.angle_convert_max_us;
  case RateCheckStage::RecordValidate:
    return diagnostics.record_validate_max_us;
  case RateCheckStage::WriterEnqueue:
    return diagnostics.writer_enqueue_max_us;
  case RateCheckStage::EncoderRead:
    return diagnostics.encoder_read_max_us;
  }
  return diagnostics.encoder_read_max_us;
}

} // 無名名前空間

void beginRateCheckStageDiagnostics(EncoderRate rate,
                                    RunKind run_kind) noexcept {
  g_diagnostics = {};
  g_diagnostics.rate = rate;
  g_diagnostics.active = run_kind == RunKind::RateCheck;
}

void endRateCheckStageDiagnostics() noexcept { g_diagnostics.active = false; }

bool rateCheckStageDiagnosticsActive() noexcept {
  return g_diagnostics.active;
}

std::uint64_t rateCheckStageNowUs() noexcept {
#if defined(ESP_PLATFORM)
  if (!g_diagnostics.active)
    return 0U;
  const std::int64_t now = esp_timer_get_time();
  return now > 0 ? static_cast<std::uint64_t>(now) : 0U;
#else
  return 0U;
#endif
}

void recordRateCheckStageDuration(RateCheckStage stage,
                                  std::uint64_t duration_us) noexcept {
  if (!g_diagnostics.active)
    return;
  std::uint32_t &maximum = stageMaximum(g_diagnostics, stage);
  maximum = std::max(maximum, saturateU32(duration_us));
}

RateCheckStageDiagnostics rateCheckStageDiagnosticsSnapshot() noexcept {
  return g_diagnostics;
}

RateCheckStageScope::RateCheckStageScope(RateCheckStage stage) noexcept
    : stage_(stage), started_us_(rateCheckStageNowUs()),
      enabled_(started_us_ != 0U) {}

RateCheckStageScope::~RateCheckStageScope() {
  if (!enabled_)
    return;
  const std::uint64_t finished_us = rateCheckStageNowUs();
  if (finished_us >= started_us_)
    recordRateCheckStageDuration(stage_, finished_us - started_us_);
}

} // 名前空間 avi::characterization
