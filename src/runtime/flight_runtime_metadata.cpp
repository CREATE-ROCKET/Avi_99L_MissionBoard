#include "runtime/flight_runtime_metadata.hpp"

namespace runtime::flight_runtime_metadata {
namespace {

// MissionRealtimeTaskだけが更新し、同じTaskのflight log serialize直前に読む。
// 1 kHz pathでlockを追加してjitter源を増やさないためsingle-owner契約とする。
Snapshot latest{};

} // namespace

void publishFinZero(const FinZeroMetadata &metadata) { latest.fin_zero = metadata; }

void publishEncoderTiming(const EncoderTimingMetadata &metadata) {
  latest.encoder = metadata;
}

Snapshot snapshot() { return latest; }

} // namespace runtime::flight_runtime_metadata
