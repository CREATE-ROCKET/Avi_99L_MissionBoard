#include "actuators/parachute_configuration.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

using actuators::AbsoluteParachuteAngle;
using actuators::FlightParachutePreparationError;
using actuators::ParachuteBlobError;
using actuators::ParachuteConfiguration;
using actuators::ParachuteConfigurationState;
using actuators::ParachuteEndpoint;
using actuators::ParachuteEndpointBlob;
using actuators::ParachutePathError;

[[noreturn]] void fail(const char *message) {
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

void require(bool condition, const char *message) {
  if (!condition)
    fail(message);
}

AbsoluteParachuteAngle angle(uint16_t count) {
  const auto value = AbsoluteParachuteAngle::fromCanonicalCount(count);
  require(value.has_value(), "test angle must be valid");
  return *value;
}

uint16_t nearestCount(double degrees) {
  return static_cast<uint16_t>(std::lround(
      degrees / actuators::kParachuteDegreesPerCount));
}

void refreshCrc(ParachuteEndpointBlob &blob) {
  actuators::writeLe32(blob.data() + 12,
                       actuators::parachuteCrc32(blob.data(), 12));
}

void requireCorruptNullopt(const ParachuteEndpointBlob &blob,
                           ParachuteEndpoint endpoint,
                           ParachuteBlobError expected,
                           const char *message) {
  const auto decoded = actuators::decodeParachuteEndpoint(
      blob.data(), blob.size(), endpoint);
  require(decoded.error == expected && !decoded.angle.has_value(), message);
}

void testAbsoluteAnglesAndConfiguration() {
  require(angle(0).count() == 0, "count zero must be valid");
  require(angle(4095).count() == 4095, "count 4095 must be valid");
  require(!AbsoluteParachuteAngle::fromCanonicalCount(4096).has_value(),
          "canonical count 4096 must remain invalid");
  require(actuators::decodeStsSignedMagnitudePositionCount(0x0000U) == 0,
          "STS zero raw must decode to zero");
  require(actuators::decodeStsSignedMagnitudePositionCount(0x1000U) == 4096,
          "STS positive one-turn raw must remain unwrapped");
  require(actuators::decodeStsSignedMagnitudePositionCount(0x8001U) == -1,
          "STS negative one raw must decode signed magnitude");
  require(actuators::decodeStsSignedMagnitudePositionCount(0x9000U) == -4096,
          "STS negative one-turn raw must remain unwrapped");

  const auto one_turn = AbsoluteParachuteAngle::fromCount(4096);
  require(one_turn.has_value() && one_turn->count() == 0,
          "STS +1 turn must wrap to zero");
  const auto positive_multi_turn = AbsoluteParachuteAngle::fromCount(
      static_cast<uint16_t>(7 * actuators::kParachuteCountsPerRevolution +
                            321));
  require(positive_multi_turn.has_value() &&
              positive_multi_turn->count() == 321,
          "STS positive multi-turn position must wrap to one turn");
  const auto negative_one = AbsoluteParachuteAngle::fromCount(0x8001U);
  require(negative_one.has_value() && negative_one->count() == 4095,
          "STS signed-magnitude -1 must wrap to 4095");
  const auto negative_turn = AbsoluteParachuteAngle::fromCount(0x9000U);
  require(negative_turn.has_value() && negative_turn->count() == 0,
          "STS signed-magnitude -4096 must wrap to zero");
  const auto negative_4095 = AbsoluteParachuteAngle::fromCount(0x8FFFU);
  require(negative_4095.has_value() && negative_4095->count() == 1,
          "STS signed-magnitude -4095 must wrap to one");

  ParachuteConfiguration configuration{};
  require(!configuration.openConfigured() && !configuration.closeConfigured(),
          "both endpoints must start unconfigured");
  configuration.open = angle(1);
  require(configuration.openConfigured() && !configuration.closeConfigured(),
          "open-only configuration must be represented");
  configuration = {};
  configuration.close = angle(2);
  require(!configuration.openConfigured() && configuration.closeConfigured(),
          "close-only configuration must be represented");
  configuration.open = angle(1);
  require(configuration.openConfigured() && configuration.closeConfigured(),
          "both endpoints must be represented independently");
}

void testShortestPath() {
  const auto forward =
      actuators::shortestParachuteDisplacement(angle(0), angle(2047));
  require(forward.valid() && forward.counts == 2047,
          "0 -> 2047 must be positive");
  const auto half =
      actuators::shortestParachuteDisplacement(angle(0), angle(2048));
  require(!half.valid() && half.error == ParachutePathError::exactly_half_turn,
          "0 -> 2048 must be rejected");
  const auto reverse =
      actuators::shortestParachuteDisplacement(angle(0), angle(2049));
  require(reverse.valid() && reverse.counts == -2047,
          "0 -> 2049 must be negative");
  require(actuators::shortestParachuteDisplacement(angle(2048), angle(0))
              .error == ParachutePathError::exactly_half_turn,
          "2048 -> 0 must be rejected");
  require(actuators::shortestParachuteDisplacement(angle(4095), angle(0))
              .counts == 1,
          "4095 -> 0 must be +1");
  require(actuators::shortestParachuteDisplacement(angle(0), angle(4095))
              .counts == -1,
          "0 -> 4095 must be -1");

  const auto path = [](double current, double target) {
    return actuators::shortestParachuteDisplacement(
        angle(nearestCount(current)), angle(nearestCount(target)));
  };
  require(std::abs(path(350.0, 10.0).degrees() - 20.0) < 0.1,
          "350 -> 10 must be about +20 degrees");
  require(std::abs(path(10.0, 350.0).degrees() + 20.0) < 0.1,
          "10 -> 350 must be about -20 degrees");
  require(path(0.0, 179.9).valid(), "179.9 degrees must remain valid");
  require(!path(0.0, 180.0).valid(), "180 degrees must be rejected");
}

void testBlobValidation() {
  const auto valid = actuators::encodeParachuteEndpoint(
      ParachuteEndpoint::open, angle(4095));
  const auto decoded = actuators::decodeParachuteEndpoint(
      valid.data(), valid.size(), ParachuteEndpoint::open);
  require(decoded.valid() && decoded.angle->count() == 4095,
          "valid blob must round-trip");
  const auto wrong_size = actuators::decodeParachuteEndpoint(
      valid.data(), valid.size() - 1, ParachuteEndpoint::open);
  require(wrong_size.error == ParachuteBlobError::wrong_size &&
              !wrong_size.angle.has_value(),
          "wrong size must be operational nullopt");
  const auto wrong_endpoint = actuators::decodeParachuteEndpoint(
      valid.data(), valid.size(), ParachuteEndpoint::close);
  require(wrong_endpoint.error == ParachuteBlobError::wrong_endpoint &&
              !wrong_endpoint.angle.has_value(),
          "wrong endpoint must be operational nullopt");

  auto blob = valid;
  blob[8] ^= 1U;
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::crc_mismatch,
                        "CRC mismatch must preserve corrupt reason");

  blob = valid;
  blob[0] = 'X';
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::wrong_magic,
                        "wrong magic must preserve corrupt reason");

  blob = valid;
  blob[4] = 2;
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::wrong_schema,
                        "wrong schema must preserve corrupt reason");

  blob = valid;
  actuators::writeLe16(blob.data() + 6, 3);
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::wrong_payload_size,
                        "wrong payload size must preserve corrupt reason");

  blob = valid;
  actuators::writeLe16(blob.data() + 10, 1);
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::reserved_nonzero,
                        "nonzero reserved must preserve corrupt reason");

  blob = valid;
  actuators::writeLe16(blob.data() + 8, 4096);
  refreshCrc(blob);
  requireCorruptNullopt(blob, ParachuteEndpoint::open,
                        ParachuteBlobError::angle_out_of_range,
                        "out-of-range stored count must not be normalized");
}

void testTransactionAndRebootLoad() {
  ParachuteConfigurationState missing_keys;
  missing_keys.replaceLoadedConfiguration({});
  require(!missing_keys.active().open.has_value() &&
              !missing_keys.active().close.has_value(),
          "missing keys must load as independent nullopt endpoints");

  ParachuteConfigurationState state;
  ParachuteConfiguration loaded{};
  loaded.open = angle(100);
  loaded.close = angle(500);
  state.replaceLoadedConfiguration(loaded);
  const auto candidate =
      state.candidateWith(ParachuteEndpoint::open, angle(200));
  require(state.active().open->count() == 100,
          "candidate must not update active RAM");
  state.activatePersistedCandidate(candidate);
  require(state.active().open->count() == 200,
          "verified save may update active RAM");

  const auto open_blob = actuators::encodeParachuteEndpoint(
      ParachuteEndpoint::open, angle(321));
  auto close_blob = actuators::encodeParachuteEndpoint(
      ParachuteEndpoint::close, angle(654));
  close_blob[8] ^= 1U;
  ParachuteConfiguration rebooted_configuration{};
  const auto open = actuators::decodeParachuteEndpoint(
      open_blob.data(), open_blob.size(), ParachuteEndpoint::open);
  const auto close = actuators::decodeParachuteEndpoint(
      close_blob.data(), close_blob.size(), ParachuteEndpoint::close);
  if (open.valid())
    rebooted_configuration.open = open.angle;
  if (close.valid())
    rebooted_configuration.close = close.angle;
  ParachuteConfigurationState rebooted;
  rebooted.replaceLoadedConfiguration(rebooted_configuration);
  require(rebooted.active().open->count() == 321 &&
              !rebooted.active().close.has_value(),
          "one corrupt endpoint must not discard the other");
  require(close.error == ParachuteBlobError::crc_mismatch,
          "corrupt reason must remain observable separately from nullopt");
}

void testFlightSnapshot() {
  ParachuteConfigurationState state;
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::open_not_configured,
          "normal Start must require Open");

  ParachuteConfiguration open_only{};
  open_only.open = angle(100);
  state.replaceLoadedConfiguration(open_only);
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::close_not_configured,
          "normal Start must require Close");
  state.freezeFlightSnapshotForced();
  require(state.flightSnapshotValid() &&
              state.flightSnapshot()->open.has_value() &&
              !state.flightSnapshot()->close.has_value() &&
              state.flightSnapshot()->open->count() == 100,
          "Force snapshot must preserve Open-only configuration");

  ParachuteConfiguration close_only{};
  close_only.close = angle(200);
  state.replaceLoadedConfiguration(close_only);
  state.freezeFlightSnapshotForced();
  require(!state.flightSnapshot()->open.has_value() &&
              state.flightSnapshot()->close.has_value() &&
              state.flightSnapshot()->close->count() == 200,
          "Force snapshot must preserve Close-only configuration");

  state.replaceLoadedConfiguration({});
  state.freezeFlightSnapshotForced();
  require(!state.flightSnapshot()->open.has_value() &&
              !state.flightSnapshot()->close.has_value(),
          "Force snapshot must preserve both-null configuration");

  ParachuteConfiguration mutual_half_turn{};
  mutual_half_turn.open = angle(100);
  mutual_half_turn.close = angle(2148);
  state.replaceLoadedConfiguration(mutual_half_turn);
  require(state.freezeFlightSnapshot(angle(0)).ready(),
          "Open/Close mutual half-turn must not reject normal Start");
  require(state.flightSnapshot()->open->count() == 100 &&
              state.flightSnapshot()->close->count() == 2148,
          "normal snapshot must freeze both endpoints");

  require(state.freezeFlightSnapshot(angle(2148)).error ==
              FlightParachutePreparationError::current_open_exactly_half_turn,
          "actual current-to-Open half-turn must reject normal Start");

  ParachuteConfiguration valid{};
  valid.open = angle(300);
  valid.close = angle(100);
  state.replaceLoadedConfiguration(valid);
  require(state.freezeFlightSnapshot(angle(200)).ready(),
          "valid endpoints must prepare normal Start");
  require(state.flightSnapshot()->open->count() == 300,
          "snapshot must contain Start-time Open");
  state.activatePersistedCandidate(
      state.candidateWith(ParachuteEndpoint::open, angle(700)));
  require(state.active().open->count() == 700 &&
              state.flightSnapshot()->open->count() == 300,
          "active changes must not alter flight snapshot");
  state.discardFlightSnapshot();
  require(!state.flightSnapshotValid() && state.active().open->count() == 700,
          "Cancel must discard only flight snapshot");

  actuators::FlightParachuteConfiguration restored{};
  restored.open = angle(333);
  state.restoreFlightSnapshot(restored);
  ParachuteConfiguration reloaded{};
  reloaded.open = angle(900);
  reloaded.close = angle(800);
  state.replaceLoadedConfiguration(reloaded);
  require(state.flightSnapshot()->open->count() == 333 &&
              !state.flightSnapshot()->close.has_value() &&
              state.active().open->count() == 900,
          "RTC optional snapshot must remain independent of active NVS config");
}

} // 無名名前空間

int main() {
  testAbsoluteAnglesAndConfiguration();
  testShortestPath();
  testBlobValidation();
  testTransactionAndRebootLoad();
  testFlightSnapshot();
  std::cout << "parachute configuration tests: PASS\n";
  return 0;
}
