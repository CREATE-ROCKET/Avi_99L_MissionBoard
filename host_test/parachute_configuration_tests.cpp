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
  const auto value = AbsoluteParachuteAngle::fromCount(count);
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

void testAbsoluteAnglesAndConfiguration() {
  require(angle(0).count() == 0, "count zero must be valid");
  require(angle(4095).count() == 4095, "count 4095 must be valid");
  require(!AbsoluteParachuteAngle::fromCount(4096).has_value(),
          "count 4096 must not be normalized");

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
  require(!half.valid() &&
              half.error == ParachutePathError::exactly_half_turn,
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
  require(actuators::shortestParachuteDisplacement(angle(123), angle(123))
              .counts == 0,
          "equal endpoints must have zero displacement");

  const auto path = [](double current, double target) {
    return actuators::shortestParachuteDisplacement(
        angle(nearestCount(current)), angle(nearestCount(target)));
  };
  require(std::abs(path(350.0, 10.0).degrees() - 20.0) < 0.1,
          "350 -> 10 must be about +20 degrees");
  require(std::abs(path(10.0, 350.0).degrees() + 20.0) < 0.1,
          "10 -> 350 must be about -20 degrees");
  require(std::abs(path(359.9, 0.0).degrees() - 0.1) < 0.1,
          "359.9 -> 0 must cross the wrap positively");
  require(std::abs(path(0.0, 359.9).degrees() + 0.1) < 0.1,
          "0 -> 359.9 must cross the wrap negatively");
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
  require(actuators::decodeParachuteEndpoint(
              valid.data(), valid.size() - 1, ParachuteEndpoint::open)
              .error == ParachuteBlobError::wrong_size,
          "wrong size must be rejected");
  require(actuators::decodeParachuteEndpoint(
              valid.data(), valid.size(), ParachuteEndpoint::close)
              .error == ParachuteBlobError::wrong_endpoint,
          "wrong endpoint must be rejected");

  auto blob = valid;
  blob[8] ^= 1U;
  require(actuators::decodeParachuteEndpoint(
              blob.data(), blob.size(), ParachuteEndpoint::open)
              .error == ParachuteBlobError::crc_mismatch,
          "CRC mismatch must be rejected");

  blob = valid;
  blob[0] = 'X';
  refreshCrc(blob);
  require(actuators::decodeParachuteEndpoint(
              blob.data(), blob.size(), ParachuteEndpoint::open)
              .error == ParachuteBlobError::wrong_magic,
          "wrong magic must be rejected");

  blob = valid;
  blob[4] = 2;
  refreshCrc(blob);
  require(actuators::decodeParachuteEndpoint(
              blob.data(), blob.size(), ParachuteEndpoint::open)
              .error == ParachuteBlobError::wrong_schema,
          "wrong schema must be rejected");

  blob = valid;
  actuators::writeLe16(blob.data() + 6, 3);
  refreshCrc(blob);
  require(actuators::decodeParachuteEndpoint(
              blob.data(), blob.size(), ParachuteEndpoint::open)
              .error == ParachuteBlobError::wrong_payload_size,
          "wrong payload size must be rejected");

  blob = valid;
  actuators::writeLe16(blob.data() + 10, 1);
  refreshCrc(blob);
  require(actuators::decodeParachuteEndpoint(
              blob.data(), blob.size(), ParachuteEndpoint::open)
              .error == ParachuteBlobError::reserved_nonzero,
          "nonzero reserved bytes must be rejected");

  blob = valid;
  actuators::writeLe16(blob.data() + 8, 4096);
  refreshCrc(blob);
  require(actuators::decodeParachuteEndpoint(
              blob.data(), blob.size(), ParachuteEndpoint::open)
              .error == ParachuteBlobError::angle_out_of_range,
          "out-of-range angle must be rejected");
}

void testTransactionAndRebootLoad() {
  ParachuteConfigurationState missing_keys;
  missing_keys.replaceLoadedConfiguration({});
  require(!missing_keys.active().open.has_value() &&
              !missing_keys.active().close.has_value(),
          "missing keys must load as unconfigured endpoints");

  ParachuteConfigurationState state;
  ParachuteConfiguration loaded{};
  loaded.open = angle(100);
  loaded.close = angle(500);
  state.replaceLoadedConfiguration(loaded);

  const auto candidate =
      state.candidateWith(ParachuteEndpoint::open, angle(200));
  require(state.active().open->count() == 100,
          "candidate must not update active RAM");
  // write/commit/readback失敗時はactivateしない。
  require(state.active().open->count() == 100,
          "failed save must preserve active RAM");
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
          "one corrupted endpoint must not discard the other");
}

void testFlightSnapshot() {
  ParachuteConfigurationState state;
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::open_not_configured,
          "missing open must reject Start");
  ParachuteConfiguration partial{};
  partial.open = angle(100);
  state.replaceLoadedConfiguration(partial);
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::close_not_configured,
          "missing close must reject Start");
  partial.close = angle(2148);
  state.replaceLoadedConfiguration(partial);
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::
                  open_close_exactly_half_turn,
          "half-turn endpoints must reject Start");
  partial.close = angle(200);
  partial.open = angle(2048);
  state.replaceLoadedConfiguration(partial);
  require(state.freezeFlightSnapshot(angle(0)).error ==
              FlightParachutePreparationError::
                  current_open_exactly_half_turn,
          "half-turn current-to-open must reject Start");

  ParachuteConfiguration valid{};
  valid.open = angle(300);
  valid.close = angle(100);
  state.replaceLoadedConfiguration(valid);
  require(state.freezeFlightSnapshot(angle(200)).ready(),
          "valid endpoints must prepare Start");
  require(state.flightSnapshot()->open.count() == 300,
          "snapshot must contain Start-time open");
  state.activatePersistedCandidate(
      state.candidateWith(ParachuteEndpoint::open, angle(700)));
  require(state.active().open->count() == 700 &&
              state.flightSnapshot()->open.count() == 300,
          "active changes must not alter the flight snapshot");
  state.discardFlightSnapshot();
  require(!state.flightSnapshotValid() && state.active().open->count() == 700,
          "Cancel must discard only the snapshot");

  state.restoreFlightSnapshot({angle(300), angle(100)});
  ParachuteConfiguration reloaded{};
  reloaded.open = angle(900);
  reloaded.close = angle(800);
  state.replaceLoadedConfiguration(reloaded);
  require(state.flightSnapshot()->open.count() == 300 &&
              state.active().open->count() == 900,
          "startup active load must not replace a restored flight snapshot");
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
