#include "actuators/parachute_configuration.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

[[noreturn]] void fail(const char *message) {
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

void require(bool condition, const char *message) {
  if (!condition)
    fail(message);
}

void testRelativeCommandContract() {
  static_assert(actuators::kParachuteOpenRelativeDegrees == -130.0F);
  static_assert(actuators::kParachuteCloseRelativeDegrees == 130.0F);
  require(actuators::kParachuteOpenRelativeDegrees < 0.0F,
          "Open must use the counterclockwise relative direction");
  require(actuators::kParachuteCloseRelativeDegrees > 0.0F,
          "Close must use the clockwise relative direction");
  require(std::abs(actuators::kParachuteOpenRelativeDegrees) == 130.0F &&
              std::abs(actuators::kParachuteCloseRelativeDegrees) == 130.0F,
          "Open/Close must both move exactly 130 degrees");
}

void testPositionTelemetryWrapping() {
  require(actuators::decodeStsSignedMagnitudePositionCount(0x0000U) == 0,
          "STS zero raw must decode to zero");
  require(actuators::decodeStsSignedMagnitudePositionCount(0x1000U) == 4096,
          "STS positive one-turn raw must remain unwrapped");
  require(actuators::decodeStsSignedMagnitudePositionCount(0x8001U) == -1,
          "STS negative one raw must decode signed magnitude");

  const auto one_turn = actuators::AbsoluteParachuteAngle::fromCount(4096);
  require(one_turn.has_value() && one_turn->count() == 0,
          "STS +1 turn must wrap to zero for telemetry");
  const auto negative_one =
      actuators::AbsoluteParachuteAngle::fromCount(0x8001U);
  require(negative_one.has_value() && negative_one->count() == 4095,
          "STS -1 must wrap to 4095 for telemetry");

  const auto current =
      actuators::AbsoluteParachuteAngle::fromCanonicalCount(4095);
  const auto target = actuators::AbsoluteParachuteAngle::fromCanonicalCount(0);
  require(current.has_value() && target.has_value(),
          "test canonical positions must be valid");
  const auto displacement =
      actuators::shortestParachuteDisplacement(*current, *target);
  require(displacement.valid() && displacement.counts == 1,
          "telemetry wrap helper must still cross 4095 to zero correctly");
}

} // 無名名前空間

int main() {
  testRelativeCommandContract();
  testPositionTelemetryWrapping();
  std::cout << "parachute configuration tests: PASS\n";
  return 0;
}
