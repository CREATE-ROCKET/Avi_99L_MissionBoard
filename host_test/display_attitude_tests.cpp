#include "sensors/display_attitude_estimator.hpp"

#include <array>
#include <cassert>
#include <cmath>

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kDegToRad = kPi / 180.0;

bool near(double actual, double expected, double tolerance = 1.0e-6) {
  return std::abs(actual - expected) <= tolerance;
}

std::array<double, 3> gravityForNoseTilt(double tilt_deg) {
  const double tilt = tilt_deg * kDegToRad;
  // vehicle nose = sensor -Z。sensor_upとの内積がcos(tilt)になる静止1gを作る。
  return {std::sin(tilt), 0.0, -std::cos(tilt)};
}

void testGravityDeterminesTiltInsteadOfLauncherConstant() {
  sensors::DisplayAttitudeEstimator estimator;
  const std::array<double, 3> zero_bias{};

  assert(estimator.initialize(1'000'000, zero_bias,
                              gravityForNoseTilt(20.0), 20.0, 280.66));
  assert(estimator.state().valid);
  assert(near(estimator.state().tilt_deg, 20.0));
  assert(near(estimator.state().roll_deg, 0.0));

  sensors::DisplayAttitudeEstimator horizontal;
  assert(horizontal.initialize(1'000'000, zero_bias,
                               gravityForNoseTilt(90.0), 20.0, 280.66));
  assert(horizontal.state().valid);
  assert(near(horizontal.state().tilt_deg, 90.0));

  sensors::DisplayAttitudeEstimator beyond_horizontal;
  assert(beyond_horizontal.initialize(1'000'000, zero_bias,
                                      gravityForNoseTilt(120.0), 20.0,
                                      280.66));
  assert(beyond_horizontal.state().valid);
  assert(near(beyond_horizontal.state().tilt_deg, 120.0));
}

void testRollUsesFixedNegativeSensorZAsVehicleNose() {
  sensors::DisplayAttitudeEstimator estimator;
  const std::array<double, 3> zero_bias{};
  assert(estimator.initialize(1'000'000, zero_bias,
                              gravityForNoseTilt(20.0), 20.0, 280.66));

  // sensor -Z回り+90 deg/sを1秒与える。vehicle nose=-Zなのでdisplay rollは+90 deg。
  assert(estimator.update(2'000'000, {0.0, 0.0, -90.0 * kDegToRad}, true));
  assert(estimator.state().valid);
  assert(near(estimator.state().roll_deg, 90.0));
  assert(near(estimator.state().tilt_deg, 20.0));
}

void testInvalidSampleInvalidatesDisplayAttitude() {
  sensors::DisplayAttitudeEstimator estimator;
  const std::array<double, 3> zero_bias{};
  assert(estimator.initialize(1'000'000, zero_bias,
                              gravityForNoseTilt(20.0), 20.0, 280.66));
  assert(!estimator.update(2'000'000, {}, false));
  assert(!estimator.state().valid);
  assert(estimator.state().invalid_reason ==
         sensors::DisplayAttitudeInvalidReason::sample_invalid);
}

} // namespace

int main() {
  testGravityDeterminesTiltInsteadOfLauncherConstant();
  testRollUsesFixedNegativeSensorZAsVehicleNose();
  testInvalidSampleInvalidatesDisplayAttitude();
  return 0;
}
