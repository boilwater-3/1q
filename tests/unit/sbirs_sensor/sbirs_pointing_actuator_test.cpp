#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sbirs_sensor/pipeline/SbirsPointingActuator.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

session::SbirsVector3M Los(double azimuth_deg, double elevation_deg) {
  const double azimuth_rad = azimuth_deg * kPi / 180.0;
  const double elevation_rad = elevation_deg * kPi / 180.0;
  const double horizontal = std::cos(elevation_rad);
  return {horizontal * std::cos(azimuth_rad), horizontal * std::sin(azimuth_rad),
          std::sin(elevation_rad)};
}

double AzimuthDeg(const session::SbirsVector3M& los) {
  return std::atan2(los.y, los.x) * 180.0 / kPi;
}

void ExpectLosNear(const session::SbirsVector3M& left, const session::SbirsVector3M& right,
                   double tolerance) {
  EXPECT_NEAR(left.x, right.x, tolerance);
  EXPECT_NEAR(left.y, right.y, tolerance);
  EXPECT_NEAR(left.z, right.z, tolerance);
}

SbirsPointingActuatorConfig Config(double rate = 30.0, double tolerance = 0.01) {
  SbirsPointingActuatorConfig config;
  config.max_slew_rate_deg_per_sec = rate;
  config.settle_tolerance_deg = tolerance;
  return config;
}

TEST(SbirsPointingActuatorTest, ZeroAngleIsImmediatelySettled) {
  SbirsPointingActuator actuator;
  ASSERT_TRUE(actuator.Initialize(Los(10.0, 20.0)));
  SbirsPointingActuatorResult result;
  ASSERT_TRUE(actuator.Step(Los(10.0, 20.0), 1.0, Config(), &result));
  EXPECT_TRUE(result.settled);
  EXPECT_NEAR(result.remaining_angle_deg, 0.0, 1.0e-9);
}

TEST(SbirsPointingActuatorTest, SlewRateLimitsProgressAndPreventsOvershoot) {
  SbirsPointingActuator actuator;
  ASSERT_TRUE(actuator.Initialize(Los(0.0, 0.0)));
  SbirsPointingActuatorResult result;
  ASSERT_TRUE(actuator.Step(Los(90.0, 0.0), 1.0, Config(30.0), &result));
  EXPECT_NEAR(AzimuthDeg(result.current_los), 30.0, 1.0e-9);
  EXPECT_NEAR(result.remaining_angle_deg, 60.0, 1.0e-9);
  EXPECT_FALSE(result.settled);

  ASSERT_TRUE(actuator.Step(Los(90.0, 0.0), 3.0, Config(30.0), &result));
  ExpectLosNear(result.current_los, Los(90.0, 0.0), 1.0e-12);
  EXPECT_TRUE(result.settled);
}

TEST(SbirsPointingActuatorTest, CommandChangeUsesCurrentPointingAsNewOrigin) {
  SbirsPointingActuator actuator;
  ASSERT_TRUE(actuator.Initialize(Los(0.0, 0.0)));
  SbirsPointingActuatorResult result;
  ASSERT_TRUE(actuator.Step(Los(90.0, 0.0), 1.0, Config(30.0), &result));
  ASSERT_TRUE(actuator.Step(Los(0.0, 0.0), 0.5, Config(30.0), &result));
  EXPECT_NEAR(AzimuthDeg(result.current_los), 15.0, 1.0e-9);
}

TEST(SbirsPointingActuatorTest, SphericalPathHandlesAzimuthWrapAndElevation) {
  SbirsPointingActuator actuator;
  ASSERT_TRUE(actuator.Initialize(Los(179.0, 10.0)));
  SbirsPointingActuatorResult result;
  ASSERT_TRUE(actuator.Step(Los(-179.0, 10.0), 1.0, Config(1.0), &result));
  EXPECT_NEAR(std::abs(AzimuthDeg(result.current_los)), 180.0, 0.02);
  EXPECT_FALSE(result.settled);
}

TEST(SbirsPointingActuatorTest, InvalidInputIsRejectedAtomically) {
  SbirsPointingActuator actuator;
  ASSERT_TRUE(actuator.Initialize(Los(5.0, 6.0)));
  const SbirsPointingActuatorSnapshot before = actuator.Capture();
  SbirsPointingActuatorResult result;
  EXPECT_FALSE(actuator.Step(Los(20.0, 0.0), 0.0, Config(), &result));
  EXPECT_FALSE(actuator.Step(Los(20.0, 0.0), 1.0, Config(0.0), &result));
  session::SbirsVector3M invalid = Los(20.0, 0.0);
  invalid.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(actuator.Step(invalid, 1.0, Config(), &result));
  const SbirsPointingActuatorSnapshot after = actuator.Capture();
  ExpectLosNear(after.current_los, before.current_los, 0.0);
  ExpectLosNear(after.command_los, before.command_los, 0.0);
  EXPECT_EQ(after.initialized, before.initialized);
  EXPECT_EQ(after.settled, before.settled);
}

TEST(SbirsPointingActuatorTest, CaptureRestorePreservesDeterministicContinuation) {
  SbirsPointingActuator uninterrupted;
  ASSERT_TRUE(uninterrupted.Initialize(Los(0.0, 0.0)));
  SbirsPointingActuatorResult first;
  ASSERT_TRUE(uninterrupted.Step(Los(90.0, 20.0), 1.0, Config(20.0), &first));
  const SbirsPointingActuatorSnapshot snapshot = uninterrupted.Capture();

  SbirsPointingActuator restored;
  ASSERT_TRUE(restored.Restore(snapshot));
  SbirsPointingActuatorResult expected;
  SbirsPointingActuatorResult actual;
  ASSERT_TRUE(uninterrupted.Step(Los(90.0, 20.0), 0.5, Config(20.0), &expected));
  ASSERT_TRUE(restored.Step(Los(90.0, 20.0), 0.5, Config(20.0), &actual));
  ExpectLosNear(actual.current_los, expected.current_los, 1.0e-12);
  EXPECT_NEAR(actual.remaining_angle_deg, expected.remaining_angle_deg, 1.0e-12);
  EXPECT_EQ(actual.settled, expected.settled);
}

TEST(SbirsPointingActuatorTest, AntipodalCommandUsesDeterministicGreatCircle) {
  SbirsPointingActuator first;
  SbirsPointingActuator second;
  ASSERT_TRUE(first.Initialize(Los(0.0, 0.0)));
  ASSERT_TRUE(second.Initialize(Los(0.0, 0.0)));
  SbirsPointingActuatorResult left;
  SbirsPointingActuatorResult right;
  ASSERT_TRUE(first.Step(Los(180.0, 0.0), 1.0, Config(45.0), &left));
  ASSERT_TRUE(second.Step(Los(180.0, 0.0), 1.0, Config(45.0), &right));
  ExpectLosNear(left.current_los, right.current_los, 0.0);
  EXPECT_NEAR(left.remaining_angle_deg, 135.0, 1.0e-9);
}

}  // namespace
}  // namespace pipeline
}  // namespace sbirs_sensor
