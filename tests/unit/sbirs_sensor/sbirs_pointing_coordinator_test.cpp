#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sbirs_sensor/pipeline/SbirsPointingCoordinator.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

session::SbirsVector3M Vector(double x, double y, double z) { return {x, y, z}; }

SbirsPointingActuatorConfig Config(double rate = 30.0, double tolerance = 0.01) {
  SbirsPointingActuatorConfig config;
  config.max_slew_rate_deg_per_sec = rate;
  config.settle_tolerance_deg = tolerance;
  return config;
}

void ExpectVectorNear(const session::SbirsVector3M& actual, const session::SbirsVector3M& expected,
                      double tolerance = 1.0e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

TEST(SbirsPointingCoordinatorTest, ReservesChannelAndAdvancesWithRateLimit) {
  SbirsPointingCoordinator coordinator(1);
  ASSERT_TRUE(coordinator.Reserve(0, 7U, Vector(1.0, 0.0, 0.0)));

  const SbirsPointingAdvanceResult result =
      coordinator.Advance(0, 7U, Vector(0.0, 1.0, 0.0), 1.0, Config(30.0));

  EXPECT_EQ(result.status, SbirsPointingAdvanceStatus::kSlewing);
  EXPECT_NEAR(result.remaining_angle_deg, 60.0, 1.0e-9);
  EXPECT_EQ(coordinator.ChannelOf(7U), 0);
}

TEST(SbirsPointingCoordinatorTest, UpdatedCommandContinuesFromCurrentLos) {
  SbirsPointingCoordinator coordinator(1);
  ASSERT_TRUE(coordinator.Reserve(0, 1U, Vector(1.0, 0.0, 0.0)));
  ASSERT_EQ(coordinator.Advance(0, 1U, Vector(0.0, 1.0, 0.0), 1.0, Config()).status,
            SbirsPointingAdvanceStatus::kSlewing);

  const SbirsPointingAdvanceResult result =
      coordinator.Advance(0, 1U, Vector(0.0, 0.0, 1.0), 1.0, Config());

  EXPECT_EQ(result.status, SbirsPointingAdvanceStatus::kSlewing);
  EXPECT_GT(result.current_los.z, 0.0);
  EXPECT_GT(result.current_los.y, 0.0);
}

TEST(SbirsPointingCoordinatorTest, ChannelsRemainIsolated) {
  SbirsPointingCoordinator coordinator(2);
  ASSERT_TRUE(coordinator.Reserve(0, 10U, Vector(1.0, 0.0, 0.0)));
  ASSERT_TRUE(coordinator.Reserve(1, 20U, Vector(0.0, 1.0, 0.0)));
  coordinator.Advance(1, 20U, Vector(0.0, 0.0, 1.0), 0.5, Config());
  coordinator.Advance(0, 10U, Vector(0.0, 1.0, 0.0), 0.5, Config());

  const SbirsPointingCoordinatorSnapshot snapshot = coordinator.Capture();
  EXPECT_EQ(snapshot.channels[0].target_id, 10U);
  EXPECT_EQ(snapshot.channels[1].target_id, 20U);
  EXPECT_NE(snapshot.channels[0].actuator.current_los.x,
            snapshot.channels[1].actuator.current_los.x);
}

TEST(SbirsPointingCoordinatorTest, ReleaseKeepsLosAndClearResetsIt) {
  SbirsPointingCoordinator coordinator(1, 41U);
  SbirsPointingDisturbanceParameters disturbance_parameters;
  disturbance_parameters.channel_pointing_sigma_deg = 0.1;
  disturbance_parameters.channel_pointing_correlation_time_s = 1.0;
  ASSERT_TRUE(coordinator.AdvanceDisturbance(0.1, disturbance_parameters));
  const double released_disturbance =
      coordinator.Capture().disturbance.channels[0].gauss_markov.azimuth_deg;
  ASSERT_TRUE(coordinator.Reserve(0, 1U, Vector(1.0, 0.0, 0.0)));
  coordinator.Advance(0, 1U, Vector(0.0, 1.0, 0.0), 1.0, Config());
  const session::SbirsVector3M released_los =
      coordinator.Capture().channels[0].actuator.current_los;
  ASSERT_TRUE(coordinator.ReleaseTarget(1U));
  EXPECT_DOUBLE_EQ(coordinator.Capture().disturbance.channels[0].gauss_markov.azimuth_deg,
                   released_disturbance);
  ASSERT_TRUE(coordinator.Reserve(0, 2U, Vector(0.0, 0.0, 1.0)));
  ExpectVectorNear(coordinator.Capture().channels[0].actuator.current_los, released_los);

  coordinator.Clear();
  EXPECT_FALSE(coordinator.Capture().channels[0].actuator.initialized);
  EXPECT_FALSE(coordinator.IsTargetBound(2U));
  EXPECT_DOUBLE_EQ(coordinator.Capture().disturbance.channels[0].gauss_markov.azimuth_deg, 0.0);
}

TEST(SbirsPointingCoordinatorTest, MovingCommandTimesOutAndReleasesBinding) {
  SbirsPointingCoordinator coordinator(1);
  ASSERT_TRUE(coordinator.Reserve(0, 9U, Vector(1.0, 0.0, 0.0)));
  SbirsPointingAdvanceResult result;
  for (int step = 0; step < 4; ++step) {
    const session::SbirsVector3M current = coordinator.Capture().channels[0].actuator.current_los;
    result =
        coordinator.Advance(0, 9U, Vector(-current.x, -current.y, -current.z), 0.5, Config(90.0));
  }

  EXPECT_EQ(result.status, SbirsPointingAdvanceStatus::kTimedOut);
  EXPECT_FALSE(coordinator.IsTargetBound(9U));
  EXPECT_TRUE(coordinator.Capture().channels[0].actuator.initialized);
}

TEST(SbirsPointingCoordinatorTest, CaptureRestorePreservesContinuation) {
  SbirsPointingCoordinator uninterrupted(1, 43U);
  SbirsPointingDisturbanceParameters disturbance_parameters;
  disturbance_parameters.common_attitude_sigma_deg = 0.1;
  disturbance_parameters.channel_pointing_sigma_deg = 0.2;
  ASSERT_TRUE(uninterrupted.AdvanceDisturbance(0.1, disturbance_parameters));
  ASSERT_TRUE(uninterrupted.Reserve(0, 4U, Vector(1.0, 0.0, 0.0)));
  uninterrupted.Advance(0, 4U, Vector(0.0, 1.0, 0.0), 0.5, Config());
  const SbirsPointingCoordinatorSnapshot snapshot = uninterrupted.Capture();
  SbirsPointingCoordinator restored(1);
  ASSERT_TRUE(restored.Restore(snapshot));
  ASSERT_TRUE(uninterrupted.AdvanceDisturbance(0.2, disturbance_parameters));
  ASSERT_TRUE(restored.AdvanceDisturbance(0.2, disturbance_parameters));

  const SbirsPointingAdvanceResult expected =
      uninterrupted.Advance(0, 4U, Vector(0.0, 0.0, 1.0), 0.5, Config());
  const SbirsPointingAdvanceResult actual =
      restored.Advance(0, 4U, Vector(0.0, 0.0, 1.0), 0.5, Config());
  EXPECT_EQ(actual.status, expected.status);
  EXPECT_DOUBLE_EQ(actual.elapsed_wait_sec, expected.elapsed_wait_sec);
  ExpectVectorNear(actual.current_los, expected.current_los);
  EXPECT_DOUBLE_EQ(restored.Capture().disturbance.common.azimuth_deg,
                   uninterrupted.Capture().disturbance.common.azimuth_deg);
  EXPECT_DOUBLE_EQ(restored.Capture().disturbance.channels[0].gauss_markov.azimuth_deg,
                   uninterrupted.Capture().disturbance.channels[0].gauss_markov.azimuth_deg);
}

TEST(SbirsPointingCoordinatorTest, CommonModeIsSharedAndChannelResidualsAreIndependent) {
  SbirsPointingCoordinator coordinator(2, 47U);
  SbirsPointingDisturbanceParameters parameters;
  parameters.common_attitude_sigma_deg = 0.1;
  parameters.channel_pointing_sigma_deg = 0.2;
  ASSERT_TRUE(coordinator.AdvanceDisturbance(0.1, parameters));
  SbirsPointingDisturbanceSample first;
  SbirsPointingDisturbanceSample second;
  ASSERT_TRUE(coordinator.DisturbanceSample(0, parameters, &first));
  ASSERT_TRUE(coordinator.DisturbanceSample(1, parameters, &second));
  EXPECT_DOUBLE_EQ(first.common.azimuth_deg, second.common.azimuth_deg);
  EXPECT_DOUBLE_EQ(first.common.elevation_deg, second.common.elevation_deg);
  EXPECT_NE(first.channel.azimuth_deg, second.channel.azimuth_deg);
}

TEST(SbirsPointingCoordinatorTest, TrackingAdvanceKeepsBindingWithoutAcquisitionTimeout) {
  SbirsPointingCoordinator coordinator(1);
  ASSERT_TRUE(coordinator.Reserve(0, 4U, Vector(1.0, 0.0, 0.0)));
  ASSERT_TRUE(coordinator.PromoteToTracking(4U));

  for (int step = 0; step < 5; ++step) {
    const SbirsPointingAdvanceResult result = coordinator.AdvanceTracking(
        0, 4U, Vector(0.0, 1.0, 0.0), 0.5, Config(30.0));
    EXPECT_NE(result.status, SbirsPointingAdvanceStatus::kTimedOut);
  }

  EXPECT_EQ(coordinator.ChannelOf(4U), 0);
  EXPECT_DOUBLE_EQ(coordinator.Capture().channels[0].elapsed_wait_sec, 0.0);
}

TEST(SbirsPointingCoordinatorTest, TrackingGateCountResetsAndRoundtrips) {
  SbirsPointingCoordinator coordinator(1);
  ASSERT_TRUE(coordinator.Reserve(0, 8U, Vector(1.0, 0.0, 0.0)));
  ASSERT_TRUE(coordinator.PromoteToTracking(8U));
  EXPECT_EQ(coordinator.RecordTrackingGateResult(8U, false), 1U);
  EXPECT_EQ(coordinator.RecordTrackingGateResult(8U, false), 2U);

  const SbirsPointingCoordinatorSnapshot snapshot = coordinator.Capture();
  ASSERT_EQ(snapshot.channels[0].tracking_gate_failure_count, 2U);
  SbirsPointingCoordinator restored(1);
  ASSERT_TRUE(restored.Restore(snapshot));
  EXPECT_EQ(restored.RecordTrackingGateResult(8U, true), 0U);
  EXPECT_EQ(restored.Capture().channels[0].tracking_gate_failure_count, 0U);
}

TEST(SbirsPointingCoordinatorTest, InvalidSnapshotIsRejectedAtomically) {
  SbirsPointingCoordinator coordinator(2);
  ASSERT_TRUE(coordinator.Reserve(0, 3U, Vector(1.0, 0.0, 0.0)));
  const SbirsPointingCoordinatorSnapshot before = coordinator.Capture();
  SbirsPointingCoordinatorSnapshot invalid = before;
  invalid.channels[1].channel_id = 1;
  invalid.channels[1].has_bound_target = true;
  invalid.channels[1].target_id = 3U;
  invalid.channels[1].elapsed_wait_sec = std::numeric_limits<double>::infinity();
  invalid.channels[1].tracking_gate_failure_count = 1U;
  invalid.channels[1].actuator.initialized = true;
  invalid.channels[1].actuator.current_los = Vector(0.0, 1.0, 0.0);
  invalid.channels[1].actuator.command_los = Vector(0.0, 1.0, 0.0);
  invalid.disturbance.channels[1].elapsed_time_s = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(coordinator.Restore(invalid));
  const SbirsPointingCoordinatorSnapshot after = coordinator.Capture();
  EXPECT_EQ(after.channels[0].target_id, before.channels[0].target_id);
  ExpectVectorNear(after.channels[0].actuator.current_los, before.channels[0].actuator.current_los);
  EXPECT_EQ(after.disturbance.common.random_state, before.disturbance.common.random_state);
}

}  // namespace
}  // namespace pipeline
}  // namespace sbirs_sensor
