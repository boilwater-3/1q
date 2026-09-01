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

TEST(SbirsPointingCoordinatorTest, InitializesOnceAndAdvancesWithRateLimit) {
  SbirsPointingCoordinator coordinator;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  // 已初始化后幂等：后续 Initialize 不改写镜筒视线。
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(0.0, 1.0, 0.0)));

  const SbirsPointingAdvanceResult result =
      coordinator.AdvanceAcquisition(7U, Vector(0.0, 1.0, 0.0), 1.0, Config(30.0));

  EXPECT_EQ(result.status, SbirsPointingAdvanceStatus::kSlewing);
  EXPECT_NEAR(result.remaining_angle_deg, 60.0, 1.0e-9);
  EXPECT_NEAR(result.settled_duration_sec, 0.0, 1.0e-9);
}

TEST(SbirsPointingCoordinatorTest, SettledDurationCountsStareTimeAfterSlew) {
  SbirsPointingCoordinator coordinator;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));

  // 转动 10° @30°/s：slew ≈ (10−0.01)/30 s，周期 1 s 内剩余为稳定时长。
  const SbirsPointingAdvanceResult result =
      coordinator.AdvanceAcquisition(7U, Vector(std::cos(10.0 * M_PI / 180.0),
                                                std::sin(10.0 * M_PI / 180.0), 0.0),
                                     1.0, Config(30.0));
  ASSERT_EQ(result.status, SbirsPointingAdvanceStatus::kSettled);
  EXPECT_NEAR(result.settled_duration_sec, 1.0 - (10.0 - 0.01) / 30.0, 1.0e-6);
}

TEST(SbirsPointingCoordinatorTest, UpdatedCommandContinuesFromCurrentLos) {
  SbirsPointingCoordinator coordinator;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  ASSERT_EQ(coordinator.AdvanceAcquisition(1U, Vector(0.0, 1.0, 0.0), 1.0, Config()).status,
            SbirsPointingAdvanceStatus::kSlewing);

  const SbirsPointingAdvanceResult result =
      coordinator.AdvanceTracking(Vector(0.0, 0.0, 1.0), 1.0, Config());

  EXPECT_EQ(result.status, SbirsPointingAdvanceStatus::kSlewing);
  EXPECT_GT(result.current_los.z, 0.0);
  EXPECT_GT(result.current_los.y, 0.0);
}

TEST(SbirsPointingCoordinatorTest, ReleaseKeepsSharedLosAndClearResetsIt) {
  SbirsPointingCoordinator coordinator(41U);
  SbirsPointingDisturbanceParameters disturbance_parameters;
  disturbance_parameters.channel_pointing_sigma_deg = 0.1;
  disturbance_parameters.channel_pointing_correlation_time_s = 1.0;
  ASSERT_TRUE(coordinator.AdvanceDisturbance(0.1, disturbance_parameters));
  const double released_disturbance =
      coordinator.Capture().disturbance.channels[0].gauss_markov.azimuth_deg;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  coordinator.AdvanceAcquisition(1U, Vector(0.0, 1.0, 0.0), 1.0, Config());
  const session::SbirsVector3M released_los = coordinator.Capture().actuator.current_los;
  ASSERT_TRUE(coordinator.ReleaseTarget(1U));
  EXPECT_DOUBLE_EQ(coordinator.Capture().disturbance.channels[0].gauss_markov.azimuth_deg,
                   released_disturbance);
  // 单镜筒：释放目标只清簿记，镜筒视线保持（下一目标从当前位置续转）。
  ExpectVectorNear(coordinator.Capture().actuator.current_los, released_los);
  EXPECT_TRUE(coordinator.Capture().actuator.initialized);

  coordinator.Clear();
  EXPECT_FALSE(coordinator.Capture().actuator.initialized);
  EXPECT_TRUE(coordinator.Capture().acquisition_wait_sec.empty());
  EXPECT_TRUE(coordinator.Capture().tracking_gate_failure_counts.empty());
  EXPECT_DOUBLE_EQ(coordinator.Capture().disturbance.channels[0].gauss_markov.azimuth_deg, 0.0);
}

TEST(SbirsPointingCoordinatorTest, MovingCommandTimesOutAndClearsAcquisitionWait) {
  SbirsPointingCoordinator coordinator;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  SbirsPointingAdvanceResult result;
  for (int step = 0; step < 4; ++step) {
    const session::SbirsVector3M current = coordinator.Capture().actuator.current_los;
    result = coordinator.AdvanceAcquisition(9U, Vector(-current.x, -current.y, -current.z), 0.5,
                                            Config(90.0));
  }

  EXPECT_EQ(result.status, SbirsPointingAdvanceStatus::kTimedOut);
  EXPECT_EQ(coordinator.Capture().acquisition_wait_sec.count(9U), 0U);
  EXPECT_TRUE(coordinator.Capture().actuator.initialized);
}

TEST(SbirsPointingCoordinatorTest, AcquisitionWaitIsPerTarget) {
  SbirsPointingCoordinator coordinator;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  coordinator.AdvanceAcquisition(1U, Vector(0.0, 1.0, 0.0), 0.5, Config(1.0));
  coordinator.AdvanceAcquisition(2U, Vector(0.0, -1.0, 0.0), 0.5, Config(1.0));

  const SbirsPointingCoordinatorSnapshot snapshot = coordinator.Capture();
  ASSERT_EQ(snapshot.acquisition_wait_sec.size(), 2U);
  EXPECT_DOUBLE_EQ(snapshot.acquisition_wait_sec.at(1U), 0.5);
  EXPECT_DOUBLE_EQ(snapshot.acquisition_wait_sec.at(2U), 0.5);

  ASSERT_TRUE(coordinator.PromoteToTracking(1U));
  EXPECT_EQ(coordinator.Capture().acquisition_wait_sec.size(), 1U);
}

TEST(SbirsPointingCoordinatorTest, CaptureRestorePreservesContinuation) {
  SbirsPointingCoordinator uninterrupted(43U);
  SbirsPointingDisturbanceParameters disturbance_parameters;
  disturbance_parameters.common_attitude_sigma_deg = 0.1;
  disturbance_parameters.channel_pointing_sigma_deg = 0.2;
  ASSERT_TRUE(uninterrupted.AdvanceDisturbance(0.1, disturbance_parameters));
  ASSERT_TRUE(uninterrupted.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  uninterrupted.AdvanceAcquisition(4U, Vector(0.0, 1.0, 0.0), 0.5, Config());
  const SbirsPointingCoordinatorSnapshot snapshot = uninterrupted.Capture();
  SbirsPointingCoordinator restored;
  ASSERT_TRUE(restored.Restore(snapshot));
  ASSERT_TRUE(uninterrupted.AdvanceDisturbance(0.2, disturbance_parameters));
  ASSERT_TRUE(restored.AdvanceDisturbance(0.2, disturbance_parameters));

  const SbirsPointingAdvanceResult expected =
      uninterrupted.AdvanceTracking(Vector(0.0, 0.0, 1.0), 0.5, Config());
  const SbirsPointingAdvanceResult actual =
      restored.AdvanceTracking(Vector(0.0, 0.0, 1.0), 0.5, Config());
  EXPECT_EQ(actual.status, expected.status);
  EXPECT_DOUBLE_EQ(actual.current_los.x, expected.current_los.x);
  ExpectVectorNear(actual.current_los, expected.current_los);
  EXPECT_DOUBLE_EQ(restored.Capture().disturbance.common.azimuth_deg,
                   uninterrupted.Capture().disturbance.common.azimuth_deg);
  EXPECT_DOUBLE_EQ(restored.Capture().disturbance.channels[0].gauss_markov.azimuth_deg,
                   uninterrupted.Capture().disturbance.channels[0].gauss_markov.azimuth_deg);
}

TEST(SbirsPointingCoordinatorTest, TrackingGateCountResetsAndRoundtrips) {
  SbirsPointingCoordinator coordinator;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  EXPECT_EQ(coordinator.RecordTrackingGateResult(8U, false), 1U);
  EXPECT_EQ(coordinator.RecordTrackingGateResult(8U, false), 2U);

  const SbirsPointingCoordinatorSnapshot snapshot = coordinator.Capture();
  ASSERT_EQ(snapshot.tracking_gate_failure_counts.at(8U), 2U);
  SbirsPointingCoordinator restored;
  ASSERT_TRUE(restored.Restore(snapshot));
  EXPECT_EQ(restored.RecordTrackingGateResult(8U, true), 0U);
  EXPECT_EQ(restored.Capture().tracking_gate_failure_counts.at(8U), 0U);
}

TEST(SbirsPointingCoordinatorTest, InvalidSnapshotIsRejectedAtomically) {
  SbirsPointingCoordinator coordinator;
  ASSERT_TRUE(coordinator.EnsureActuatorInitialized(Vector(1.0, 0.0, 0.0)));
  coordinator.AdvanceAcquisition(3U, Vector(0.0, 1.0, 0.0), 1.0, Config());
  const SbirsPointingCoordinatorSnapshot before = coordinator.Capture();

  SbirsPointingCoordinatorSnapshot invalid = before;
  invalid.acquisition_wait_sec[3U] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(coordinator.Restore(invalid));

  invalid = before;
  invalid.actuator_initialized = true;
  invalid.actuator.initialized = false;
  EXPECT_FALSE(coordinator.Restore(invalid));

  const SbirsPointingCoordinatorSnapshot after = coordinator.Capture();
  ExpectVectorNear(after.actuator.current_los, before.actuator.current_los);
  EXPECT_EQ(after.disturbance.common.random_state, before.disturbance.common.random_state);
}

}  // namespace
}  // namespace pipeline
}  // namespace sbirs_sensor
