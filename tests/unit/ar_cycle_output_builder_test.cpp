/**
 * @file ar_cycle_output_builder_test.cpp
 * @brief 验证 RadarCycleOutputBuilder 将内部雷达局部输出转换回外部 ECEF 输出。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/session/RadarCycleInputBuilder.h"
#include "1q/airborne_radar/session/RadarCycleOutputBuilder.h"
#include "1q/coordinate/position_transform.h"

namespace {

using airborne_radar::session::TrackStateSnapshot;
using airborne_radar::session::RadarCycleInput;
using airborne_radar::session::RadarCycleInputBuilder;
using airborne_radar::session::RadarCycleOutputBuilder;
using airborne_radar::session::RadarCycleResult;
using airborne_radar::session::RadarExternalPoseInput;
using airborne_radar::session::RadarExternalTrackOutputFrame;
using airborne_radar::session::RadarSession;
using airborne_radar::session::RadarExternalTargetInput;
using airborne_radar::session::TrackOutputFrame;

airborne_radar::config::RadarSessionConfig MakeDetectionFocusedConfig() {
  return airborne_radar::config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(
          airborne_radar::config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(
          airborne_radar::config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(
          airborne_radar::config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

RadarExternalPoseInput MakePlatformInput() {
  oneq::coordinate::EcefPositionM platform_ecef;
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  RadarExternalPoseInput platform;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_velocity_mps.x_mps = -20.0;
  platform.platform_velocity_mps.y_mps = 35.0;
  platform.platform_velocity_mps.z_mps = 2.0;
  platform.platform_attitude_deg.yaw_deg = 12.0;
  platform.platform_attitude_deg.pitch_deg = -3.0;
  platform.platform_attitude_deg.roll_deg = 1.5;
  platform.radar_mount_angles_deg.yaw_deg = 4.0;
  platform.radar_mount_angles_deg.pitch_deg = -1.0;
  platform.radar_mount_angles_deg.roll_deg = 0.5;
  return platform;
}

RadarExternalTargetInput MakeTargetInput() {
  oneq::coordinate::EcefPositionM target_ecef;
  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 30.0007;
  target_lla.longitude_deg = 120.0012;
  target_lla.altitude_m = 1035.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  RadarExternalTargetInput target;
  target.target_id = 9001U;
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target.kinematics.position_ecef_m = target_ecef;
  target.kinematics.velocity_mps.x_mps = -15.0;
  target.kinematics.velocity_mps.y_mps = 42.0;
  target.kinematics.velocity_mps.z_mps = 3.5;
  target.rcs = 1.7f;
  target.swerling_type = 2;
  return target;
}

std::vector<RadarExternalTargetInput> MakeMovingTargetInputs(std::size_t target_count) {
  std::vector<RadarExternalTargetInput> targets;
  targets.reserve(target_count);

  for (std::size_t i = 0; i < target_count; ++i) {
    oneq::coordinate::EcefPositionM target_ecef;
    oneq::coordinate::LlaPositionDegM target_lla;
    target_lla.latitude_deg = 30.0004 + static_cast<double>(i) * 0.0003;
    target_lla.longitude_deg = 120.0006 + static_cast<double>(i % 4U) * 0.0005;
    target_lla.altitude_m = 1010.0 + static_cast<double>(i % 5U) * 8.0;
    EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

    RadarExternalTargetInput target;
    target.target_id = static_cast<std::uint64_t>(i) + 1U;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = target_ecef;
    target.kinematics.velocity_mps.x_mps = -18.0 + static_cast<double>(i % 5U) * 2.5;
    target.kinematics.velocity_mps.y_mps = 24.0 + static_cast<double>(i % 7U) * 1.7;
    target.kinematics.velocity_mps.z_mps = -0.4 + static_cast<double>(i % 3U) * 0.4;
    target.rcs = 0.8f + static_cast<float>(i % 4U) * 0.2f;
    target.swerling_type = static_cast<int>(i % 3U);
    targets.push_back(target);
  }
  return targets;
}

void AdvanceExternalTargets(double dt_sec, std::vector<RadarExternalTargetInput>* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    RadarExternalTargetInput& target = (*targets)[i];
    target.kinematics.position_ecef_m.x_m += target.kinematics.velocity_mps.x_mps * dt_sec;
    target.kinematics.position_ecef_m.y_m += target.kinematics.velocity_mps.y_mps * dt_sec;
    target.kinematics.position_ecef_m.z_m += target.kinematics.velocity_mps.z_mps * dt_sec;
  }
}

const airborne_radar::session::RadarExternalTrackKinematics* FindExternalTrackByTargetId(
    const RadarExternalTrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].external_target_id == external_target_id) {
      return &frame.tracks[i];
    }
  }
  return nullptr;
}

TrackOutputFrame MakeFrameFromInternalTarget(const RadarCycleInput& input) {
  TrackOutputFrame frame;
  frame.cycle_index = 7U;
  frame.batch_id = 11U;

  TrackStateSnapshot snapshot;
  snapshot.association_key = 1001U;
  snapshot.external_target_id = 9001U;
  snapshot.status = airborne_radar::session::TrackStatus::kConfirmed;
  snapshot.position_x = input.scene[0].position_x;
  snapshot.position_y = input.scene[0].position_y;
  snapshot.position_z = input.scene[0].position_z;
  snapshot.velocity_x = input.scene[0].velocity_x;
  snapshot.velocity_y = input.scene[0].velocity_y;
  snapshot.velocity_z = input.scene[0].velocity_z;
  snapshot.speed = std::sqrt(snapshot.velocity_x * snapshot.velocity_x +
                             snapshot.velocity_y * snapshot.velocity_y +
                             snapshot.velocity_z * snapshot.velocity_z);
  snapshot.rcs = input.scene[0].rcs;
  snapshot.hit_count = 3U;
  frame.tracks.push_back(snapshot);
  return frame;
}

}  // namespace

TEST(RadarCycleOutputBuilderTest, ConvertsInternalLocalFrameBackToExternalEcef) {
  const RadarExternalPoseInput platform = MakePlatformInput();
  const RadarExternalTargetInput target = MakeTargetInput();

  RadarCycleInput input;
  ASSERT_TRUE(RadarCycleInputBuilder::Build(platform, {target}, 1.0f, &input));
  ASSERT_EQ(input.scene.size(), 1U);

  const TrackOutputFrame frame = MakeFrameFromInternalTarget(input);
  RadarExternalTrackOutputFrame external_frame;
  ASSERT_TRUE(RadarCycleOutputBuilder::Build(platform, frame, &external_frame));

  ASSERT_EQ(external_frame.cycle_index, frame.cycle_index);
  ASSERT_EQ(external_frame.batch_id, frame.batch_id);
  ASSERT_EQ(external_frame.tracks.size(), 1U);

  const airborne_radar::session::RadarExternalTrackKinematics& output = external_frame.tracks[0];
  EXPECT_EQ(output.association_key, 1001U);
  EXPECT_EQ(output.external_target_id, 9001U);
  EXPECT_EQ(output.status, airborne_radar::session::TrackStatus::kConfirmed);
  EXPECT_NEAR(output.target_position_ecef_m.x_m, target.kinematics.position_ecef_m.x_m, 0.1);
  EXPECT_NEAR(output.target_position_ecef_m.y_m, target.kinematics.position_ecef_m.y_m, 0.1);
  EXPECT_NEAR(output.target_position_ecef_m.z_m, target.kinematics.position_ecef_m.z_m, 0.1);
  EXPECT_NEAR(output.target_velocity_mps.x_mps, target.kinematics.velocity_mps.x_mps, 1.0e-4);
  EXPECT_NEAR(output.target_velocity_mps.y_mps, target.kinematics.velocity_mps.y_mps, 1.0e-4);
  EXPECT_NEAR(output.target_velocity_mps.z_mps, target.kinematics.velocity_mps.z_mps, 1.0e-4);
  EXPECT_FLOAT_EQ(output.rcs, target.rcs);
  EXPECT_EQ(output.hit_count, 3U);
}

TEST(RadarCycleOutputBuilderTest, NullOutputReturnsFalse) {
  const RadarExternalPoseInput platform = MakePlatformInput();
  const TrackOutputFrame frame;
  EXPECT_FALSE(RadarCycleOutputBuilder::Build(platform, frame, nullptr));
}

TEST(RadarCycleOutputBuilderTest, FullSessionEstimateConvertsNearExternalTruth) {
  const RadarExternalPoseInput platform = MakePlatformInput();
  const RadarExternalTargetInput target = MakeTargetInput();

  RadarCycleInput input;
  ASSERT_TRUE(RadarCycleInputBuilder::Build(platform, {target}, 1.0f, &input));

  RadarSession session =
      airborne_radar::session::RadarSession::Create(MakeDetectionFocusedConfig());
  const RadarCycleResult result = session.StepWithResult(input);
  ASSERT_FALSE(result.has_validation_error);
  ASSERT_FALSE(result.track_output_frame.tracks.empty());

  RadarExternalTrackOutputFrame external_frame;
  ASSERT_TRUE(RadarCycleOutputBuilder::Build(platform, result.track_output_frame, &external_frame));
  ASSERT_FALSE(external_frame.tracks.empty());

  const airborne_radar::session::RadarExternalTrackKinematics& estimate = external_frame.tracks[0];
  EXPECT_NEAR(estimate.target_position_ecef_m.x_m, target.kinematics.position_ecef_m.x_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.y_m, target.kinematics.position_ecef_m.y_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.z_m, target.kinematics.position_ecef_m.z_m, 5.0);
  EXPECT_NEAR(estimate.target_velocity_mps.x_mps, target.kinematics.velocity_mps.x_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.y_mps, target.kinematics.velocity_mps.y_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.z_mps, target.kinematics.velocity_mps.z_mps, 0.5);
}

TEST(RadarCycleOutputBuilderTest, MultiCycleMovingTargetsStayNearExternalTruth) {
  RadarExternalPoseInput platform = MakePlatformInput();
  platform.platform_velocity_mps.x_mps = 0.0;
  platform.platform_velocity_mps.y_mps = 0.0;
  platform.platform_velocity_mps.z_mps = 0.0;

  std::vector<RadarExternalTargetInput> targets = MakeMovingTargetInputs(12U);
  RadarSession session =
      airborne_radar::session::RadarSession::Create(MakeDetectionFocusedConfig());

  const std::size_t cycle_count = 40U;
  const float dt_sec = 0.5f;
  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    RadarCycleInput input;
    ASSERT_TRUE(RadarCycleInputBuilder::Build(platform, targets, dt_sec, &input))
        << "cycle=" << cycle;
    input.cycle_index = static_cast<std::uint32_t>(cycle);

    const RadarCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(result.has_validation_error) << "cycle=" << cycle;
    ASSERT_FALSE(result.track_output_frame.tracks.empty()) << "cycle=" << cycle;

    RadarExternalTrackOutputFrame external_frame;
    ASSERT_TRUE(
        RadarCycleOutputBuilder::Build(platform, result.track_output_frame, &external_frame))
        << "cycle=" << cycle;

    for (std::size_t target_index = 1U; target_index < targets.size(); ++target_index) {
      const airborne_radar::session::RadarExternalTrackKinematics* estimate =
          FindExternalTrackByTargetId(external_frame, static_cast<std::uint64_t>(target_index) + 1U);
      ASSERT_NE(estimate, nullptr) << "cycle=" << cycle << " target_index=" << target_index;

      const RadarExternalTargetInput& truth = targets[target_index];
      EXPECT_NEAR(estimate->target_position_ecef_m.x_m, truth.kinematics.position_ecef_m.x_m, 10.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_position_ecef_m.y_m, truth.kinematics.position_ecef_m.y_m, 10.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_position_ecef_m.z_m, truth.kinematics.position_ecef_m.z_m, 10.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_velocity_mps.x_mps, truth.kinematics.velocity_mps.x_mps, 1.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_velocity_mps.y_mps, truth.kinematics.velocity_mps.y_mps, 1.0)
          << "cycle=" << cycle << " target_index=" << target_index;
      EXPECT_NEAR(estimate->target_velocity_mps.z_mps, truth.kinematics.velocity_mps.z_mps, 1.0)
          << "cycle=" << cycle << " target_index=" << target_index;
    }

    AdvanceExternalTargets(dt_sec, &targets);
  }
}
