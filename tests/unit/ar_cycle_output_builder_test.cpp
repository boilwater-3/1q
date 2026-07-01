/**
 * @file ar_cycle_output_builder_test.cpp
 * @brief 验证 ArCycleOutputAdapter 将内部雷达局部输出转换回外部 ECEF 输出。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/session/ArCycleInputAdapter.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/coordinate/position_transform.h"

namespace {

using airborne_radar::session::TrackStateSnapshot;
using airborne_radar::session::ArCycleInput;
using airborne_radar::session::ArCycleInputAdapter;
using airborne_radar::session::ArCycleOutputAdapter;
using airborne_radar::session::ArCycleResult;
using airborne_radar::session::ArExternalPoseInput;
using airborne_radar::session::ArExternalTrackOutputFrame;
using airborne_radar::session::ArSession;
using airborne_radar::session::ArExternalTargetInput;
using airborne_radar::session::TrackOutputFrame;

airborne_radar::config::ArSessionConfig MakeDetectionFocusedConfig() {
  return airborne_radar::config::ArSessionConfigBuilder()
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

ArExternalPoseInput MakePlatformInput() {
  oneq::coordinate::EcefPositionM platform_ecef;
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  ArExternalPoseInput platform;
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

ArExternalTargetInput MakeTargetInput() {
  oneq::coordinate::EcefPositionM target_ecef;
  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 30.0007;
  target_lla.longitude_deg = 120.0012;
  target_lla.altitude_m = 1035.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  ArExternalTargetInput target;
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

std::vector<ArExternalTargetInput> MakeMovingTargetInputs(std::size_t target_count) {
  std::vector<ArExternalTargetInput> targets;
  targets.reserve(target_count);

  for (std::size_t i = 0; i < target_count; ++i) {
    oneq::coordinate::EcefPositionM target_ecef;
    oneq::coordinate::LlaPositionDegM target_lla;
    target_lla.latitude_deg = 30.0004 + static_cast<double>(i) * 0.0003;
    target_lla.longitude_deg = 120.0006 + static_cast<double>(i % 4U) * 0.0005;
    target_lla.altitude_m = 1010.0 + static_cast<double>(i % 5U) * 8.0;
    EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

    ArExternalTargetInput target;
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

void AdvanceExternalTargets(double dt_sec, std::vector<ArExternalTargetInput>* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    ArExternalTargetInput& target = (*targets)[i];
    target.kinematics.position_ecef_m.x_m += target.kinematics.velocity_mps.x_mps * dt_sec;
    target.kinematics.position_ecef_m.y_m += target.kinematics.velocity_mps.y_mps * dt_sec;
    target.kinematics.position_ecef_m.z_m += target.kinematics.velocity_mps.z_mps * dt_sec;
  }
}

const airborne_radar::session::ArExternalTrackKinematics* FindExternalTrackByTargetId(
    const ArExternalTrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].external_target_id == external_target_id) {
      return &frame.tracks[i];
    }
  }
  return nullptr;
}

TrackOutputFrame MakeFrameFromInternalTarget(const ArCycleInput& input) {
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
  const ArExternalPoseInput platform = MakePlatformInput();
  const ArExternalTargetInput target = MakeTargetInput();

  ArCycleInput input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(platform, {target}, 1.0f, &input));
  ASSERT_EQ(input.scene.size(), 1U);

  const TrackOutputFrame frame = MakeFrameFromInternalTarget(input);
  ArExternalTrackOutputFrame external_frame;
  ASSERT_TRUE(ArCycleOutputAdapter::Build(platform, frame, &external_frame));

  ASSERT_EQ(external_frame.cycle_index, frame.cycle_index);
  ASSERT_EQ(external_frame.batch_id, frame.batch_id);
  ASSERT_EQ(external_frame.tracks.size(), 1U);

  const airborne_radar::session::ArExternalTrackKinematics& output = external_frame.tracks[0];
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
  const ArExternalPoseInput platform = MakePlatformInput();
  const TrackOutputFrame frame;
  EXPECT_FALSE(ArCycleOutputAdapter::Build(platform, frame, nullptr));
}

TEST(RadarCycleOutputBuilderTest, FullSessionEstimateConvertsNearExternalTruth) {
  const ArExternalPoseInput platform = MakePlatformInput();
  const ArExternalTargetInput target = MakeTargetInput();

  ArCycleInput input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(platform, {target}, 1.0f, &input));

  ArSession session =
      airborne_radar::session::ArSession::Create(MakeDetectionFocusedConfig());
  const ArCycleResult result = session.StepWithResult(input);
  ASSERT_FALSE(result.has_validation_error);
  ASSERT_FALSE(result.track_output_frame.tracks.empty());

  ArExternalTrackOutputFrame external_frame;
  ASSERT_TRUE(ArCycleOutputAdapter::Build(platform, result.track_output_frame, &external_frame));
  ASSERT_FALSE(external_frame.tracks.empty());

  const airborne_radar::session::ArExternalTrackKinematics& estimate = external_frame.tracks[0];
  EXPECT_NEAR(estimate.target_position_ecef_m.x_m, target.kinematics.position_ecef_m.x_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.y_m, target.kinematics.position_ecef_m.y_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.z_m, target.kinematics.position_ecef_m.z_m, 5.0);
  EXPECT_NEAR(estimate.target_velocity_mps.x_mps, target.kinematics.velocity_mps.x_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.y_mps, target.kinematics.velocity_mps.y_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.z_mps, target.kinematics.velocity_mps.z_mps, 0.5);
}

TEST(RadarCycleOutputBuilderTest, OmittedCycleEnvironmentPreservesStaticEnvironment) {
  const ArExternalPoseInput platform = MakePlatformInput();
  const ArExternalTargetInput target = MakeTargetInput();

  airborne_radar::config::ArSessionConfig config = MakeDetectionFocusedConfig();
  config.environment.jamming_sensitivity_profile =
      airborne_radar::config::JammingSensitivityProfile::kStrict;
  airborne_radar::config::JammerEmitterState jammer;
  jammer.technique = airborne_radar::config::JammingTechnique::kNoiseSuppression;
  jammer.power_db = 12.0f;
  jammer.js_db = 12.0f;
  jammer.angular_span_deg = 10.0f;
  config.environment.scenario_config.jammer_sources.push_back(jammer);

  ArCycleInput static_environment_input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(platform, {target}, 1.0f,
                                            &static_environment_input));
  ASSERT_FALSE(static_environment_input.has_environment);

  ArSession static_environment_session = ArSession::Create(config);
  const ArCycleResult static_environment_result =
      static_environment_session.StepWithResult(static_environment_input);
  ASSERT_FALSE(static_environment_result.has_validation_error);
  ASSERT_FALSE(static_environment_result.track_output_frame.tracks.empty());
  EXPECT_TRUE(static_environment_result.track_output_frame.tracks[0].jamming_detected);

  airborne_radar::session::ArEnvironmentInput no_jammer_environment;
  no_jammer_environment.atmospheric_observation.pressure_hpa = 1013.25f;
  no_jammer_environment.atmospheric_observation.temperature_k = 288.15f;
  no_jammer_environment.atmospheric_observation.relative_humidity = 0.5f;

  ArCycleInput explicit_environment_input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(platform, {target}, 1.0f, no_jammer_environment,
                                            &explicit_environment_input));
  ASSERT_TRUE(explicit_environment_input.has_environment);

  ArSession explicit_environment_session = ArSession::Create(config);
  const ArCycleResult explicit_environment_result =
      explicit_environment_session.StepWithResult(explicit_environment_input);
  ASSERT_FALSE(explicit_environment_result.has_validation_error);
  ASSERT_FALSE(explicit_environment_result.track_output_frame.tracks.empty());
  EXPECT_FALSE(explicit_environment_result.track_output_frame.tracks[0].jamming_detected);
}

TEST(RadarCycleOutputBuilderTest, MultiCycleMovingTargetsStayNearExternalTruth) {
  ArExternalPoseInput platform = MakePlatformInput();
  platform.platform_velocity_mps.x_mps = 0.0;
  platform.platform_velocity_mps.y_mps = 0.0;
  platform.platform_velocity_mps.z_mps = 0.0;

  std::vector<ArExternalTargetInput> targets = MakeMovingTargetInputs(12U);
  ArSession session =
      airborne_radar::session::ArSession::Create(MakeDetectionFocusedConfig());

  const std::size_t cycle_count = 40U;
  const float dt_sec = 0.5f;
  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    ArCycleInput input;
    ASSERT_TRUE(ArCycleInputAdapter::Build(platform, targets, dt_sec, &input))
        << "cycle=" << cycle;
    input.cycle_index = static_cast<std::uint32_t>(cycle);

    const ArCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(result.has_validation_error) << "cycle=" << cycle;
    ASSERT_FALSE(result.track_output_frame.tracks.empty()) << "cycle=" << cycle;

    ArExternalTrackOutputFrame external_frame;
    ASSERT_TRUE(
        ArCycleOutputAdapter::Build(platform, result.track_output_frame, &external_frame))
        << "cycle=" << cycle;

    for (std::size_t target_index = 1U; target_index < targets.size(); ++target_index) {
      const airborne_radar::session::ArExternalTrackKinematics* estimate =
          FindExternalTrackByTargetId(external_frame, static_cast<std::uint64_t>(target_index) + 1U);
      ASSERT_NE(estimate, nullptr) << "cycle=" << cycle << " target_index=" << target_index;

      const ArExternalTargetInput& truth = targets[target_index];
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
