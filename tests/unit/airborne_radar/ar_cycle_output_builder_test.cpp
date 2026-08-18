/**
 * @file ar_cycle_output_builder_test.cpp
 * @brief 验证 ArCycleOutputAdapter 将内部雷达局部输出转换回外部 ECEF 输出。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/coordinate/position_transform.h"

namespace {

using airborne_radar::session::TrackStateSnapshot;
using airborne_radar::session::ArCycleInput;
using airborne_radar::session::ArCycleOutputAdapter;
using airborne_radar::session::ArCycleResult;
using airborne_radar::session::ArExternalPoseInput;
using airborne_radar::session::ArExternalTrackOutputFrame;
using airborne_radar::session::ArSession;
using airborne_radar::session::ArExternalTargetInput;
using airborne_radar::session::TrackOutputFrame;
using airborne_radar::session::ArTrackLifecycleRecorder;

airborne_radar::config::ArSessionConfig MakeDetectionFocusedConfig() {
  airborne_radar::config::ArSessionConfig cfg;
  cfg.policy.detection = airborne_radar::config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = airborne_radar::config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = airborne_radar::config::profiles::kFastConfirmLifecycle;
  return cfg;
}

ArExternalPoseInput MakePlatformInput() {
  oneq::coordinate::EcefPositionM platform_ecef;
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  ArExternalPoseInput platform;
  platform.platform_entity_id = 42U;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_velocity_mps.x_mps = -20.0;
  platform.platform_velocity_mps.y_mps = 35.0;
  platform.platform_velocity_mps.z_mps = 2.0;
  platform.platform_attitude_deg.yaw_deg = 12.0;
  platform.platform_attitude_deg.pitch_deg = -3.0;
  platform.platform_attitude_deg.roll_deg = 1.5;
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

TrackOutputFrame MakeFrameFromInternalTarget(const ArExternalPoseInput& platform,
                                             const ArExternalTargetInput& target) {
  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::Vector3f velocity;
  const oneq::coordinate::EulerAnglesDeg zero_mount{};
  EXPECT_TRUE(airborne_radar::session::TryMakeArPoseFromExternalKinematics(
      platform, zero_mount, &reference, &velocity));
  airborne_radar::session::ArSceneTarget local_target;
  EXPECT_TRUE(airborne_radar::session::TryMakeArTargetFromExternalKinematics(
      target, reference, velocity, &local_target));
  TrackOutputFrame frame;
  frame.cycle_index = 7U;
  frame.batch_id = 11U;

  TrackStateSnapshot snapshot;
  snapshot.association_key = 1001U;
  snapshot.external_target_id = 9001U;
  snapshot.status = airborne_radar::session::TrackStatus::kConfirmed;
  snapshot.position_x = local_target.position_x;
  snapshot.position_y = local_target.position_y;
  snapshot.position_z = local_target.position_z;
  snapshot.velocity_x = local_target.velocity_x;
  snapshot.velocity_y = local_target.velocity_y;
  snapshot.velocity_z = local_target.velocity_z;
  snapshot.speed = std::sqrt(snapshot.velocity_x * snapshot.velocity_x +
                             snapshot.velocity_y * snapshot.velocity_y +
                             snapshot.velocity_z * snapshot.velocity_z);
  snapshot.rcs = local_target.rcs;
  snapshot.hit_count = 3U;
  frame.tracks.push_back(snapshot);
  return frame;
}

}  // namespace

TEST(RadarCycleOutputBuilderTest, ConvertsInternalLocalFrameBackToExternalEcef) {
  const ArExternalPoseInput platform = MakePlatformInput();
  const ArExternalTargetInput target = MakeTargetInput();

  const TrackOutputFrame frame = MakeFrameFromInternalTarget(platform, target);
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
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);

  ArSession session =
      airborne_radar::session::ArSession::Create(MakeDetectionFocusedConfig());
  const ArCycleResult result = session.StepWithResult(input);
  ASSERT_FALSE(airborne_radar::session::HasValidationError(result.issues));
  ASSERT_FALSE(result.output_frame.tracks.empty());

  ArExternalTrackOutputFrame external_frame;
  ASSERT_TRUE(ArCycleOutputAdapter::Build(platform, result.output_frame, &external_frame));
  ASSERT_FALSE(external_frame.tracks.empty());

  const airborne_radar::session::ArExternalTrackKinematics& estimate = external_frame.tracks[0];
  EXPECT_NEAR(estimate.target_position_ecef_m.x_m, target.kinematics.position_ecef_m.x_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.y_m, target.kinematics.position_ecef_m.y_m, 5.0);
  EXPECT_NEAR(estimate.target_position_ecef_m.z_m, target.kinematics.position_ecef_m.z_m, 5.0);
  EXPECT_NEAR(estimate.target_velocity_mps.x_mps, target.kinematics.velocity_mps.x_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.y_mps, target.kinematics.velocity_mps.y_mps, 0.5);
  EXPECT_NEAR(estimate.target_velocity_mps.z_mps, target.kinematics.velocity_mps.z_mps, 0.5);
}

TEST(RadarCycleOutputBuilderTest, NaturalEnvironmentIsIndependentFromInterference) {
  const ArExternalPoseInput platform = MakePlatformInput();
  const ArExternalTargetInput target = MakeTargetInput();

  ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  EXPECT_TRUE(input.interference.emissions.empty());

  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  const ArCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
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
    input.cycle_index = static_cast<std::uint32_t>(cycle + 1U);
    input.cycle_start_time_s = static_cast<double>(cycle) * dt_sec;
    input.dt_sec = dt_sec;
    input.platform = platform;
    input.targets = targets;

    const ArCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(airborne_radar::session::HasValidationError(result.issues)) << "cycle=" << cycle;
    ASSERT_FALSE(result.output_frame.tracks.empty()) << "cycle=" << cycle;

    ArExternalTrackOutputFrame external_frame;
    ASSERT_TRUE(
        ArCycleOutputAdapter::Build(platform, result.output_frame, &external_frame))
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

TEST(RadarCycleOutputBuilderTest, AttachRecorderDrivesUpdateAutomatically) {
  const ArExternalPoseInput platform = MakePlatformInput();
  const ArExternalTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);

  // 运行足够多周期让轨迹进入 confirmed 状态（kFastConfirmLifecycle）。
  ArExternalTargetInput moving_target = target;
  const float dt_sec = 0.5f;
  bool saw_event = false;
  for (std::uint32_t cycle = 1U; cycle <= 10U; ++cycle) {
    ArCycleInput input;
    input.cycle_index = cycle;
    input.cycle_start_time_s = static_cast<double>(cycle - 1U) * dt_sec;
    input.dt_sec = dt_sec;
    input.platform = platform;
    input.targets.push_back(moving_target);
    session.StepWithResult(input);
    if (!recorder.GetLastEvents().empty()) {
      saw_event = true;
    }
    moving_target.kinematics.position_ecef_m.x_m += moving_target.kinematics.velocity_mps.x_mps * dt_sec;
    moving_target.kinematics.position_ecef_m.y_m += moving_target.kinematics.velocity_mps.y_mps * dt_sec;
    moving_target.kinematics.position_ecef_m.z_m += moving_target.kinematics.velocity_mps.z_mps * dt_sec;
  }
  EXPECT_TRUE(saw_event);
}

TEST(RadarCycleOutputBuilderTest, DetachRecorderStopsAutomaticDriving) {
  const ArExternalPoseInput platform = MakePlatformInput();
  ArExternalTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);

  // 第一个周期驱动 recorder。
  ArCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  session.StepWithResult(input);
  const std::size_t first_size = recorder.GetLastEvents().size();

  // 解除注册后再步进——recorder 不应被驱动，GetLastEvents 不变。
  session.AttachTrackLifecycleRecorder(nullptr);
  target.kinematics.position_ecef_m.x_m += target.kinematics.velocity_mps.x_mps * 1.0;
  target.kinematics.position_ecef_m.y_m += target.kinematics.velocity_mps.y_mps * 1.0;
  input.cycle_index = 2U;
  input.targets.clear();
  input.targets.push_back(target);
  session.StepWithResult(input);
  EXPECT_EQ(recorder.GetLastEvents().size(), first_size);
}

TEST(RadarCycleOutputBuilderTest, SessionWithoutRecorderIsBackwardCompatible) {
  const ArExternalPoseInput platform = MakePlatformInput();
  const ArExternalTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());

  ArCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  const ArCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, airborne_radar::session::ArCycleStatus::kCompleted);
}

TEST(RadarCycleOutputBuilderTest, GetLastEventsEmptyAfterConstruction) {
  ArTrackLifecycleRecorder recorder;
  EXPECT_TRUE(recorder.GetLastEvents().empty());
}

TEST(RadarCycleOutputBuilderTest, NonExecutedCycleDoesNotUpdateLastEvents) {
  const ArExternalPoseInput platform = MakePlatformInput();
  ArExternalTargetInput target = MakeTargetInput();
  ArSession session = ArSession::Create(MakeDetectionFocusedConfig());
  ArTrackLifecycleRecorder recorder;
  session.AttachTrackLifecycleRecorder(&recorder);

  // 第一个周期执行并驱动 recorder。
  ArCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0;
  input.platform = platform;
  input.targets.push_back(target);
  session.StepWithResult(input);
  const std::size_t first_size = recorder.GetLastEvents().size();

  // 非法输入（dt_sec=0）→ validation rejection → 非执行周期，缓存保持不变。
  ArCycleInput invalid = input;
  invalid.dt_sec = 0.0;
  const ArCycleResult rejected = session.StepWithResult(invalid);
  EXPECT_EQ(rejected.status, airborne_radar::session::ArCycleStatus::kRejectedInvalidInput);
  EXPECT_EQ(recorder.GetLastEvents().size(), first_size);
}
