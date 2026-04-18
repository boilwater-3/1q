// Copyright 2026. All Rights Reserved.
//
// @file public_api_convenience_test.cpp
// @brief 验证对外易用性增强 API 的默认语义与集成行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/presets/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/output/TrackOutputQueries.h"
#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/session/MutableRadarContext.h"
#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"
#include "common/geometry/GeometryTransform.h"

namespace airborne_radar {
namespace tests {

namespace {

config::PipelineConfig MakeConveniencePipelineConfig() {
  const session::RadarSessionConfig session_config = config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::semantic::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::semantic::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::semantic::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
  return session::BuildLegacyPipelineConfig(session_config);
}

session::RadarSessionConfig MakeConvenienceSessionConfig() {
  return session::BuildRadarSessionConfig(MakeConveniencePipelineConfig());
}

session::RadarCycleInput MakeCycleInput(model::TargetFeatureList targets, float dt_sec = 1.0f) {
  session::RadarCycleInput input;
  input.target_features = std::move(targets);
  input.dt_sec = dt_sec;
  input.platform_pose.attitude_deg.yaw_deg = 5.0f;
  input.platform_pose.attitude_deg.pitch_deg = -1.0f;
  input.platform_pose.attitude_deg.roll_deg = 2.0f;
  return input;
}

/// @brief 比较两组控制命令的类型和来源是否一致。
/// @param expected 期望命令列表。
/// @param actual 实际命令列表。
void ExpectEquivalentCommands(const std::vector<extension::control::RadarCommand>& expected,
                              const std::vector<extension::control::RadarCommand>& actual) {
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(expected[i].type, actual[i].type);
    EXPECT_EQ(expected[i].source, actual[i].source);
  }
}

/// @brief 比较两份控制真值的公开语义字段。
/// @param expected 期望控制真值。
/// @param actual 实际控制真值。
void ExpectEquivalentProfiles(const extension::control::RadarControlProfile& expected,
                              const extension::control::RadarControlProfile& actual) {
  EXPECT_EQ(expected.version, actual.version);
  EXPECT_EQ(expected.enable_lpi_power_control, actual.enable_lpi_power_control);
  EXPECT_NEAR(expected.lpi_power_scale, actual.lpi_power_scale, 1e-5f);
  EXPECT_EQ(expected.enable_lpi_beamforming, actual.enable_lpi_beamforming);
  EXPECT_NEAR(expected.lpi_dwell_scale, actual.lpi_dwell_scale, 1e-5f);
  EXPECT_EQ(expected.enable_agility_frequency, actual.enable_agility_frequency);
  EXPECT_EQ(expected.agility_frequency_hop_phase, actual.agility_frequency_hop_phase);
  EXPECT_EQ(expected.enable_sidelobe_canceller, actual.enable_sidelobe_canceller);
  EXPECT_EQ(expected.enable_adaptive_beamforming, actual.enable_adaptive_beamforming);
  EXPECT_EQ(expected.enable_eccm_rejitter, actual.enable_eccm_rejitter);
  EXPECT_NEAR(expected.eccm_burnthrough_gain, actual.eccm_burnthrough_gain, 1e-5f);
}

/// @brief 判断问题列表是否包含指定编码。
/// @param issues 校验问题列表。
/// @param code 目标编码。
/// @return 若存在则返回 true。
bool ContainsIssueCode(const std::vector<session::ValidationIssue>& issues,
                       session::ValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

class RecordingRadarContext : public extension::IRadarContext {
 public:
  void BeginCycle(const session::RadarCycleInput& input) override {
    ++begin_cycle_count_;
    target_features_ = input.target_features;
    platform_attitude_deg_.yaw_deg = input.platform_pose.attitude_deg.yaw_deg;
    platform_attitude_deg_.pitch_deg = input.platform_pose.attitude_deg.pitch_deg;
    platform_attitude_deg_.roll_deg = input.platform_pose.attitude_deg.roll_deg;
    cycle_dt_sec_ = input.dt_sec;
    submitted_commands_.clear();
  }

  const model::TargetFeatureList& GetTargetFeatures() const override { return target_features_; }

  model::PlatformAttitudeDeg GetPlatformAttitude() const override { return platform_attitude_deg_; }

  float GetCycleDeltaTimeSec() const override { return cycle_dt_sec_; }

  void SubmitControlCommand(extension::control::RadarCommand cmd) override {
    submitted_commands_.push_back(std::move(cmd));
  }

  void UpdateRadarControlProfile(const extension::control::RadarControlProfile& profile) override {
    latest_control_profile_ = profile;
    has_latest_control_profile_ = true;
  }

  const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const override {
    return submitted_commands_;
  }

  bool HasLatestControlProfile() const override { return has_latest_control_profile_; }

  const extension::control::RadarControlProfile& GetLatestControlProfile() const override {
    return latest_control_profile_;
  }

  extension::RadarContextRuntimeState CaptureRuntimeState() const override {
    extension::RadarContextRuntimeState state;
    state.target_features = target_features_;
    state.platform_attitude_deg = platform_attitude_deg_;
    state.cycle_dt_sec = cycle_dt_sec_;
    state.submitted_commands = submitted_commands_;
    state.latest_control_profile = latest_control_profile_;
    state.has_latest_control_profile = has_latest_control_profile_;
    return state;
  }

  void RestoreRuntimeState(const extension::RadarContextRuntimeState& state) override {
    target_features_ = state.target_features;
    platform_attitude_deg_ = state.platform_attitude_deg;
    cycle_dt_sec_ = state.cycle_dt_sec;
    submitted_commands_ = state.submitted_commands;
    latest_control_profile_ = state.latest_control_profile;
    has_latest_control_profile_ = state.has_latest_control_profile;
  }

  std::size_t begin_cycle_count() const { return begin_cycle_count_; }

 private:
  model::TargetFeatureList target_features_{};
  model::PlatformAttitudeDeg platform_attitude_deg_{};
  float cycle_dt_sec_{1.0f};
  std::vector<extension::control::RadarCommand> submitted_commands_{};
  extension::control::RadarControlProfile latest_control_profile_{};
  bool has_latest_control_profile_{false};
  std::size_t begin_cycle_count_{0U};
};

class RecordingEnvironmentService : public environment::IEnvironmentService {
 public:
  void BeginCycle(const environment::EnvironmentCycleContext& cycle_context) override {
    ++begin_cycle_count_;
    cycle_context_ = cycle_context;
    active_scene_state_ = pending_scene_state_;
  }

  environment::EnvironmentSnapshot SampleEnvironment() const override {
    environment::EnvironmentSnapshot snapshot;
    snapshot.cycle_dt_sec = cycle_context_.dt_sec;
    snapshot.jammer_sources.reserve(active_scene_state_.jammer_emitters.size());
    for (std::size_t i = 0; i < active_scene_state_.jammer_emitters.size(); ++i) {
      const environment::JammerEmitterState& emitter = active_scene_state_.jammer_emitters[i];
      environment::JammerSourceFact source;
      source.technique = emitter.technique;
      source.power_db = emitter.power_db;
      source.js_db = emitter.js_db;
      source.has_direction_deg = emitter.has_direction_deg;
      if (source.has_direction_deg) {
        source.direction_deg.azimuth_deg = emitter.azimuth_deg;
        source.direction_deg.elevation_deg = emitter.elevation_deg;
      }
      source.angular_span_deg = emitter.angular_span_deg;
      source.confidence = emitter.confidence;
      snapshot.jammer_sources.push_back(source);
    }
    snapshot.jamming_detected = !snapshot.jammer_sources.empty();
    return snapshot;
  }

  void UpdateSceneState(const environment::EnvironmentSceneState& scene_state) override {
    ++update_scene_state_count_;
    pending_scene_state_ = scene_state;
  }

  environment::EnvironmentSceneState GetPendingSceneState() const override {
    return pending_scene_state_;
  }

  void UpdateModelConfig(const environment::EnvironmentModelConfig& config) override {
    ++update_model_config_count_;
    model_config_ = config;
  }

  void SetJammingSensitivityProfile(
      environment::JammingSensitivityProfile profile) override {
    ++set_sensitivity_count_;
    jamming_sensitivity_profile_ = profile;
  }

  environment::EnvironmentServiceRuntimeState CaptureRuntimeState() const override {
    environment::EnvironmentServiceRuntimeState state;
    state.active_scene_state = active_scene_state_;
    state.pending_scene_state = pending_scene_state_;
    state.active_cycle_context = cycle_context_;
    state.jamming_sensitivity_profile = jamming_sensitivity_profile_;
    return state;
  }

  void RestoreRuntimeState(const environment::EnvironmentServiceRuntimeState& state) override {
    active_scene_state_ = state.active_scene_state;
    pending_scene_state_ = state.pending_scene_state;
    cycle_context_ = state.active_cycle_context;
    jamming_sensitivity_profile_ = state.jamming_sensitivity_profile;
  }

  std::size_t begin_cycle_count() const { return begin_cycle_count_; }
  std::size_t update_scene_state_count() const { return update_scene_state_count_; }
  std::size_t update_model_config_count() const { return update_model_config_count_; }
  std::size_t set_sensitivity_count() const { return set_sensitivity_count_; }
  environment::JammingSensitivityProfile jamming_sensitivity_profile() const {
    return jamming_sensitivity_profile_;
  }

 private:
  environment::EnvironmentSceneState active_scene_state_{};
  environment::EnvironmentSceneState pending_scene_state_{};
  environment::EnvironmentModelConfig model_config_{};
  environment::EnvironmentCycleContext cycle_context_{};
  environment::JammingSensitivityProfile jamming_sensitivity_profile_{
      environment::JammingSensitivityProfile::kBalanced};
  std::size_t begin_cycle_count_{0U};
  std::size_t update_scene_state_count_{0U};
  std::size_t update_model_config_count_{0U};
  std::size_t set_sensitivity_count_{0U};
};

class RecordingSignalPipeline : public extension::ISignalPipeline {
 public:
  extension::SignalCycleResult RunCycle(
      const model::TargetFeatureList& input_state,
      const environment::IEnvironmentService& environment) override {
    (void)input_state;
    (void)environment;
    ++run_cycle_count_;
    extension::SignalCycleResult result;
    result.executed_this_cycle = should_execute_;
    result.abort_reason = should_execute_
                              ? extension::SignalCycleAbortReason::kNone
                              : extension::SignalCycleAbortReason::kRuntimePreparationFailed;
    return result;
  }

  void UpdatePlatformAttitude(const model::PlatformAttitudeDeg& platform_attitude_deg) override {
    platform_attitude_deg_ = platform_attitude_deg;
  }

  model::PlatformAttitudeDeg GetPlatformAttitude() const override { return platform_attitude_deg_; }

  void SetControlProfile(const extension::control::RadarControlProfile& control_profile) override {
    control_profile_ = control_profile;
  }

  extension::control::RadarControlProfile GetControlProfile() const override {
    return control_profile_;
  }

  bool UpdateConfig(config::PipelineConfig config) override {
    ++update_config_count_;
    if (!should_accept_updates_) {
      return false;
    }
    config_ = std::move(config);
    return true;
  }

  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const override {
    return {};
  }

  extension::SignalPipelineRuntimeState CaptureRuntimeState() const override {
    std::shared_ptr<RuntimeState> state(new RuntimeState());
    state->platform_attitude_deg = platform_attitude_deg_;
    state->control_profile = control_profile_;
    state->config = config_;
    state->should_execute = should_execute_;
    state->should_accept_updates = should_accept_updates_;
    state->run_cycle_count = run_cycle_count_;
    state->update_config_count = update_config_count_;
    extension::SignalPipelineRuntimeState runtime_state;
    runtime_state.owner_identity = this;
    runtime_state.schema_version = 1U;
    runtime_state.opaque = state;
    return runtime_state;
  }

  void RestoreRuntimeState(const extension::SignalPipelineRuntimeState& state) override {
    if (state.owner_identity != this || state.schema_version != 1U) {
      return;
    }
    const std::shared_ptr<RuntimeState> snapshot =
        std::static_pointer_cast<RuntimeState>(state.opaque);
    if (snapshot == nullptr) {
      return;
    }
    platform_attitude_deg_ = snapshot->platform_attitude_deg;
    control_profile_ = snapshot->control_profile;
    config_ = snapshot->config;
    should_execute_ = snapshot->should_execute;
    should_accept_updates_ = snapshot->should_accept_updates;
    run_cycle_count_ = snapshot->run_cycle_count;
    update_config_count_ = snapshot->update_config_count;
  }

  void SetShouldExecute(bool should_execute) { should_execute_ = should_execute; }
  void SetShouldAcceptUpdates(bool should_accept_updates) {
    should_accept_updates_ = should_accept_updates;
  }
  std::size_t run_cycle_count() const { return run_cycle_count_; }
  std::size_t update_config_count() const { return update_config_count_; }
  const config::PipelineConfig& config() const { return config_; }

 private:
  struct RuntimeState {
    model::PlatformAttitudeDeg platform_attitude_deg{};
    extension::control::RadarControlProfile control_profile{};
    config::PipelineConfig config{};
    bool should_execute{true};
    bool should_accept_updates{true};
    std::size_t run_cycle_count{0U};
    std::size_t update_config_count{0U};
  };

  model::PlatformAttitudeDeg platform_attitude_deg_{};
  extension::control::RadarControlProfile control_profile_{};
  config::PipelineConfig config_{};
  bool should_execute_{true};
  bool should_accept_updates_{true};
  std::size_t run_cycle_count_{0U};
  std::size_t update_config_count_{0U};
};

class NoopDecisionEngine : public extension::ITacticalDecisionEngine {
 public:
  extension::TacticalDecisionResult Evaluate(const model::DecisionInputFrame& input_frame,
                                             extension::TacticalStateStore& state_store) override {
    (void)input_frame;
    (void)state_store;
    return {};
  }
};

}  // namespace

TEST(PublicApiConvenienceTest, MutableRadarContextBeginsCycleAndResetsPerCycleCommands) {
  session::MutableRadarContext context;
  session::RadarCycleInput input = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(101U, 120.0f, -5.0f, 18.0f, 62.0f, 0.0f, 0.0f, 1.0f),
      },
      0.5f);

  context.BeginCycle(input);
  ASSERT_EQ(context.GetTargetFeatures().size(), 1U);
  EXPECT_NEAR(context.GetPlatformAttitude().yaw_deg, 5.0f, 1e-5f);
  EXPECT_NEAR(context.GetCycleDeltaTimeSec(), 0.5f, 1e-5f);
  EXPECT_TRUE(context.GetSubmittedCommands().empty());

  context.SubmitControlCommand(
      extension::control::RadarCommand(extension::control::RadarCommandType::SET_AGILITY_FREQ,
                                       extension::control::RadarCommandSource::ECCM));
  extension::control::RadarControlProfile profile;
  profile.enable_agility_frequency = true;
  profile.version = 4U;
  context.UpdateRadarControlProfile(profile);

  ASSERT_EQ(context.GetSubmittedCommands().size(), 1U);
  ASSERT_TRUE(context.HasLatestControlProfile());
  EXPECT_TRUE(context.GetLatestControlProfile().enable_agility_frequency);
  EXPECT_EQ(context.GetLatestControlProfile().version, 4U);

  session::RadarCycleInput second_input = MakeCycleInput({}, 1.25f);
  context.BeginCycle(second_input);
  EXPECT_TRUE(context.GetSubmittedCommands().empty());
  EXPECT_NEAR(context.GetCycleDeltaTimeSec(), 1.25f, 1e-5f);
  EXPECT_TRUE(context.HasLatestControlProfile());
  EXPECT_EQ(context.GetLatestControlProfile().version, 4U);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsBuildCartesianGroundAndAirTargets) {
  const model::TargetFeature cartesian_target =
      model::MakeTargetFromCartesian(201U, 3.0f, 4.0f, 12.0f, 10.0f, -2.0f, 1.0f, 0.9f, 2);
  EXPECT_EQ(cartesian_target.external_target_id, 201U);
  EXPECT_TRUE(cartesian_target.has_cartesian_position);
  EXPECT_NEAR(cartesian_target.range_m, 13.0f, 1e-5f);
  EXPECT_NEAR(cartesian_target.current_track_speed, std::sqrt(105.0f), 1e-5f);
  EXPECT_EQ(cartesian_target.target_swerling_type, 2);

  const model::TargetFeature ground_target = model::MakeGroundTarget(202U, 20.0f, -15.0f, 0.8f);
  EXPECT_TRUE(ground_target.has_cartesian_position);
  EXPECT_NEAR(ground_target.position_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.current_track_velocity_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.range_m, 25.0f, 1e-5f);

  const model::TargetFeature air_target =
      model::MakeAirTarget(203U, 30.0f, 40.0f, 50.0f, 90.0f, -3.0f, 4.0f, 1.2f);
  EXPECT_TRUE(air_target.has_cartesian_position);
  EXPECT_NEAR(air_target.range_m, std::sqrt(5000.0f), 1e-5f);
  EXPECT_NEAR(air_target.current_track_speed, std::sqrt(8125.0f), 1e-5f);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsNormalizeGeometryOnlyWhenCartesianPositionExists) {
  model::TargetFeature normalized =
      model::MakeAirTarget(301U, 6.0f, 8.0f, 0.0f, 50.0f, 0.0f, 0.0f, 1.0f);
  normalized.range_m = 0.0f;
  model::NormalizeTargetGeometry(&normalized);
  EXPECT_NEAR(normalized.range_m, 10.0f, 1e-5f);

  model::TargetFeature unresolved;
  unresolved.external_target_id = 302U;
  unresolved.range_m = 0.0f;
  model::NormalizeTargetGeometry(&unresolved);
  EXPECT_NEAR(unresolved.range_m, 0.0f, 1e-5f);

  model::TargetFeatureList targets;
  targets.push_back(normalized);
  targets.back().range_m = -1.0f;
  targets.push_back(unresolved);
  model::NormalizeTargetGeometry(&targets);
  EXPECT_NEAR(targets[0].range_m, 10.0f, 1e-5f);
  EXPECT_NEAR(targets[1].range_m, 0.0f, 1e-5f);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsComposeRadarAttitudeUsesRotationComposition) {
  model::EulerAnglesDeg platform_attitude;
  platform_attitude.yaw_deg = 40.0f;
  platform_attitude.pitch_deg = 12.0f;
  platform_attitude.roll_deg = -18.0f;

  model::EulerAnglesDeg mount_angles;
  mount_angles.yaw_deg = 3.0f;
  mount_angles.pitch_deg = -2.5f;
  mount_angles.roll_deg = 1.5f;

  const oneq::foundation::EulerAnglesDeg composed =
      session::ComposeRadarAttitudeDeg(platform_attitude, mount_angles);

  oneq::internal::geometry::EulerAnglesDeg platform_geometry;
  platform_geometry.yaw_deg = platform_attitude.yaw_deg;
  platform_geometry.pitch_deg = platform_attitude.pitch_deg;
  platform_geometry.roll_deg = platform_attitude.roll_deg;
  oneq::internal::geometry::EulerAnglesDeg mount_geometry;
  mount_geometry.yaw_deg = mount_angles.yaw_deg;
  mount_geometry.pitch_deg = mount_angles.pitch_deg;
  mount_geometry.roll_deg = mount_angles.roll_deg;
  oneq::internal::geometry::EulerAnglesDeg composed_geometry;
  composed_geometry.yaw_deg = composed.yaw_deg;
  composed_geometry.pitch_deg = composed.pitch_deg;
  composed_geometry.roll_deg = composed.roll_deg;

  const Eigen::Matrix3f expected =
      oneq::internal::geometry::BuildRotationMatrix(platform_geometry) *
      oneq::internal::geometry::BuildRotationMatrix(mount_geometry);
  const Eigen::Matrix3f actual = oneq::internal::geometry::BuildRotationMatrix(composed_geometry);

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), 1.0e-4f);
    }
  }
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsBuildsTargetFromExternalCoordinates) {
  oneq::foundation::LlaCoordinateDegM radar_lla;
  radar_lla.latitude_deg = 0.0;
  radar_lla.longitude_deg = 0.0;
  radar_lla.altitude_m = 0.0;
  oneq::foundation::EcefCoordinateM radar_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(radar_lla, &radar_ecef));

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));

  session::TargetExternalKinematicsInput input;
  input.radar_position_ecef_m = radar_ecef;
  input.target_position_ecef_m = target_ecef;
  input.target_velocity_frame = session::VelocityFrame::kEcef;
  input.target_velocity_mps.x = 80.0f;
  input.target_velocity_mps.y = -30.0f;
  input.target_velocity_mps.z = 10.0f;
  input.rcs = 2.0f;
  input.swerling_type = 0;

  model::TargetFeature target_from_external;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(403U, input, &target_from_external));
  EXPECT_EQ(target_from_external.external_target_id, 403U);
  EXPECT_TRUE(target_from_external.has_cartesian_position);
  EXPECT_GT(target_from_external.position_x, 100.0f);
  EXPECT_NEAR(target_from_external.position_y, 0.0f, 2.0f);
  EXPECT_NEAR(target_from_external.position_z, 0.0f, 2.0f);
  EXPECT_GT(target_from_external.range_m, 100.0f);
  EXPECT_GT(target_from_external.current_track_speed, 0.0f);
}

TEST(PublicApiConvenienceTest, TargetFeatureUtilsBuildsTargetFromExternalKinematicsFrames) {
  oneq::foundation::LlaCoordinateDegM radar_lla;
  radar_lla.latitude_deg = 31.2304;
  radar_lla.longitude_deg = 121.4737;
  radar_lla.altitude_m = 200.0;
  oneq::foundation::EcefCoordinateM radar_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(radar_lla, &radar_ecef));

  oneq::foundation::LlaCoordinateDegM target_lla = radar_lla;
  target_lla.longitude_deg += 0.001;
  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));

  session::TargetExternalKinematicsInput input;
  input.radar_position_ecef_m = radar_ecef;
  input.target_position_ecef_m = target_ecef;
  input.platform_attitude_deg.yaw_deg = 5.0f;
  input.radar_mount_angles_deg.pitch_deg = 2.0f;
  input.rcs = 1.2f;

  // ECEF 速度 + 雷达自身 ECEF 速度补偿后应接近零相对速度。
  input.target_velocity_frame = session::VelocityFrame::kEcef;
  input.target_velocity_mps.x = 120.0f;
  input.target_velocity_mps.y = -70.0f;
  input.target_velocity_mps.z = 30.0f;
  input.has_radar_velocity_ecef_mps = true;
  input.radar_velocity_ecef_mps = input.target_velocity_mps;
  model::TargetFeature ecef_target;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(501U, input, &ecef_target));
  EXPECT_NEAR(ecef_target.current_track_speed, 0.0f, 1.0e-4f);

  // ENU 输入。
  input.target_velocity_frame = session::VelocityFrame::kEnu;
  input.has_radar_velocity_ecef_mps = false;
  input.target_velocity_mps.x = 200.0f;  // east
  input.target_velocity_mps.y = 0.0f;    // north
  input.target_velocity_mps.z = 0.0f;    // up
  model::TargetFeature enu_target;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(502U, input, &enu_target));
  EXPECT_GT(enu_target.current_track_speed, 100.0f);

  // NED 输入（north, east, down）。
  input.target_velocity_frame = session::VelocityFrame::kNed;
  input.target_velocity_mps.x = 100.0f;  // north
  input.target_velocity_mps.y = 50.0f;   // east
  input.target_velocity_mps.z = -20.0f;  // down
  model::TargetFeature ned_target;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(503U, input, &ned_target));
  EXPECT_GT(ned_target.current_track_speed, 50.0f);

  // RadarLocal 输入应直接保留速度分量。
  input.target_velocity_frame = session::VelocityFrame::kRadarLocal;
  input.target_velocity_mps.x = 11.0f;
  input.target_velocity_mps.y = 12.0f;
  input.target_velocity_mps.z = 13.0f;
  model::TargetFeature local_target;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(504U, input, &local_target));
  EXPECT_NEAR(local_target.current_track_velocity_x, 11.0f, 1.0e-5f);
  EXPECT_NEAR(local_target.current_track_velocity_y, 12.0f, 1.0e-5f);
  EXPECT_NEAR(local_target.current_track_velocity_z, 13.0f, 1.0e-5f);
}

TEST(PublicApiConvenienceTest, EnvironmentSceneBuilderDefaultsMatchEnvironmentSceneState) {
  const environment::EnvironmentSceneState built_scene =
      environment::EnvironmentSceneBuilder().Build();
  const environment::EnvironmentSceneState default_scene;

  EXPECT_EQ(built_scene.atmospheric_physics.enable_physical_model,
            default_scene.atmospheric_physics.enable_physical_model);
  EXPECT_TRUE(built_scene.jammer_emitters.empty());
}

TEST(PublicApiConvenienceTest, EnvironmentSceneBuilderJammerHelpersPopulateTypedEmitters) {
  environment::JammerEmitterState noise_emitter_1;
  noise_emitter_1.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_1.power_db = 11.0f;
  noise_emitter_1.js_db = 8.0f;
  noise_emitter_1.has_direction_deg = true;
  noise_emitter_1.azimuth_deg = 12.0f;
  noise_emitter_1.elevation_deg = 1.0f;
  noise_emitter_1.angular_span_deg = 5.0f;
  noise_emitter_1.confidence = 0.9f;

  environment::JammerEmitterState deception_emitter;
  deception_emitter.technique = environment::JammingTechnique::kDeception;
  deception_emitter.power_db = 9.0f;
  deception_emitter.js_db = 7.5f;
  deception_emitter.has_direction_deg = true;
  deception_emitter.azimuth_deg = -8.0f;
  deception_emitter.elevation_deg = 2.5f;
  deception_emitter.angular_span_deg = 3.0f;
  deception_emitter.confidence = 0.8f;

  environment::JammerEmitterState repeater_emitter;
  repeater_emitter.technique = environment::JammingTechnique::kRepeater;
  repeater_emitter.power_db = 8.5f;
  repeater_emitter.js_db = 6.5f;
  repeater_emitter.has_direction_deg = true;
  repeater_emitter.azimuth_deg = 4.0f;
  repeater_emitter.elevation_deg = -1.0f;
  repeater_emitter.angular_span_deg = 2.0f;
  repeater_emitter.confidence = 0.7f;

  const environment::EnvironmentSceneState scene = environment::EnvironmentSceneBuilder()
                                                       .AddNoiseJammer(noise_emitter_1)
                                                       .AddDeceptionJammer(deception_emitter)
                                                       .AddRepeaterJammer(repeater_emitter)
                                                       .Build();

  ASSERT_EQ(scene.jammer_emitters.size(), 3U);
  EXPECT_EQ(scene.jammer_emitters[0].technique, environment::JammingTechnique::kNoiseSuppression);
  EXPECT_TRUE(scene.jammer_emitters[0].has_direction_deg);
  EXPECT_EQ(scene.jammer_emitters[1].technique, environment::JammingTechnique::kDeception);
  EXPECT_NEAR(scene.jammer_emitters[1].azimuth_deg, -8.0f, 1e-5f);
  EXPECT_EQ(scene.jammer_emitters[2].technique, environment::JammingTechnique::kRepeater);
  EXPECT_NEAR(scene.jammer_emitters[2].elevation_deg, -1.0f, 1e-5f);
}

TEST(PublicApiConvenienceTest, TrackOutputQueriesSupportUniqueDuplicateAndJammingSearch) {
  output::TrackOutputFrame frame;
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(10.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false, 401U, 1U));
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(20.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, true, 402U, 2U));
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(21.0f, 0.0f, 0.0f, 1.1f, 0.0f, 0.0f, 0.0f, false, 402U, 3U));
  frame.tracks.push_back(
      model::DecisionTrackSnapshot(8.0f, 0.0f, 0.0f, 0.7f, 0.0f, 0.0f, 0.0f, true, 0U, 4U));

  const auto track_map = output::BuildTrackMapByExternalTargetId(frame);
  EXPECT_EQ(track_map.size(), 2U);
  ASSERT_NE(track_map.count(401U), 0U);
  ASSERT_NE(track_map.count(402U), 0U);
  EXPECT_EQ(track_map.at(402U).state.association_key, 3U);

  const model::DecisionTrackSnapshotList duplicate_tracks =
      output::CollectTracksByExternalTargetId(frame, 402U);
  EXPECT_EQ(duplicate_tracks.size(), 2U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 401U));
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 0U));
  EXPECT_FALSE(output::ContainsExternalTargetId(frame, 999U));
  EXPECT_EQ(output::CountJammingTracks(frame), 2U);
  EXPECT_EQ(output::CountTracksByStatus(frame, model::DecisionTrackStatus::kTentative), 4U);
}

TEST(PublicApiConvenienceTest, TrackOutputQueriesSupportAssociationKeyStatusAndJammingCollections) {
  output::TrackOutputFrame frame;
  model::DecisionTrackSnapshot confirmed(20.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false, 410U,
                                         11U);
  confirmed.state.status = model::DecisionTrackStatus::kConfirmed;
  model::DecisionTrackSnapshot lost(5.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, false, 411U, 12U);
  lost.state.status = model::DecisionTrackStatus::kLost;
  model::DecisionTrackSnapshot jammed(18.0f, 0.0f, 0.0f, 1.2f, 0.0f, 0.0f, 0.0f, true, 412U, 13U);
  jammed.state.status = model::DecisionTrackStatus::kConfirmed;
  frame.tracks.push_back(confirmed);
  frame.tracks.push_back(lost);
  frame.tracks.push_back(jammed);

  const auto track_map = output::BuildTrackMapByAssociationKey(frame);
  ASSERT_EQ(track_map.size(), 3U);
  EXPECT_EQ(track_map.at(13U).state.external_target_id, 412U);
  EXPECT_EQ(output::CollectConfirmedTracks(frame).size(), 2U);
  EXPECT_EQ(output::CollectLostTracks(frame).size(), 1U);
  EXPECT_EQ(output::CollectJammingTracks(frame).size(), 1U);
  EXPECT_EQ(output::CountTracksByStatus(frame, model::DecisionTrackStatus::kConfirmed), 2U);
}

TEST(PublicApiConvenienceTest, RadarInputValidationReportsWarningsAndErrorsForCommonBoundaryCases) {
  session::RadarCycleInput input = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
          model::MakeAirTarget(800U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f, 1.0f),
          model::MakeAirTarget(800U, 155.0f, 2.0f, 12.0f, 63.0f, -0.2f, 0.0f, -1.1f),
      },
      0.0f);
  input.target_features[2].position_x = std::numeric_limits<float>::infinity();
  input.target_features[2].range_m = 0.0f;

  const std::vector<session::ValidationIssue> issues = session::ValidateRadarCycleInput(input);
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kUnknownExternalTargetId));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kDuplicateExternalTargetId));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kNegativeRcs));
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kNonFiniteTargetField));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(PublicApiConvenienceTest, RadarInputValidationFlagsMissingGeometryAndNonFiniteCycleDelta) {
  session::RadarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  model::TargetFeature invalid_target;
  invalid_target.external_target_id = 900U;
  invalid_target.range_m = 0.0f;
  input.target_features.push_back(invalid_target);

  const std::vector<session::ValidationIssue> issues = session::ValidateRadarCycleInput(input);
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(
      ContainsIssueCode(issues, session::ValidationCode::kMissingRangeAndCartesianPosition));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(PublicApiConvenienceTest, ConfigPresetsProvideExpectedDetectionAndRobustnessDefaults) {
  const config::PipelineConfig detection_config =
      config::presets::MakeDetectionMissionPipelineConfig();
  EXPECT_EQ(detection_config.expert.lifecycle.confirm_hits, 1U);
  EXPECT_EQ(detection_config.expert.lifecycle.max_miss_before_lost, 1U);
  EXPECT_EQ(detection_config.expert.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(detection_config.expert.detection.detection_policy.min_snr_db, -12.0f);

  const config::PipelineConfig robust_config =
      config::presets::MakeHighRobustnessPipelineConfig();
  EXPECT_EQ(robust_config.expert.lifecycle.confirm_hits, 3U);
  EXPECT_EQ(robust_config.expert.lifecycle.max_miss_before_lost, 3U);
  EXPECT_EQ(robust_config.expert.tracking.kalman_update_backend,
            config::expert::KalmanUpdateBackend::kUdKf);
  EXPECT_FLOAT_EQ(robust_config.expert.association.unassigned_cost, 12.0f);

  const session::RadarSessionConfig session_config =
      config::presets::MakeDetectionMissionRadarSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(session_config);
  const output::TrackOutputFrame frame = session.Step(MakeCycleInput(model::TargetFeatureList{
      model::MakeGroundTarget(901U, 20.0f, 5.0f, 0.8f),
  }));
  EXPECT_EQ(frame.confirmed_track_count, 1U);
}

TEST(PublicApiConvenienceTest, DefaultSessionConfigUsesLifecycleManagedTracks) {
  const session::RadarSessionConfig default_config = config::presets::MakeDefaultRadarSessionConfig();

  session::RadarSession session = session::RadarSessionFactory::Create();
  const output::TrackOutputFrame frame = session.Step(MakeCycleInput(model::TargetFeatureList{
      model::MakeGroundTarget(902U, 15.0f, -3.0f, 0.8f),
  }));

  ASSERT_EQ(frame.tracks.size(), 1U);
  EXPECT_EQ(frame.published_track_count, 1U);
  EXPECT_EQ(frame.confirmed_track_count, 0U);
  EXPECT_EQ(frame.tracks[0].state.status, model::DecisionTrackStatus::kTentative);
}

TEST(PublicApiConvenienceTest, RadarSessionStepProducesReadableOutputWithoutInterference) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());
  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeGroundTarget(501U, 25.0f, -5.0f, 0.7f),
      model::MakeAirTarget(502U, 150.0f, 6.0f, 18.0f, 72.0f, 1.0f, 0.0f, 1.0f),
  });

  const output::TrackOutputFrame frame = session.Step(input);
  EXPECT_EQ(frame.published_track_count, 2U);
  EXPECT_EQ(frame.confirmed_track_count, 2U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 501U));
  EXPECT_TRUE(output::ContainsExternalTargetId(frame, 502U));
  EXPECT_EQ(output::CountJammingTracks(frame), 0U);
  EXPECT_TRUE(session.GetSubmittedCommands().empty());
  EXPECT_TRUE(session.HasLatestControlProfile());
}

TEST(PublicApiConvenienceTest, RadarSessionSceneAwareStepMatchesManualControllerChain) {
  const session::RadarSessionConfig config = MakeConvenienceSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  session::MutableRadarContext manual_context;
  config::PipelineConfig pipeline_config = session::BuildLegacyPipelineConfig(config);
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);
  environment::EnvironmentService environment_service(
      environment::BuildModelConfigFromScenario(config.environment.scenario_config));
  environment_service.SetJammingSensitivityProfile(
      config.environment.jamming_sensitivity_profile);
  extension::RadarController controller(manual_context, signal_pipeline, environment_service);

  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(601U, 180.0f, -4.0f, 16.0f, 60.0f, 0.0f, 0.0f, 1.0f),
      model::MakeAirTarget(602U, 240.0f, 8.0f, 18.0f, 76.0f, 0.5f, 0.0f, 1.1f),
  });
  environment::JammerEmitterState noise_emitter_2;
  noise_emitter_2.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_2.power_db = 12.0f;
  noise_emitter_2.js_db = 8.0f;
  noise_emitter_2.has_direction_deg = true;
  noise_emitter_2.azimuth_deg = 20.0f;
  noise_emitter_2.elevation_deg = 1.0f;
  noise_emitter_2.angular_span_deg = 10.0f;

  const environment::EnvironmentSceneState scene =
      environment::EnvironmentSceneBuilder().AddNoiseJammer(noise_emitter_2).Build();

  const output::TrackOutputFrame session_frame = session.Step(input, scene);

  environment_service.UpdateSceneState(scene);
  manual_context.BeginCycle(input);
  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame manual_frame = controller.GetLatestTrackOutputFrame();

  EXPECT_EQ(session_frame.published_track_count, manual_frame.published_track_count);
  EXPECT_EQ(session_frame.confirmed_track_count, manual_frame.confirmed_track_count);
  EXPECT_EQ(output::CountJammingTracks(session_frame), output::CountJammingTracks(manual_frame));

  const auto session_track_map = output::BuildTrackMapByExternalTargetId(session_frame);
  const auto manual_track_map = output::BuildTrackMapByExternalTargetId(manual_frame);
  ASSERT_EQ(session_track_map.size(), manual_track_map.size());
  for (std::size_t i = 0; i < input.target_features.size(); ++i) {
    const std::uint64_t target_id = input.target_features[i].external_target_id;
    ASSERT_NE(session_track_map.count(target_id), 0U);
    ASSERT_NE(manual_track_map.count(target_id), 0U);
    EXPECT_EQ(session_track_map.at(target_id).state.association_key,
              manual_track_map.at(target_id).state.association_key);
  }

  ExpectEquivalentCommands(manual_context.GetSubmittedCommands(), session.GetSubmittedCommands());
  ASSERT_TRUE(session.HasLatestControlProfile());
  ASSERT_TRUE(manual_context.HasLatestControlProfile());
  ExpectEquivalentProfiles(manual_context.GetLatestControlProfile(),
                           session.GetLatestControlProfile());

  const extension::AssociationQualityMetrics session_metrics =
      session.GetLastAssociationQualityMetrics();
  const extension::AssociationQualityMetrics manual_metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  EXPECT_EQ(session_metrics.detection_count, manual_metrics.detection_count);
  EXPECT_NEAR(session_metrics.match_rate, manual_metrics.match_rate, 1e-5f);
  EXPECT_NEAR(session_metrics.association_stress, manual_metrics.association_stress, 1e-5f);
}

TEST(PublicApiConvenienceTest,
     RadarSessionRejectsDuplicateIdsAndRetainsPreviousOutputAcrossInvalidCycles) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());

  session::RadarCycleInput cycle_1 = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
          model::MakeAirTarget(701U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f, 1.0f),
          model::MakeGroundTarget(702U, 40.0f, -6.0f, 0.8f),
      },
      1.0f);
  const session::RadarCycleResult result_1 = session.StepWithResult(cycle_1);
  const output::TrackOutputFrame& frame_1 = result_1.track_output_frame;
  EXPECT_GT(frame_1.published_track_count, 0U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame_1, 0U));
  EXPECT_EQ(output::CollectTracksByExternalTargetId(frame_1, 701U).size(), 1U);
  EXPECT_FALSE(result_1.has_validation_error);
  EXPECT_TRUE(result_1.executed_this_cycle);
  EXPECT_FALSE(result_1.reused_previous_track_output);

  session::RadarCycleInput duplicate_cycle = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
          model::MakeAirTarget(701U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f, 1.0f),
          model::MakeAirTarget(701U, 155.0f, 2.0f, 12.0f, 63.0f, -0.2f, 0.0f, 1.1f),
          model::MakeGroundTarget(702U, 40.0f, -6.0f, 0.8f),
      },
      1.0f);
  const session::RadarCycleResult duplicate_result = session.StepWithResult(duplicate_cycle);
  EXPECT_TRUE(ContainsIssueCode(duplicate_result.validation_issues,
                                session::ValidationCode::kDuplicateExternalTargetId));
  EXPECT_TRUE(duplicate_result.has_validation_error);
  EXPECT_FALSE(duplicate_result.executed_this_cycle);
  EXPECT_TRUE(duplicate_result.reused_previous_track_output);
  EXPECT_EQ(duplicate_result.track_output_frame.cycle_index, frame_1.cycle_index);
  EXPECT_EQ(duplicate_result.track_output_frame.batch_id, frame_1.batch_id);
  EXPECT_EQ(duplicate_result.track_output_frame.published_track_count,
            frame_1.published_track_count);

  session::RadarCycleInput cycle_2 = cycle_1;
  cycle_2.dt_sec = -1.0f;
  for (std::size_t i = 0; i < cycle_2.target_features.size(); ++i) {
    cycle_2.target_features[i].position_x += 5.0f;
  }
  environment::JammerEmitterState noise_emitter_3;
  noise_emitter_3.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_3.power_db = 12.0f;
  noise_emitter_3.js_db = 8.0f;
  noise_emitter_3.has_direction_deg = true;
  noise_emitter_3.azimuth_deg = 20.0f;
  noise_emitter_3.elevation_deg = 1.0f;
  noise_emitter_3.angular_span_deg = 10.0f;

  const session::RadarCycleResult result_2 = session.StepWithResult(
      cycle_2, environment::EnvironmentSceneBuilder().AddNoiseJammer(noise_emitter_3).Build());
  const output::TrackOutputFrame& frame_2 = result_2.track_output_frame;

  EXPECT_TRUE(result_2.has_validation_error);
  EXPECT_FALSE(result_2.executed_this_cycle);
  EXPECT_TRUE(result_2.reused_previous_track_output);
  EXPECT_TRUE(ContainsIssueCode(result_2.validation_issues,
                                session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_EQ(frame_2.cycle_index, frame_1.cycle_index);
  EXPECT_EQ(frame_2.batch_id, frame_1.batch_id);
  EXPECT_EQ(frame_2.published_track_count, frame_1.published_track_count);
  EXPECT_EQ(output::CountJammingTracks(frame_2), output::CountJammingTracks(frame_1));
  EXPECT_TRUE(result_2.submitted_commands.empty());
  EXPECT_FALSE(result_2.has_control_profile);
  EXPECT_EQ(result_2.association_quality_metrics.detection_count, 0U);
  EXPECT_TRUE(session.HasLatestControlProfile());

  session::RadarCycleInput cycle_3 = cycle_2;
  cycle_3.dt_sec = 1.0f;
  cycle_3.target_features[1].external_target_id = 703U;
  cycle_3.target_features[2].external_target_id = 704U;
  const output::TrackOutputFrame frame_3 =
      session.Step(cycle_3, environment::EnvironmentSceneBuilder().Build());
  EXPECT_GT(frame_3.published_track_count, 0U);
  EXPECT_EQ(output::CountJammingTracks(frame_3), 0U);
  EXPECT_TRUE(output::ContainsExternalTargetId(frame_3, 703U));
  EXPECT_TRUE(output::ContainsExternalTargetId(frame_3, 704U));
}

TEST(PublicApiConvenienceTest,
     RadarSessionStagesRuntimePatchAndCommitsSceneOnlyAfterSuccessfulValidation) {
  RecordingRadarContext radar_context;
  RecordingSignalPipeline signal_pipeline;
  RecordingEnvironmentService environment_service;
  NoopDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);
  session::RadarSession session = session::RadarSessionFactory::CreateWithController(
      MakeConvenienceSessionConfig(), controller);

  session::RadarCycleInput baseline_input = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(810U, 120.0f, 1.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
      },
      1.0f);
  const session::RadarCycleResult baseline = session.StepWithResult(baseline_input);
  EXPECT_FALSE(baseline.has_validation_error);
  EXPECT_TRUE(baseline.executed_this_cycle);
  EXPECT_FALSE(baseline.reused_previous_track_output);
  ASSERT_EQ(radar_context.GetTargetFeatures().size(), 1U);
  EXPECT_EQ(radar_context.GetTargetFeatures()[0].external_target_id, 810U);
  EXPECT_FLOAT_EQ(radar_context.GetCycleDeltaTimeSec(), 1.0f);

  const std::size_t committed_begin_cycles = radar_context.begin_cycle_count();
  const std::size_t committed_signal_runs = signal_pipeline.run_cycle_count();
  const std::size_t committed_update_config_count = signal_pipeline.update_config_count();
  const std::size_t committed_update_scene_count = environment_service.update_scene_state_count();
  const std::size_t committed_update_model_count = environment_service.update_model_config_count();
  const std::size_t committed_threshold_count = environment_service.set_sensitivity_count();

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();
  session.ApplyRuntimeConfig(patch);

  EXPECT_EQ(signal_pipeline.update_config_count(), committed_update_config_count);
  EXPECT_EQ(environment_service.update_model_config_count(), committed_update_model_count);
  EXPECT_EQ(environment_service.set_sensitivity_count(), committed_threshold_count);

  environment::JammerEmitterState jammer;
  jammer.technique = environment::JammingTechnique::kNoiseSuppression;
  jammer.power_db = 12.0f;
  jammer.js_db = 8.0f;
  jammer.has_direction_deg = true;
  jammer.azimuth_deg = 18.0f;
  jammer.elevation_deg = 1.0f;
  jammer.angular_span_deg = 10.0f;

  environment::EnvironmentSceneState jammed_scene;
  jammed_scene.jammer_emitters.push_back(jammer);

  session::RadarCycleInput invalid_input = MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(811U, 140.0f, -2.0f, 12.0f, 60.0f, 0.0f, 0.0f, 1.0f),
      },
      -1.0f);
  const session::RadarCycleResult invalid_result =
      session.StepWithResult(invalid_input, jammed_scene);
  EXPECT_TRUE(invalid_result.has_validation_error);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_TRUE(invalid_result.reused_previous_track_output);
  EXPECT_TRUE(ContainsIssueCode(invalid_result.validation_issues,
                                session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_EQ(radar_context.begin_cycle_count(), committed_begin_cycles);
  EXPECT_EQ(signal_pipeline.run_cycle_count(), committed_signal_runs);
  EXPECT_EQ(signal_pipeline.update_config_count(), committed_update_config_count);
  EXPECT_EQ(environment_service.update_scene_state_count(), committed_update_scene_count);
  EXPECT_EQ(environment_service.update_model_config_count(), committed_update_model_count);
  EXPECT_EQ(environment_service.set_sensitivity_count(), committed_threshold_count);
  ASSERT_EQ(radar_context.GetTargetFeatures().size(), 1U);
  EXPECT_EQ(radar_context.GetTargetFeatures()[0].external_target_id, 810U);
  EXPECT_FLOAT_EQ(radar_context.GetCycleDeltaTimeSec(), 1.0f);
  EXPECT_TRUE(environment_service.GetPendingSceneState().jammer_emitters.empty());

  const session::RadarCycleResult committed_result =
      session.StepWithResult(baseline_input, jammed_scene);
  EXPECT_FALSE(committed_result.has_validation_error);
  EXPECT_TRUE(committed_result.executed_this_cycle);
  EXPECT_FALSE(committed_result.reused_previous_track_output);
  EXPECT_EQ(radar_context.begin_cycle_count(), committed_begin_cycles + 1U);
  EXPECT_EQ(signal_pipeline.run_cycle_count(), committed_signal_runs + 1U);
  EXPECT_EQ(signal_pipeline.update_config_count(), committed_update_config_count + 1U);
  EXPECT_EQ(environment_service.update_scene_state_count(), committed_update_scene_count + 1U);
  EXPECT_EQ(environment_service.update_model_config_count(), committed_update_model_count + 1U);
  EXPECT_EQ(environment_service.set_sensitivity_count(), committed_threshold_count + 1U);
  EXPECT_EQ(signal_pipeline.config().orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
  EXPECT_EQ(environment_service.jamming_sensitivity_profile(),
            environment::JammingSensitivityProfile::kStrict);
  EXPECT_EQ(environment_service.GetPendingSceneState().jammer_emitters.size(), 1U);
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultAggregatesCurrentCycleObservations) {
  const session::RadarSessionConfig config = config::presets::MakeDetectionMissionRadarSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(950U, 200.0f, -3.0f, 15.0f, 70.0f, 0.0f, 0.0f, 1.0f),
      model::MakeAirTarget(951U, 240.0f, 4.0f, 18.0f, 78.0f, 0.3f, 0.0f, 1.1f),
  });

  const session::RadarCycleResult result = session.StepWithResult(input);

  EXPECT_GT(result.track_output_frame.published_track_count, 0U);
  EXPECT_GE(result.track_output_frame.published_track_count,
            result.track_output_frame.confirmed_track_count);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_track_output);
  EXPECT_EQ(result.submitted_commands.size(), session.GetSubmittedCommands().size());
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.validation_issues.empty());
  EXPECT_EQ(result.has_control_profile, session.HasLatestControlProfile());
  if (result.has_control_profile) {
    ExpectEquivalentProfiles(result.control_profile, session.GetLatestControlProfile());
  }
  EXPECT_EQ(result.association_quality_metrics.detection_count,
            session.GetLastAssociationQualityMetrics().detection_count);
}

TEST(PublicApiConvenienceTest, RadarSessionReusesPreviousOutputWhenSignalPipelineAborts) {
  RecordingRadarContext radar_context;
  RecordingSignalPipeline signal_pipeline;
  RecordingEnvironmentService environment_service;
  NoopDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);
  session::RadarSession session = session::RadarSessionFactory::CreateWithController(
      MakeConvenienceSessionConfig(), controller);

  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(952U, 200.0f, -3.0f, 15.0f, 70.0f, 0.0f, 0.0f, 1.0f),
  });

  const session::RadarCycleResult baseline = session.StepWithResult(input);
  EXPECT_TRUE(baseline.executed_this_cycle);
  EXPECT_EQ(baseline.signal_cycle_abort_reason, extension::SignalCycleAbortReason::kNone);
  EXPECT_FALSE(baseline.reused_previous_track_output);

  signal_pipeline.SetShouldExecute(false);
  const session::RadarCycleResult failed = session.StepWithResult(input);
  EXPECT_FALSE(failed.has_validation_error);
  EXPECT_FALSE(failed.executed_this_cycle);
  EXPECT_EQ(failed.signal_cycle_abort_reason,
            extension::SignalCycleAbortReason::kRuntimePreparationFailed);
  EXPECT_TRUE(failed.reused_previous_track_output);
  EXPECT_TRUE(failed.submitted_commands.empty());
  EXPECT_FALSE(failed.has_control_profile);
  EXPECT_EQ(failed.association_quality_metrics.detection_count, 0U);
  EXPECT_EQ(failed.track_output_frame.cycle_index, baseline.track_output_frame.cycle_index);
  EXPECT_EQ(failed.track_output_frame.batch_id, baseline.track_output_frame.batch_id);
}

TEST(PublicApiConvenienceTest,
     RadarSessionRollsBackSceneRuntimePatchAndContextWhenSignalPipelineAborts) {
  RecordingRadarContext radar_context;
  RecordingSignalPipeline signal_pipeline;
  RecordingEnvironmentService environment_service;
  NoopDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);
  session::RadarSession session = session::RadarSessionFactory::CreateWithController(
      MakeConvenienceSessionConfig(), controller);

  const session::RadarCycleInput baseline_input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(960U, 200.0f, -3.0f, 15.0f, 70.0f, 0.0f, 0.0f, 1.0f),
  });
  const session::RadarCycleResult baseline = session.StepWithResult(baseline_input);
  ASSERT_TRUE(baseline.executed_this_cycle);

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();
  session.ApplyRuntimeConfig(patch);

  environment::JammerEmitterState jammer;
  jammer.technique = environment::JammingTechnique::kNoiseSuppression;
  jammer.power_db = 12.0f;
  jammer.js_db = 8.0f;
  jammer.has_direction_deg = true;
  jammer.azimuth_deg = 18.0f;
  jammer.elevation_deg = 1.0f;
  jammer.angular_span_deg = 10.0f;

  environment::EnvironmentSceneState jammed_scene;
  jammed_scene.jammer_emitters.push_back(jammer);

  signal_pipeline.SetShouldExecute(false);
  const session::RadarCycleInput failed_input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(961U, 140.0f, -2.0f, 12.0f, 60.0f, 0.0f, 0.0f, 1.0f),
  });
  const session::RadarCycleResult failed = session.StepWithResult(failed_input, jammed_scene);
  EXPECT_FALSE(failed.executed_this_cycle);
  EXPECT_EQ(failed.signal_cycle_abort_reason,
            extension::SignalCycleAbortReason::kRuntimePreparationFailed);
  EXPECT_TRUE(failed.reused_previous_track_output);
  ASSERT_EQ(radar_context.GetTargetFeatures().size(), 1U);
  EXPECT_EQ(radar_context.GetTargetFeatures()[0].external_target_id, 960U);
  EXPECT_FLOAT_EQ(radar_context.GetCycleDeltaTimeSec(), 1.0f);
  EXPECT_EQ(signal_pipeline.config().orientation.work_sub_mode,
            model::RadarWorkSubMode::kTws);
  EXPECT_EQ(environment_service.jamming_sensitivity_profile(),
            environment::JammingSensitivityProfile::kBalanced);
  EXPECT_TRUE(environment_service.GetPendingSceneState().jammer_emitters.empty());

  signal_pipeline.SetShouldExecute(true);
  const session::RadarCycleResult committed = session.StepWithResult(baseline_input, jammed_scene);
  EXPECT_TRUE(committed.executed_this_cycle);
  EXPECT_EQ(signal_pipeline.config().orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
  EXPECT_EQ(environment_service.jamming_sensitivity_profile(),
            environment::JammingSensitivityProfile::kStrict);
  EXPECT_EQ(environment_service.GetPendingSceneState().jammer_emitters.size(), 1U);
  EXPECT_EQ(committed.track_output_frame.cycle_index, baseline.track_output_frame.cycle_index + 1U);
  EXPECT_EQ(committed.track_output_frame.batch_id, baseline.track_output_frame.batch_id + 1U);
}

TEST(PublicApiConvenienceTest,
     RadarSessionRuntimePatchPreservesKnownTrackContinuityAcrossSuccessfulCycles) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());

  const session::RadarCycleResult cycle_1 = session.StepWithResult(MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(970U, 180.0f, 1.5f, 12.0f, 65.0f, 0.0f, 0.0f, 1.0f),
      }));
  ASSERT_TRUE(cycle_1.executed_this_cycle);
  ASSERT_EQ(cycle_1.track_output_frame.confirmed_track_count, 1U);
  ASSERT_EQ(cycle_1.track_output_frame.tracks.size(), 1U);
  const std::uint64_t baseline_key = cycle_1.track_output_frame.tracks[0].state.association_key;
  ASSERT_NE(baseline_key, 0U);

  session.ApplyRuntimeConfig(config::RadarRuntimeConfigBuilder()
                                 .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
                                 .Build());

  const session::RadarCycleResult cycle_2 = session.StepWithResult(MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(970U, 182.0f, 1.7f, 12.0f, 65.0f, 0.0f, 0.0f, 1.0f),
      }));
  ASSERT_TRUE(cycle_2.executed_this_cycle);
  ASSERT_GE(cycle_2.track_output_frame.confirmed_track_count, 1U);
  bool retained_known_track = false;
  for (std::size_t i = 0; i < cycle_2.track_output_frame.tracks.size(); ++i) {
    const model::DecisionTrackSnapshot& track = cycle_2.track_output_frame.tracks[i];
    if (track.state.external_target_id == 970U &&
        track.state.association_key == baseline_key) {
      retained_known_track = true;
      break;
    }
  }
  EXPECT_TRUE(retained_known_track);
}

TEST(PublicApiConvenienceTest,
     RadarSessionKeepsRuntimePatchStagedWhenSignalPipelineRejectsConfigUpdate) {
  RecordingRadarContext radar_context;
  RecordingSignalPipeline signal_pipeline;
  RecordingEnvironmentService environment_service;
  NoopDecisionEngine decision_engine;
  extension::RadarController controller(radar_context, signal_pipeline, decision_engine,
                                        environment_service);
  session::RadarSession session = session::RadarSessionFactory::CreateWithController(
      MakeConvenienceSessionConfig(), controller);

  const session::RadarCycleResult baseline = session.StepWithResult(MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(980U, 160.0f, 0.0f, 10.0f, 60.0f, 0.0f, 0.0f, 1.0f),
      }));
  ASSERT_TRUE(baseline.executed_this_cycle);
  const std::size_t committed_update_config_count = signal_pipeline.update_config_count();

  session.ApplyRuntimeConfig(config::RadarRuntimeConfigBuilder()
                                 .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
                                 .Build());

  signal_pipeline.SetShouldAcceptUpdates(false);
  const session::RadarCycleResult rejected = session.StepWithResult(MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(981U, 162.0f, 0.0f, 10.0f, 60.0f, 0.0f, 0.0f, 1.0f),
      }));
  EXPECT_FALSE(rejected.executed_this_cycle);
  EXPECT_EQ(rejected.signal_cycle_abort_reason,
            extension::SignalCycleAbortReason::kRuntimePreparationFailed);
  EXPECT_TRUE(rejected.reused_previous_track_output);
  EXPECT_EQ(signal_pipeline.config().orientation.work_sub_mode,
            model::RadarWorkSubMode::kTws);
  EXPECT_EQ(signal_pipeline.update_config_count(), committed_update_config_count);

  signal_pipeline.SetShouldAcceptUpdates(true);
  const session::RadarCycleResult committed = session.StepWithResult(MakeCycleInput(
      model::TargetFeatureList{
          model::MakeAirTarget(982U, 164.0f, 0.0f, 10.0f, 60.0f, 0.0f, 0.0f, 1.0f),
      }));
  EXPECT_TRUE(committed.executed_this_cycle);
  EXPECT_EQ(signal_pipeline.config().orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultSurfacesValidationErrors) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());

  session::RadarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  model::TargetFeature invalid_target;
  invalid_target.external_target_id = 123U;
  invalid_target.range_m = 0.0f;
  input.target_features.push_back(invalid_target);

  const session::RadarCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_track_output);
  EXPECT_TRUE(result.submitted_commands.empty());
  EXPECT_FALSE(result.has_control_profile);
  EXPECT_EQ(result.association_quality_metrics.detection_count, 0U);
  EXPECT_TRUE(ContainsIssueCode(result.validation_issues,
                                session::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsIssueCode(result.validation_issues,
                                session::ValidationCode::kMissingRangeAndCartesianPosition));
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultMatchesManualChainUnderJammedScene) {
  const session::RadarSessionConfig config = config::presets::MakeDetectionMissionRadarSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  session::MutableRadarContext manual_context;
  config::PipelineConfig pipeline_config = session::BuildLegacyPipelineConfig(config);
  signal::pipeline::SignalPipeline signal_pipeline(pipeline_config);
  environment::EnvironmentService environment_service(
      environment::BuildModelConfigFromScenario(config.environment.scenario_config));
  environment_service.SetJammingSensitivityProfile(
      config.environment.jamming_sensitivity_profile);
  extension::RadarController controller(manual_context, signal_pipeline, environment_service);

  const session::RadarCycleInput input = MakeCycleInput(model::TargetFeatureList{
      model::MakeAirTarget(960U, 180.0f, -4.0f, 16.0f, 60.0f, 0.0f, 0.0f, 1.0f),
  });
  environment::JammerEmitterState noise_emitter_5;
  noise_emitter_5.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_5.power_db = 12.0f;
  noise_emitter_5.js_db = 8.0f;
  noise_emitter_5.has_direction_deg = true;
  noise_emitter_5.azimuth_deg = 20.0f;
  noise_emitter_5.elevation_deg = 1.0f;
  noise_emitter_5.angular_span_deg = 10.0f;

  const environment::EnvironmentSceneState scene =
      environment::EnvironmentSceneBuilder().AddNoiseJammer(noise_emitter_5).Build();

  const session::RadarCycleResult session_result = session.StepWithResult(input, scene);

  environment_service.UpdateSceneState(scene);
  manual_context.BeginCycle(input);
  controller.RunOnce();

  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  EXPECT_EQ(session_result.track_output_frame.published_track_count,
            controller.GetLatestTrackOutputFrame().published_track_count);
  ExpectEquivalentCommands(manual_context.GetSubmittedCommands(),
                           session_result.submitted_commands);
  ASSERT_TRUE(manual_context.HasLatestControlProfile());
  ASSERT_TRUE(session_result.has_control_profile);
  ExpectEquivalentProfiles(manual_context.GetLatestControlProfile(),
                           session_result.control_profile);
  EXPECT_EQ(session_result.association_quality_metrics.detection_count,
            signal_pipeline.GetLastAssociationQualityMetrics().detection_count);
}

}  // namespace tests
}  // namespace airborne_radar
