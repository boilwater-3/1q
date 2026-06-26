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
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "airborne_radar/runtime/RadarController.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/session/RadarSceneTargetUtils.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/coordinate/position_transform.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/session/MutableRadarContext.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "common/geometry/GeometryTransform.h"

namespace airborne_radar {

namespace model {
namespace {

using SceneTarget = session::RadarSceneTarget;
using SceneTargetList = session::RadarSceneTargetList;

SceneTarget CloneSceneTarget(const session::RadarSceneTarget& target) {
  SceneTarget out;
  out.external_target_id = target.external_target_id;
  out.velocity_x = target.velocity_x;
  out.velocity_y = target.velocity_y;
  out.velocity_z = target.velocity_z;
  out.rcs = target.rcs;
  out.range_m = target.range_m;

  out.position_x = target.position_x;
  out.position_y = target.position_y;
  out.position_z = target.position_z;
  out.target_swerling_type = target.target_swerling_type;
  return out;
}

float ComputeNorm3(float x, float y, float z) { return std::sqrt(x * x + y * y + z * z); }

}  // namespace

SceneTarget MakeTargetFromCartesian(std::uint64_t external_target_id, float position_x,
                                    float position_y, float position_z, float velocity_x,
                                    float velocity_y, float velocity_z, float rcs,
                                    int swerling_type = 0) {
  return CloneSceneTarget(session::MakeSceneTarget(external_target_id, position_x, position_y,
                                                   position_z, velocity_x, velocity_y, velocity_z,
                                                   rcs, swerling_type));
}

SceneTarget MakeGroundTarget(std::uint64_t external_target_id, float position_x, float position_y,
                             float rcs = 1.0f, float velocity_x = 0.0f, float velocity_y = 0.0f,
                             int swerling_type = 0) {
  return CloneSceneTarget(session::MakeGroundSceneTarget(
      external_target_id, position_x, position_y, rcs, velocity_x, velocity_y, swerling_type));
}

SceneTarget MakeAirTarget(std::uint64_t external_target_id, float position_x, float position_y,
                          float position_z, float velocity_x, float velocity_y, float velocity_z,
                          float rcs = 1.0f, int swerling_type = 0) {
  return CloneSceneTarget(session::MakeAirSceneTarget(external_target_id, position_x, position_y,
                                                      position_z, velocity_x, velocity_y,
                                                      velocity_z, rcs, swerling_type));
}

void NormalizeTargetGeometry(SceneTarget* target) {
  if (target == nullptr) {
    return;
  }
  if (target->range_m > 0.0f) {
    return;
  }
  target->range_m = ComputeNorm3(target->position_x, target->position_y, target->position_z);
}

void NormalizeTargetGeometry(SceneTargetList* targets) {
  if (targets == nullptr) {
    return;
  }
  for (std::size_t i = 0; i < targets->size(); ++i) {
    NormalizeTargetGeometry(&(*targets)[i]);
  }
}

}  // namespace model

namespace tests {

namespace {

float SpeedOf(const session::RadarSceneTarget& target) {
  return std::sqrt(target.velocity_x * target.velocity_x + target.velocity_y * target.velocity_y +
                   target.velocity_z * target.velocity_z);
}

session::RadarSceneTarget ToSceneTarget(const session::RadarSceneTarget& target) {
  session::RadarSceneTarget out;
  out.external_target_id = target.external_target_id;
  out.velocity_x = target.velocity_x;
  out.velocity_y = target.velocity_y;
  out.velocity_z = target.velocity_z;
  out.rcs = target.rcs;
  out.range_m = target.range_m;
  out.position_x = target.position_x;
  out.position_y = target.position_y;
  out.position_z = target.position_z;
  out.target_swerling_type = target.target_swerling_type;
  return out;
}

session::RadarSceneTargetList ToSceneTargets(const session::RadarSceneTargetList& targets) {
  session::RadarSceneTargetList out;
  out.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    out.push_back(ToSceneTarget(targets[i]));
  }
  return out;
}

config::RadarSessionConfig MakeConvenienceSessionConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

session::RadarCycleInput MakeCycleInput(session::RadarSceneTargetList targets, float dt_sec = 1.0f,
                                        std::uint32_t cycle_index = 0U) {
  static std::uint32_t next_cycle_index = 1U;
  session::RadarCycleInput input;
  input.cycle_index = cycle_index == 0U ? next_cycle_index++ : cycle_index;
  input.scene = ToSceneTargets(targets);
  input.dt_sec = dt_sec;
  input.platform_pose.attitude_deg.yaw_deg = 5.0f;
  input.platform_pose.attitude_deg.pitch_deg = -1.0f;
  input.platform_pose.attitude_deg.roll_deg = 2.0f;
  return input;
}

void ApplySceneStateToCycleInput(const environment::EnvironmentSceneState& scene_state,
                                 session::RadarCycleInput* input) {
  if (input == nullptr) {
    return;
  }
  input->environment.atmospheric_observation = scene_state.atmospheric_physics;
  input->environment.atmospheric_context = scene_state.atmospheric_context;
  input->environment.surface_observation = scene_state.vegetation_scatter_physics;
  input->environment.jammer_sources = scene_state.jammer_emitters;
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

model::TrackStateSnapshot MakeTrackStateSnapshot(float velocity_x, float velocity_y,
                                                 float velocity_z, float rcs, bool jamming_detected,
                                                 std::uint64_t external_target_id,
                                                 std::uint64_t association_key) {
  model::TrackStateSnapshot track;
  track.velocity_x = velocity_x;
  track.velocity_y = velocity_y;
  track.velocity_z = velocity_z;
  track.speed =
      std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y + velocity_z * velocity_z);
  track.rcs = rcs;
  track.jamming_detected = jamming_detected;
  track.external_target_id = external_target_id;
  track.association_key = association_key;
  return track;
}

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

  void SetJammingSensitivityProfile(environment::JammingSensitivityProfile profile) override {
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
      const session::RadarSceneTargetList& input_state,
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

  bool UpdateConfig(const config::RadarSessionConfig& config) override {
    ++update_config_count_;
    if (!should_accept_updates_) {
      return false;
    }
    config_ = config;
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
  const config::RadarSessionConfig& config() const { return config_; }

 private:
  struct RuntimeState {
    model::PlatformAttitudeDeg platform_attitude_deg{};
    extension::control::RadarControlProfile control_profile{};
    config::RadarSessionConfig config{};
    bool should_execute{true};
    bool should_accept_updates{true};
    std::size_t run_cycle_count{0U};
    std::size_t update_config_count{0U};
  };

  model::PlatformAttitudeDeg platform_attitude_deg_{};
  extension::control::RadarControlProfile control_profile_{};
  config::RadarSessionConfig config_{};
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
      session::RadarSceneTargetList{
          model::MakeAirTarget(101U, 120.0f, -5.0f, 18.0f, 62.0f, 0.0f, 0.0f, 1.0f),
      },
      0.5f);

  context.BeginCycle(input);
  ASSERT_EQ(context.GetSceneTargets().size(), 1U);
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

TEST(PublicApiConvenienceTest, SceneTargetUtilsBuildCartesianGroundAndAirTargets) {
  const session::RadarSceneTarget cartesian_target =
      model::MakeTargetFromCartesian(201U, 3.0f, 4.0f, 12.0f, 10.0f, -2.0f, 1.0f, 0.9f, 2);
  EXPECT_EQ(cartesian_target.external_target_id, 201U);
  EXPECT_NEAR(cartesian_target.range_m, 13.0f, 1e-5f);
  EXPECT_NEAR(SpeedOf(cartesian_target), std::sqrt(105.0f), 1e-5f);
  EXPECT_EQ(cartesian_target.target_swerling_type, 2);

  const session::RadarSceneTarget ground_target =
      model::MakeGroundTarget(202U, 20.0f, -15.0f, 0.8f);
  EXPECT_NEAR(ground_target.position_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.velocity_z, 0.0f, 1e-5f);
  EXPECT_NEAR(ground_target.range_m, 25.0f, 1e-5f);

  const session::RadarSceneTarget air_target =
      model::MakeAirTarget(203U, 30.0f, 40.0f, 50.0f, 90.0f, -3.0f, 4.0f, 1.2f);
  EXPECT_NEAR(air_target.range_m, std::sqrt(5000.0f), 1e-5f);
  EXPECT_NEAR(SpeedOf(air_target), std::sqrt(8125.0f), 1e-5f);
}

TEST(PublicApiConvenienceTest, SceneTargetUtilsNormalizeGeometryFromCartesianPosition) {
  session::RadarSceneTarget normalized =
      model::MakeAirTarget(301U, 6.0f, 8.0f, 0.0f, 50.0f, 0.0f, 0.0f, 1.0f);
  normalized.range_m = 0.0f;
  model::NormalizeTargetGeometry(&normalized);
  EXPECT_NEAR(normalized.range_m, 10.0f, 1e-5f);

  session::RadarSceneTarget unresolved;
  unresolved.external_target_id = 302U;
  unresolved.range_m = 0.0f;
  model::NormalizeTargetGeometry(&unresolved);
  EXPECT_NEAR(unresolved.range_m, 0.0f, 1e-5f);

  session::RadarSceneTargetList targets;
  targets.push_back(normalized);
  targets.back().range_m = -1.0f;
  targets.push_back(unresolved);
  model::NormalizeTargetGeometry(&targets);
  EXPECT_NEAR(targets[0].range_m, 10.0f, 1e-5f);
  EXPECT_NEAR(targets[1].range_m, 0.0f, 1e-5f);
}

TEST(PublicApiConvenienceTest, SceneTargetUtilsComposeRadarAttitudeUsesRotationComposition) {
  oneq::coordinate::EulerAnglesDeg platform_attitude;
  platform_attitude.yaw_deg = 40.0f;
  platform_attitude.pitch_deg = 12.0f;
  platform_attitude.roll_deg = -18.0f;

  oneq::coordinate::EulerAnglesDeg mount_angles;
  mount_angles.yaw_deg = 3.0f;
  mount_angles.pitch_deg = -2.5f;
  mount_angles.roll_deg = 1.5f;

  const oneq::coordinate::EulerAnglesDeg composed =
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

TEST(PublicApiConvenienceTest, SceneTargetUtilsBuildsTargetFromExternalCoordinates) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 0.0;
  radar_lla.longitude_deg = 0.0;
  radar_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  session::RadarExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = radar_ecef;

  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::PoseState platform_pose;
  ASSERT_TRUE(
      session::TryMakeRadarPoseFromExternalKinematics(pose_input, &reference, &platform_pose));

  session::RadarExternalTargetInput input;
  input.target_id = 403U;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  input.kinematics.position_ecef_m = target_ecef;
  input.kinematics.velocity_mps.x_mps = 80.0f;
  input.kinematics.velocity_mps.y_mps = -30.0f;
  input.kinematics.velocity_mps.z_mps = 10.0f;
  input.rcs = 2.0f;
  input.swerling_type = 0;

  session::RadarSceneTarget target_from_external;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(
      input, reference, platform_pose.velocity_mps, &target_from_external));
  EXPECT_EQ(target_from_external.external_target_id, 403U);
  EXPECT_GT(target_from_external.position_x, 100.0f);
  EXPECT_NEAR(target_from_external.position_y, 0.0f, 2.0f);
  EXPECT_NEAR(target_from_external.position_z, 0.0f, 2.0f);
  EXPECT_GT(target_from_external.range_m, 100.0f);
}

TEST(PublicApiConvenienceTest, SceneTargetUtilsBuildsTargetFromExternalKinematicsFrames) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 31.2304;
  radar_lla.longitude_deg = 121.4737;
  radar_lla.altitude_m = 200.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  oneq::coordinate::LlaPositionDegM target_lla = radar_lla;
  target_lla.longitude_deg += 0.001;
  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  session::RadarExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = radar_ecef;
  pose_input.platform_velocity_mps.x_mps = 120.0f;
  pose_input.platform_velocity_mps.y_mps = -70.0f;
  pose_input.platform_velocity_mps.z_mps = 30.0f;
  pose_input.platform_attitude_deg.yaw_deg = 5.0f;
  pose_input.radar_mount_angles_deg.pitch_deg = 2.0f;

  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::PoseState platform_pose;
  ASSERT_TRUE(
      session::TryMakeRadarPoseFromExternalKinematics(pose_input, &reference, &platform_pose));

  session::RadarExternalTargetInput input;
  input.target_id = 501U;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  input.kinematics.position_ecef_m = target_ecef;
  input.rcs = 1.2f;

  // ECEF 速度 + 雷达自身 ECEF 速度补偿后应接近零相对速度。
  input.kinematics.velocity_mps.x_mps = 120.0f;
  input.kinematics.velocity_mps.y_mps = -70.0f;
  input.kinematics.velocity_mps.z_mps = 30.0f;
  session::RadarSceneTarget ecef_target;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(
      input, reference, platform_pose.velocity_mps, &ecef_target));
  EXPECT_NEAR(ecef_target.velocity_x, 0.0f, 1.0e-4f);
  EXPECT_NEAR(ecef_target.velocity_y, 0.0f, 1.0e-4f);
  EXPECT_NEAR(ecef_target.velocity_z, 0.0f, 1.0e-4f);
}

TEST(PublicApiConvenienceTest, TwoStepExternalKinematicsProducesStableTarget) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 31.2304;
  radar_lla.longitude_deg = 121.4737;
  radar_lla.altitude_m = 200.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  oneq::coordinate::LlaPositionDegM target_lla = radar_lla;
  target_lla.longitude_deg += 0.001;
  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  session::RadarExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = radar_ecef;
  pose_input.platform_velocity_mps.x_mps = 120.0f;
  pose_input.platform_velocity_mps.y_mps = -70.0f;
  pose_input.platform_velocity_mps.z_mps = 30.0f;
  pose_input.platform_attitude_deg.yaw_deg = 5.0f;
  pose_input.radar_mount_angles_deg.pitch_deg = 2.0f;

  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::PoseState platform_pose;
  ASSERT_TRUE(
      session::TryMakeRadarPoseFromExternalKinematics(pose_input, &reference, &platform_pose));

  session::RadarExternalTargetInput target_input;
  target_input.target_id = 600U;
  target_input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target_input.kinematics.position_ecef_m = target_ecef;
  target_input.kinematics.velocity_mps.x_mps = 120.0f;
  target_input.kinematics.velocity_mps.y_mps = -70.0f;
  target_input.kinematics.velocity_mps.z_mps = 30.0f;
  target_input.rcs = 1.2f;
  target_input.swerling_type = 0;

  session::RadarSceneTarget target_1;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(target_input, reference,
                                                           platform_pose.velocity_mps, &target_1));

  session::RadarSceneTarget target_2;
  ASSERT_TRUE(session::TryMakeTargetFromExternalKinematics(target_input, reference,
                                                           platform_pose.velocity_mps, &target_2));

  EXPECT_EQ(target_1.external_target_id, target_2.external_target_id);
  EXPECT_NEAR(target_1.position_x, target_2.position_x, 1.0e-6f);
  EXPECT_NEAR(target_1.position_y, target_2.position_y, 1.0e-6f);
  EXPECT_NEAR(target_1.position_z, target_2.position_z, 1.0e-6f);
  EXPECT_NEAR(target_1.range_m, target_2.range_m, 1.0e-6f);
}

TEST(PublicApiConvenienceTest, TwoStepApiPlatformPoseFeedsDirectlyIntoCycleInput) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 31.0;
  radar_lla.longitude_deg = 121.0;
  radar_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  session::RadarExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = radar_ecef;
  pose_input.platform_attitude_deg.yaw_deg = 10.0f;
  pose_input.platform_attitude_deg.pitch_deg = -3.0f;
  pose_input.platform_attitude_deg.roll_deg = 1.0f;

  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::PoseState platform_pose;
  ASSERT_TRUE(
      session::TryMakeRadarPoseFromExternalKinematics(pose_input, &reference, &platform_pose));

  // platform_pose 可直接赋给 RadarCycleInput::platform_pose，无需手动复制姿态
  session::RadarCycleInput cycle_input;
  cycle_input.platform_pose = platform_pose;
  EXPECT_NEAR(cycle_input.platform_pose.attitude_deg.yaw_deg, 10.0f, 1.0e-5f);
  EXPECT_NEAR(cycle_input.platform_pose.attitude_deg.pitch_deg, -3.0f, 1.0e-5f);
  EXPECT_NEAR(cycle_input.platform_pose.attitude_deg.roll_deg, 1.0f, 1.0e-5f);
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
  session::TrackOutputFrame frame;
  frame.tracks.push_back(MakeTrackStateSnapshot(10.0f, 0.0f, 0.0f, 1.0f, false, 401U, 1U));
  frame.tracks.push_back(MakeTrackStateSnapshot(20.0f, 0.0f, 0.0f, 1.0f, true, 402U, 2U));
  frame.tracks.push_back(MakeTrackStateSnapshot(21.0f, 0.0f, 0.0f, 1.1f, false, 402U, 3U));
  frame.tracks.push_back(MakeTrackStateSnapshot(8.0f, 0.0f, 0.0f, 0.7f, true, 0U, 4U));

  const auto track_map = session::BuildTrackMapByExternalTargetId(frame);
  EXPECT_EQ(track_map.size(), 2U);
  ASSERT_NE(track_map.count(401U), 0U);
  ASSERT_NE(track_map.count(402U), 0U);
  EXPECT_EQ(track_map.at(402U).association_key, 3U);

  const model::TrackStateSnapshotList duplicate_tracks =
      session::CollectTracksByExternalTargetId(frame, 402U);
  EXPECT_EQ(duplicate_tracks.size(), 2U);
  EXPECT_TRUE(session::ContainsExternalTargetId(frame, 401U));
  EXPECT_TRUE(session::ContainsExternalTargetId(frame, 0U));
  EXPECT_FALSE(session::ContainsExternalTargetId(frame, 999U));
  EXPECT_EQ(session::CountJammingTracks(frame), 2U);
  EXPECT_EQ(session::CountTracksByStatus(frame, model::TrackStatus::kTentative), 4U);
}

TEST(PublicApiConvenienceTest, TrackOutputQueriesSupportAssociationKeyStatusAndJammingCollections) {
  session::TrackOutputFrame frame;
  model::TrackStateSnapshot confirmed =
      MakeTrackStateSnapshot(20.0f, 0.0f, 0.0f, 1.0f, false, 410U, 11U);
  confirmed.status = model::TrackStatus::kConfirmed;
  model::TrackStateSnapshot lost = MakeTrackStateSnapshot(5.0f, 0.0f, 0.0f, 0.8f, false, 411U, 12U);
  lost.status = model::TrackStatus::kLost;
  model::TrackStateSnapshot jammed =
      MakeTrackStateSnapshot(18.0f, 0.0f, 0.0f, 1.2f, true, 412U, 13U);
  jammed.status = model::TrackStatus::kConfirmed;
  frame.tracks.push_back(confirmed);
  frame.tracks.push_back(lost);
  frame.tracks.push_back(jammed);

  const auto track_map = session::BuildTrackMapByAssociationKey(frame);
  ASSERT_EQ(track_map.size(), 3U);
  EXPECT_EQ(track_map.at(13U).external_target_id, 412U);
  EXPECT_EQ(session::CollectConfirmedTracks(frame).size(), 2U);
  EXPECT_EQ(session::CollectLostTracks(frame).size(), 1U);
  EXPECT_EQ(session::CollectJammingTracks(frame).size(), 1U);
  EXPECT_EQ(session::CountTracksByStatus(frame, model::TrackStatus::kConfirmed), 2U);
}

TEST(PublicApiConvenienceTest, RadarInputValidationReportsWarningsAndErrorsForCommonBoundaryCases) {
  session::RadarCycleInput input = MakeCycleInput(
      session::RadarSceneTargetList{
          model::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
          model::MakeAirTarget(800U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f, 1.0f),
          model::MakeAirTarget(800U, 155.0f, 2.0f, 12.0f, 63.0f, -0.2f, 0.0f, -1.1f),
      },
      0.0f);
  input.scene[2].position_x = std::numeric_limits<float>::infinity();
  input.scene[2].range_m = 0.0f;

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
  session::RadarSceneTarget invalid_target;
  invalid_target.external_target_id = 900U;
  invalid_target.range_m = 0.0f;
  input.scene.push_back(invalid_target);

  const std::vector<session::ValidationIssue> issues = session::ValidateRadarCycleInput(input);
  EXPECT_TRUE(ContainsIssueCode(issues, session::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(
      ContainsIssueCode(issues, session::ValidationCode::kMissingRangeAndCartesianPosition));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(PublicApiConvenienceTest, BuilderProvidesExpectedDetectionFocusedDefaults) {
  const config::RadarSessionConfig detection_config = MakeConvenienceSessionConfig();
  EXPECT_EQ(detection_config.policy.lifecycle.confirm_hits, 1U);
  EXPECT_EQ(detection_config.policy.lifecycle.max_miss_before_lost, 1U);
  EXPECT_EQ(detection_config.hardware.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(detection_config.hardware.detection.detection_policy.min_snr_db, -12.0f);

  session::RadarSession session = session::RadarSessionFactory::Create(detection_config);
  const session::TrackOutputFrame frame = session.Step(MakeCycleInput(session::RadarSceneTargetList{
      model::MakeGroundTarget(901U, 20.0f, 5.0f, 0.8f),
  }));
  EXPECT_EQ(session::CountTracksByStatus(frame, model::TrackStatus::kConfirmed), 1U);
}

TEST(PublicApiConvenienceTest, DefaultSessionConfigUsesLifecycleManagedTracks) {
  session::RadarSession session = session::RadarSessionFactory::Create();
  const session::TrackOutputFrame frame = session.Step(MakeCycleInput(session::RadarSceneTargetList{
      model::MakeGroundTarget(902U, 15.0f, -3.0f, 0.8f),
  }));

  ASSERT_EQ(frame.tracks.size(), 1U);
  EXPECT_EQ(frame.tracks.size(), 1U);
  EXPECT_EQ(session::CountTracksByStatus(frame, model::TrackStatus::kConfirmed), 0U);
  EXPECT_EQ(frame.tracks[0].status, model::TrackStatus::kTentative);
}

TEST(PublicApiConvenienceTest, RadarSessionStepProducesReadableOutputWithoutInterference) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());
  const session::RadarCycleInput input = MakeCycleInput(session::RadarSceneTargetList{
      model::MakeGroundTarget(501U, 25.0f, -5.0f, 0.7f),
      model::MakeAirTarget(502U, 150.0f, 6.0f, 18.0f, 72.0f, 1.0f, 0.0f, 1.0f),
  });

  const session::TrackOutputFrame frame = session.Step(input);
  EXPECT_EQ(frame.tracks.size(), 2U);
  EXPECT_EQ(session::CountTracksByStatus(frame, model::TrackStatus::kConfirmed), 2U);
  EXPECT_TRUE(session::ContainsExternalTargetId(frame, 501U));
  EXPECT_TRUE(session::ContainsExternalTargetId(frame, 502U));
  EXPECT_EQ(session::CountJammingTracks(frame), 0U);
  EXPECT_TRUE(session.GetSubmittedCommands().empty());
  EXPECT_TRUE(session.HasLatestControlProfile());
}

TEST(PublicApiConvenienceTest, RadarSessionSceneAwareStepMatchesManualControllerChain) {
  const config::RadarSessionConfig config = MakeConvenienceSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  session::MutableRadarContext manual_context;
  signal::pipeline::SignalPipeline signal_pipeline(config);
  environment::EnvironmentService environment_service(
      environment::BuildModelConfigFromScenario(config.environment.scenario_config));
  environment_service.SetJammingSensitivityProfile(config.environment.jamming_sensitivity_profile);
  extension::RadarController controller(manual_context, signal_pipeline, environment_service);

  const session::RadarCycleInput input = MakeCycleInput(session::RadarSceneTargetList{
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

  session::RadarCycleInput session_input = input;
  ApplySceneStateToCycleInput(scene, &session_input);
  const session::TrackOutputFrame session_frame = session.Step(session_input);

  environment_service.UpdateSceneState(scene);
  manual_context.BeginCycle(input);
  controller.RunOnce();
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame manual_frame = controller.GetLatestTrackOutputFrame();

  EXPECT_EQ(session_frame.tracks.size(), manual_frame.tracks.size());
  EXPECT_EQ(session::CountTracksByStatus(session_frame, model::TrackStatus::kConfirmed),
            session::CountTracksByStatus(manual_frame, model::TrackStatus::kConfirmed));
  EXPECT_EQ(session::CountJammingTracks(session_frame), session::CountJammingTracks(manual_frame));

  const auto session_track_map = session::BuildTrackMapByExternalTargetId(session_frame);
  const auto manual_track_map = session::BuildTrackMapByExternalTargetId(manual_frame);
  ASSERT_EQ(session_track_map.size(), manual_track_map.size());
  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    const std::uint64_t target_id = input.scene[i].external_target_id;
    ASSERT_NE(session_track_map.count(target_id), 0U);
    ASSERT_NE(manual_track_map.count(target_id), 0U);
    EXPECT_EQ(session_track_map.at(target_id).association_key,
              manual_track_map.at(target_id).association_key);
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
      session::RadarSceneTargetList{
          model::MakeAirTarget(0U, 120.0f, 0.0f, 10.0f, 55.0f, 0.0f, 0.0f, 1.0f),
          model::MakeAirTarget(701U, 150.0f, -2.0f, 12.0f, 62.0f, 0.2f, 0.0f, 1.0f),
          model::MakeGroundTarget(702U, 40.0f, -6.0f, 0.8f),
      },
      1.0f);
  const session::RadarCycleResult result_1 = session.StepWithResult(cycle_1);
  const session::TrackOutputFrame& frame_1 = result_1.track_output_frame;
  EXPECT_GT(frame_1.tracks.size(), 0U);
  EXPECT_TRUE(session::ContainsExternalTargetId(frame_1, 0U));
  EXPECT_EQ(session::CollectTracksByExternalTargetId(frame_1, 701U).size(), 1U);
  EXPECT_FALSE(result_1.has_validation_error);
  EXPECT_TRUE(result_1.executed_this_cycle);
  EXPECT_FALSE(result_1.reused_previous_output);

  session::RadarCycleInput duplicate_cycle = MakeCycleInput(
      session::RadarSceneTargetList{
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
  EXPECT_TRUE(duplicate_result.reused_previous_output);
  EXPECT_EQ(duplicate_result.track_output_frame.cycle_index, frame_1.cycle_index);
  EXPECT_EQ(duplicate_result.track_output_frame.batch_id, frame_1.batch_id);
  EXPECT_EQ(duplicate_result.track_output_frame.tracks.size(), frame_1.tracks.size());

  session::RadarCycleInput cycle_2 = cycle_1;
  cycle_2.dt_sec = -1.0f;
  for (std::size_t i = 0; i < cycle_2.scene.size(); ++i) {
    cycle_2.scene[i].position_x += 5.0f;
  }
  environment::JammerEmitterState noise_emitter_3;
  noise_emitter_3.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_emitter_3.power_db = 12.0f;
  noise_emitter_3.js_db = 8.0f;
  noise_emitter_3.has_direction_deg = true;
  noise_emitter_3.azimuth_deg = 20.0f;
  noise_emitter_3.elevation_deg = 1.0f;
  noise_emitter_3.angular_span_deg = 10.0f;

  const environment::EnvironmentSceneState noise_scene =
      environment::EnvironmentSceneBuilder().AddNoiseJammer(noise_emitter_3).Build();
  ApplySceneStateToCycleInput(noise_scene, &cycle_2);
  const session::RadarCycleResult result_2 = session.StepWithResult(cycle_2);
  const session::TrackOutputFrame& frame_2 = result_2.track_output_frame;

  EXPECT_TRUE(result_2.has_validation_error);
  EXPECT_FALSE(result_2.executed_this_cycle);
  EXPECT_TRUE(result_2.reused_previous_output);
  EXPECT_TRUE(ContainsIssueCode(result_2.validation_issues,
                                session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_EQ(frame_2.cycle_index, frame_1.cycle_index);
  EXPECT_EQ(frame_2.batch_id, frame_1.batch_id);
  EXPECT_EQ(frame_2.tracks.size(), frame_1.tracks.size());
  EXPECT_EQ(session::CountJammingTracks(frame_2), session::CountJammingTracks(frame_1));
  EXPECT_TRUE(result_2.submitted_commands.empty());
  EXPECT_FALSE(result_2.has_control_profile);
  EXPECT_EQ(result_2.association_quality_metrics.detection_count, 0U);
  EXPECT_TRUE(session.HasLatestControlProfile());

  session::RadarCycleInput cycle_3 = cycle_2;
  cycle_3.dt_sec = 1.0f;
  cycle_3.scene[1].external_target_id = 703U;
  cycle_3.scene[2].external_target_id = 704U;
  const environment::EnvironmentSceneState clear_scene =
      environment::EnvironmentSceneBuilder().Build();
  ApplySceneStateToCycleInput(clear_scene, &cycle_3);
  const session::TrackOutputFrame frame_3 = session.Step(cycle_3);
  EXPECT_GT(frame_3.tracks.size(), 0U);
  EXPECT_EQ(session::CountJammingTracks(frame_3), 0U);
  EXPECT_TRUE(session::ContainsExternalTargetId(frame_3, 703U));
  EXPECT_TRUE(session::ContainsExternalTargetId(frame_3, 704U));
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultSurfacesValidationErrors) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(MakeConvenienceSessionConfig());

  session::RadarCycleInput input;
  input.cycle_index = 88U;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  session::RadarSceneTarget invalid_target;
  invalid_target.external_target_id = 123U;
  invalid_target.range_m = 0.0f;
  input.scene.push_back(invalid_target);

  const session::RadarCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_EQ(result.input_cycle_index, 88U);
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(result.submitted_commands.empty());
  EXPECT_FALSE(result.has_control_profile);
  EXPECT_EQ(result.association_quality_metrics.detection_count, 0U);
  EXPECT_TRUE(ContainsIssueCode(result.validation_issues,
                                session::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsIssueCode(result.validation_issues,
                                session::ValidationCode::kMissingRangeAndCartesianPosition));
}

TEST(PublicApiConvenienceTest, RadarSessionStepWithResultMatchesManualChainUnderJammedScene) {
  const config::RadarSessionConfig config = MakeConvenienceSessionConfig();
  session::RadarSession session = session::RadarSessionFactory::Create(config);

  session::MutableRadarContext manual_context;
  signal::pipeline::SignalPipeline signal_pipeline(config);
  environment::EnvironmentService environment_service(
      environment::BuildModelConfigFromScenario(config.environment.scenario_config));
  environment_service.SetJammingSensitivityProfile(config.environment.jamming_sensitivity_profile);
  extension::RadarController controller(manual_context, signal_pipeline, environment_service);

  const session::RadarCycleInput input = MakeCycleInput(session::RadarSceneTargetList{
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

  session::RadarCycleInput session_input = input;
  ApplySceneStateToCycleInput(scene, &session_input);
  const session::RadarCycleResult session_result = session.StepWithResult(session_input);

  environment_service.UpdateSceneState(scene);
  manual_context.BeginCycle(input);
  controller.RunOnce();

  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  EXPECT_EQ(session_result.track_output_frame.tracks.size(),
            controller.GetLatestTrackOutputFrame().tracks.size());
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
