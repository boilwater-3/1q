// Copyright 2026. All Rights Reserved.
//
// @file ar_session_test.cpp
// @brief 验证机载雷达两阶段联调场景下的控制器驱动集成链路。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"

namespace airborne_radar {
namespace tests {

namespace {

class ScenarioRadarContext : public extension::IRadarContext {
 public:
  explicit ScenarioRadarContext(model::TargetFeatureList target_features = {})
      : target_features_(std::move(target_features)) {}

  void BeginCycle(const session::RadarCycleInput& input) override {
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

  void SubmitControlCommand(extension::control::RadarCommand command) override {
    submitted_commands_.push_back(command);
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

  void SetTargetFeatures(model::TargetFeatureList target_features) {
    target_features_ = std::move(target_features);
  }

  void SetPlatformAttitude(const model::PlatformAttitudeDeg& platform_attitude_deg) {
    platform_attitude_deg_ = platform_attitude_deg;
  }

  void SetCycleDeltaTimeSec(float cycle_dt_sec) { cycle_dt_sec_ = cycle_dt_sec; }

  const std::vector<extension::control::RadarCommand>& SubmittedCommands() const {
    return submitted_commands_;
  }

  const extension::control::RadarControlProfile& LatestControlProfile() const {
    return latest_control_profile_;
  }

 private:
  model::TargetFeatureList target_features_;
  model::PlatformAttitudeDeg platform_attitude_deg_{};
  float cycle_dt_sec_{1.0f};
  std::vector<extension::control::RadarCommand> submitted_commands_;
  extension::control::RadarControlProfile latest_control_profile_{};
  bool has_latest_control_profile_{false};
};

struct CycleStats {
  std::size_t published_track_count{0U};
  std::size_t confirmed_track_count{0U};
  std::size_t jamming_track_count{0U};
  std::size_t command_delta_count{0U};
  float association_stress{0.0f};
  float match_rate{0.0f};
};

struct SceneScriptStep {
  environment::EnvironmentSceneState scene_state{};
  bool expect_jamming{false};

  SceneScriptStep(const environment::EnvironmentSceneState& scene_state_in, bool expect_jamming_in)
      : scene_state(scene_state_in), expect_jamming(expect_jamming_in) {}
};

config::PipelineConfig MakeJointIntegrationPipelineConfig() {
  const session::RadarSessionConfig session_config =
      config::RadarSessionConfigBuilder()
          .Detection()
          .WithDetectionIntentProfile(config::semantic::DetectionIntentProfile::kDetectionPriority)
          .End()
          .Tracking()
          .EnableTrackingFilter(true)
          .WithTrackingPolicyProfile(config::semantic::TrackingPolicyProfile::kFastAssociation)
          .End()
          .Lifecycle()
          .WithLifecyclePolicyProfile(config::semantic::LifecyclePolicyProfile::kFastConfirm)
          .End()
          .Build();
  config::PipelineConfig pipeline_config;
  pipeline_config.expert.detection = session_config.hardware.detection;
  pipeline_config.expert.beam_control = session_config.policy.beam_control;
  pipeline_config.expert.association = session_config.policy.association;
  pipeline_config.expert.tracking = session_config.policy.tracking;
  pipeline_config.expert.lifecycle = session_config.policy.lifecycle;
  pipeline_config.expert.imm = session_config.policy.imm;
  pipeline_config.orientation = session_config.mission.orientation;
  return pipeline_config;
}

config::PipelineConfig MakeJointIntegrationPhysicsPipelineConfig(float pulse_width_s) {
  config::PipelineConfig config = MakeJointIntegrationPipelineConfig();
  config.expert.detection.enable_physics_detection = true;
  config.expert.detection.transmitter.pulse_width_s = pulse_width_s;
  if (pulse_width_s > 15e-6f) {
    config.expert.detection.transmitter.peak_power_w = 5.0e6f;
    config.expert.detection.transmitter.frequency_hz = 9.3e9f;
    config.expert.detection.transmitter.bandwidth_hz = 3.0e6f;
    config.expert.detection.transmitter.prf_hz = 220.0f;
    config.expert.detection.antenna.main_beam_gain_db = 38.0f;
    config.expert.detection.receiver.noise_figure_db = 3.0f;
  }
  return config;
}

float ComputeRange(const model::TargetFeature& target) {
  return std::sqrt(target.position_x * target.position_x + target.position_y * target.position_y +
                   target.position_z * target.position_z);
}

model::TargetFeature BuildTarget(std::uint64_t external_target_id, float velocity_x,
                                 float velocity_y, float velocity_z, float rcs, float position_x,
                                 float position_y, float position_z) {
  model::TargetFeature target(velocity_x, velocity_y, velocity_z, rcs, 0.0f, 0, external_target_id);
  target.has_cartesian_position = true;
  target.position_x = position_x;
  target.position_y = position_y;
  target.position_z = position_z;
  target.range_m = ComputeRange(target);
  return target;
}

model::TargetFeature BuildGroundTarget(std::uint64_t external_target_id, float position_x,
                                       float position_y, float rcs = 0.8f) {
  return BuildTarget(external_target_id, 0.0f, 0.0f, 0.0f, rcs, position_x, position_y, 0.0f);
}

model::TargetFeature BuildAirTarget(std::uint64_t external_target_id, float velocity_x,
                                    float velocity_y, float velocity_z, float rcs, float position_x,
                                    float position_y, float position_z) {
  return BuildTarget(external_target_id, velocity_x, velocity_y, velocity_z, rcs, position_x,
                     position_y, position_z);
}

model::TargetFeatureList BuildMixedPatrolTargetsWithBaseId(std::size_t count,
                                                           std::uint64_t base_target_id,
                                                           float x_bias, float y_bias,
                                                           float z_bias);

model::TargetFeatureList BuildStaticGroundTargets(std::size_t count, float x_bias, float y_bias) {
  model::TargetFeatureList targets;
  targets.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    targets.push_back(BuildGroundTarget(1000u + static_cast<std::uint64_t>(i),
                                        x_bias + static_cast<float>(i) * 15.0f,
                                        y_bias + static_cast<float>(i % 7) * 2.0f - 6.0f));
  }
  return targets;
}

model::TargetFeatureList BuildMixedPatrolTargets(std::size_t count, float x_bias, float y_bias,
                                                 float z_bias) {
  return BuildMixedPatrolTargetsWithBaseId(count, 5000u, x_bias, y_bias, z_bias);
}

model::TargetFeatureList BuildMixedPatrolTargetsWithBaseId(std::size_t count,
                                                           std::uint64_t base_target_id,
                                                           float x_bias, float y_bias,
                                                           float z_bias) {
  model::TargetFeatureList targets;
  targets.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint64_t target_id = base_target_id + static_cast<std::uint64_t>(i);
    if ((i % 3U) == 0U) {
      targets.push_back(BuildGroundTarget(target_id, x_bias + static_cast<float>(i) * 10.0f,
                                          y_bias + static_cast<float>(i % 5) * 3.0f - 6.0f,
                                          0.7f + static_cast<float>(i % 4) * 0.05f));
      continue;
    }

    const float velocity_x = 55.0f + static_cast<float>(i % 11);
    const float velocity_y = (static_cast<float>(i % 5) - 2.0f) * 0.6f;
    targets.push_back(BuildAirTarget(
        target_id, velocity_x, velocity_y, 0.0f, 0.9f + static_cast<float>(i % 3) * 0.1f,
        x_bias + static_cast<float>(i) * 12.0f, y_bias + static_cast<float>(i % 9) * 1.5f - 6.0f,
        z_bias + static_cast<float>(i % 4) * 2.0f));
  }
  return targets;
}

void AdvanceTargets(float dt_sec, model::TargetFeatureList* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    model::TargetFeature& target = (*targets)[i];
    target.position_x += target.current_track_velocity_x * dt_sec;
    target.position_y += target.current_track_velocity_y * dt_sec;
    target.position_z += target.current_track_velocity_z * dt_sec;
    target.range_m = ComputeRange(target);
  }
}

std::unordered_map<std::uint64_t, const model::DecisionTrackSnapshot*> BuildTrackMapByExternalId(
    const output::TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, const model::DecisionTrackSnapshot*> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const model::DecisionTrackSnapshot& track = frame.tracks[i];
    if (track.state.external_target_id != 0U) {
      track_map[track.state.external_target_id] = &track;
    }
  }
  return track_map;
}

std::vector<const model::DecisionTrackSnapshot*> CollectTracksByExternalId(
    const output::TrackOutputFrame& frame, std::uint64_t external_target_id) {
  std::vector<const model::DecisionTrackSnapshot*> matching_tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.external_target_id == external_target_id) {
      matching_tracks.push_back(&frame.tracks[i]);
    }
  }
  return matching_tracks;
}

bool ContainsCommandType(const ScenarioRadarContext& radar_context,
                         extension::control::RadarCommandType type) {
  for (std::size_t i = 0; i < radar_context.SubmittedCommands().size(); ++i) {
    if (radar_context.SubmittedCommands()[i].type == type) {
      return true;
    }
  }
  return false;
}

std::size_t CountJammingFlaggedTracks(const output::TrackOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.jamming_detected) {
      ++count;
    }
  }
  return count;
}

environment::JammerEmitterState BuildJammerEmitter(environment::JammingTechnique technique,
                                                   float power_db, float js_db,
                                                   float frequency_overlap_ratio,
                                                   float prf_lock_risk, bool in_sidelobe) {
  environment::JammerEmitterState emitter;
  emitter.technique = technique;
  emitter.power_db = power_db;
  emitter.confidence = 1.0f;
  emitter.js_db = js_db;
  emitter.has_direction_deg = true;
  emitter.azimuth_deg = in_sidelobe ? 20.0f : 0.0f;
  emitter.elevation_deg = 0.0f;
  emitter.angular_span_deg = 6.0f + 16.0f * frequency_overlap_ratio + 8.0f * prf_lock_risk;
  return emitter;
}

environment::EnvironmentSceneState MakeClearScene() {
  return environment::EnvironmentSceneBuilder().Build();
}

environment::EnvironmentSceneState MakeNoiseScene() {
  return environment::EnvironmentSceneBuilder()
      .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kNoiseSuppression, 11.0f, 8.0f,
                                    0.20f, 0.10f, true))
      .Build();
}

environment::EnvironmentSceneState MakeDeceptionScene() {
  return environment::EnvironmentSceneBuilder()
      .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kDeception, 8.0f, 8.0f, 0.90f,
                                    0.90f, false))
      .Build();
}

environment::EnvironmentSceneState MakeRepeaterScene() {
  return environment::EnvironmentSceneBuilder()
      .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kRepeater, 8.5f, 7.0f, 0.15f,
                                    0.95f, false))
      .Build();
}

environment::EnvironmentSceneState MakeMixedScene() {
  return environment::EnvironmentSceneBuilder()
      .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kNoiseSuppression, 12.0f, 8.0f,
                                    0.18f, 0.10f, true))
      .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kDeception, 9.0f, 7.5f, 0.90f,
                                    0.90f, false))
      .Build();
}

void ExpectFrameContainsTargets(const output::TrackOutputFrame& frame,
                                const model::TargetFeatureList& targets) {
  const auto track_map = BuildTrackMapByExternalId(frame);
  ASSERT_EQ(track_map.size(), targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const std::uint64_t external_target_id = targets[i].external_target_id;
    ASSERT_NE(track_map.count(external_target_id), 0U);
  }
}

output::TrackOutputFrame RunScenarioCycle(extension::RadarController* controller,
                                          ScenarioRadarContext* radar_context,
                                          const model::TargetFeatureList& targets) {
  EXPECT_NE(controller, nullptr);
  EXPECT_NE(radar_context, nullptr);
  if (controller == nullptr || radar_context == nullptr) {
    return output::TrackOutputFrame();
  }
  radar_context->SetTargetFeatures(targets);
  controller->RunOnce();
  EXPECT_TRUE(controller->HasLatestTrackOutputFrame());
  return controller->GetLatestTrackOutputFrame();
}

CycleStats CaptureCycleStats(const output::TrackOutputFrame& frame,
                             const ScenarioRadarContext& radar_context,
                             const signal::pipeline::SignalPipeline& signal_pipeline,
                             std::size_t previous_command_count) {
  const extension::AssociationQualityMetrics metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  CycleStats stats;
  stats.published_track_count = frame.published_track_count;
  stats.confirmed_track_count = frame.confirmed_track_count;
  stats.jamming_track_count = CountJammingFlaggedTracks(frame);
  stats.command_delta_count = radar_context.SubmittedCommands().size() - previous_command_count;
  stats.association_stress = metrics.association_stress;
  stats.match_rate = metrics.match_rate;
  return stats;
}

void ExpectNoZeroPublishedCycles(const std::vector<CycleStats>& stats) {
  for (std::size_t i = 0; i < stats.size(); ++i) {
    EXPECT_GT(stats[i].published_track_count, 0U);
  }
}

void ExpectBoundedCommandBurst(const std::vector<CycleStats>& stats,
                               std::size_t max_command_delta) {
  for (std::size_t i = 0; i < stats.size(); ++i) {
    EXPECT_LE(stats[i].command_delta_count, max_command_delta);
  }
}

void ExpectFiniteTrackState(const model::DecisionTrackSnapshot& track) {
  EXPECT_TRUE(std::isfinite(track.state.position_x));
  EXPECT_TRUE(std::isfinite(track.state.position_y));
  EXPECT_TRUE(std::isfinite(track.state.position_z));
  EXPECT_TRUE(std::isfinite(track.state.velocity_x));
  EXPECT_TRUE(std::isfinite(track.state.velocity_y));
  EXPECT_TRUE(std::isfinite(track.state.velocity_z));
  EXPECT_TRUE(std::isfinite(track.state.speed));
}

void ExpectFrameContainsTargetIds(const output::TrackOutputFrame& frame,
                                  const std::vector<std::uint64_t>& target_ids) {
  for (std::size_t i = 0; i < target_ids.size(); ++i) {
    EXPECT_FALSE(CollectTracksByExternalId(frame, target_ids[i]).empty());
  }
}

std::vector<std::uint64_t> ExtractTargetIds(const model::TargetFeatureList& targets) {
  std::vector<std::uint64_t> target_ids;
  target_ids.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    target_ids.push_back(targets[i].external_target_id);
  }
  return target_ids;
}

}  // namespace

TEST(RadarJointIntegrationTest, StageOneGroundTargetsRemainStableWithoutInterference) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildGroundTarget(101u, 20.0f, -5.0f),
      BuildGroundTarget(102u, 45.0f, 8.0f),
      BuildGroundTarget(103u, 72.0f, -12.0f),
  };

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);

    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    EXPECT_FALSE(frame.contains_lost_tracks);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const model::DecisionTrackSnapshot& track = *track_map.at(targets[i].external_target_id);
      EXPECT_NEAR(track.state.position_z, 0.0f, 1e-5f);
      EXPECT_NEAR(track.state.speed, 0.0f, 1e-5f);
      EXPECT_FALSE(track.state.jamming_detected);
    }
  }

  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
}

TEST(RadarJointIntegrationTest, StageOneMovingAirTargetsKeepStableEnemyOutputWithoutInterference) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(201u, 70.0f, 0.0f, 0.0f, 0.9f, 200.0f, -20.0f, 18.0f),
      BuildAirTarget(202u, 85.0f, 2.0f, 0.0f, 1.0f, 280.0f, 10.0f, 22.0f),
      BuildAirTarget(203u, 100.0f, -1.0f, 0.0f, 1.1f, 360.0f, -8.0f, 26.0f),
      BuildAirTarget(204u, 110.0f, 1.5f, 0.0f, 0.95f, 440.0f, 14.0f, 30.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  std::unordered_map<std::uint64_t, float> previous_position_x;

  for (std::size_t cycle = 0; cycle < 4U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);

    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    EXPECT_FALSE(frame.contains_lost_tracks);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const model::TargetFeature& target = targets[i];
      const model::DecisionTrackSnapshot& track = *track_map.at(target.external_target_id);
      const float expected_speed =
          std::sqrt(target.current_track_velocity_x * target.current_track_velocity_x +
                    target.current_track_velocity_y * target.current_track_velocity_y +
                    target.current_track_velocity_z * target.current_track_velocity_z);
      EXPECT_GT(track.state.position_z, 0.0f);
      EXPECT_NEAR(track.state.speed, expected_speed, 1e-4f);
      EXPECT_FALSE(track.state.jamming_detected);

      if (cycle > 0U) {
        EXPECT_EQ(track.state.association_key, previous_keys[target.external_target_id]);
        EXPECT_GT(track.state.position_x, previous_position_x[target.external_target_id]);
      }
      previous_keys[target.external_target_id] = track.state.association_key;
      previous_position_x[target.external_target_id] = track.state.position_x;
    }

    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
}

TEST(RadarJointIntegrationTest,
     StageTwoNoiseSuppressionInterferenceKeepsEnemyOutputAndEnablesEccm) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(301u, 60.0f, 0.0f, 0.0f, 1.0f, 180.0f, -6.0f, 16.0f),
      BuildAirTarget(302u, 75.0f, 0.5f, 0.0f, 1.0f, 260.0f, 12.0f, 20.0f),
      BuildAirTarget(303u, 90.0f, -0.5f, 0.0f, 1.1f, 340.0f, -10.0f, 24.0f),
  };

  const output::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(baseline_frame.confirmed_track_count, targets.size());

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  environment_service.UpdateSceneState(
      environment::EnvironmentSceneBuilder()
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kNoiseSuppression, 12.0f,
                                        8.0f, 0.20f, 0.10f, true))
          .Build());

  const output::TrackOutputFrame jammed_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  EXPECT_GT(CountJammingFlaggedTracks(jammed_frame), 0U);
  ExpectFrameContainsTargets(jammed_frame, targets);

  const extension::control::RadarControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_TRUE(cycle_2_profile.enable_sidelobe_canceller);
  EXPECT_TRUE(cycle_2_profile.enable_adaptive_beamforming);
  EXPECT_GT(cycle_2_profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_TRUE(ContainsCommandType(radar_context,
                                  extension::control::RadarCommandType::ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsCommandType(radar_context,
                                  extension::control::RadarCommandType::SET_ECCM_BURNTHROUGH_GAIN));

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const output::TrackOutputFrame protected_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  EXPECT_GT(protected_frame.published_track_count, 0U);
  EXPECT_GT(protected_frame.confirmed_track_count, 0U);
  EXPECT_GT(CountJammingFlaggedTracks(protected_frame), 0U);
  ExpectFrameContainsTargets(protected_frame, targets);
}

TEST(RadarJointIntegrationTest,
     StageTwoDeceptionInterferenceImprovesAssociationAfterProfileApplies) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(401u, 1.0f, 0.0f, 0.0f, 1.0f, 4.0f, 0.0f, 3.0f),
  };

  const output::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(baseline_frame.confirmed_track_count, 1U);

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  environment_service.UpdateSceneState(
      environment::EnvironmentSceneBuilder()
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kDeception, 8.0f, 8.0f,
                                        0.90f, 0.90f, false))
          .Build());

  const output::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const extension::AssociationQualityMetrics cycle_2_metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  EXPECT_GT(CountJammingFlaggedTracks(cycle_2_frame), 0U);
  EXPECT_EQ(cycle_2_metrics.dominant_jamming_semantic, model::JammingSemantic::kDeception);

  const extension::control::RadarControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_TRUE(cycle_2_profile.enable_agility_frequency);
  EXPECT_TRUE(cycle_2_profile.enable_eccm_rejitter);
  EXPECT_TRUE(
      ContainsCommandType(radar_context, extension::control::RadarCommandType::SET_AGILITY_FREQ));
  EXPECT_TRUE(
      ContainsCommandType(radar_context, extension::control::RadarCommandType::SET_ECCM_REJITTER));

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const output::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const extension::AssociationQualityMetrics cycle_3_metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_GT(cycle_3_frame.confirmed_track_count, 0U);
  ExpectFrameContainsTargets(cycle_3_frame, targets);
  EXPECT_LE(cycle_3_metrics.association_stress, cycle_2_metrics.association_stress);
  EXPECT_GE(cycle_3_metrics.match_rate, cycle_2_metrics.match_rate);
}

TEST(RadarJointIntegrationTest, StageTwoRepeaterInterferenceKeepsTrackOutputAndSustainsRejitter) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(501u, 2.0f, 0.0f, 0.0f, 1.0f, 8.0f, -1.0f, 3.0f),
      BuildAirTarget(502u, 2.5f, 0.2f, 0.0f, 1.0f, 12.0f, 1.0f, 4.0f),
  };

  const output::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(baseline_frame.confirmed_track_count, targets.size());

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  environment_service.UpdateSceneState(
      environment::EnvironmentSceneBuilder()
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kRepeater, 7.5f, 7.0f, 0.10f,
                                        0.95f, false))
          .Build());

  const output::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const extension::control::RadarControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_GT(CountJammingFlaggedTracks(cycle_2_frame), 0U);
  EXPECT_TRUE(cycle_2_profile.enable_eccm_rejitter);
  EXPECT_TRUE(cycle_2_profile.enable_adaptive_beamforming);
  EXPECT_TRUE(
      ContainsCommandType(radar_context, extension::control::RadarCommandType::SET_ECCM_REJITTER));
  EXPECT_TRUE(ContainsCommandType(
      radar_context, extension::control::RadarCommandType::ENABLE_ADAPTIVE_BEAMFORMING));

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const output::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  EXPECT_GT(cycle_3_frame.published_track_count, 0U);
  EXPECT_GT(cycle_3_frame.confirmed_track_count, 0U);
  ExpectFrameContainsTargets(cycle_3_frame, targets);
}

TEST(RadarJointIntegrationTest,
     StageTwoMixedInterferenceStacksCountermeasuresAndPreservesEnemyInfo) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(601u, 4.0f, 0.0f, 0.0f, 1.0f, 14.0f, -4.0f, 4.0f),
      BuildAirTarget(602u, 4.5f, 0.1f, 0.0f, 1.1f, 20.0f, 6.0f, 5.0f),
      BuildAirTarget(603u, 5.0f, -0.1f, 0.0f, 1.0f, 28.0f, -3.0f, 6.0f),
  };

  const output::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(baseline_frame.confirmed_track_count, targets.size());

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  environment_service.UpdateSceneState(
      environment::EnvironmentSceneBuilder()
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kNoiseSuppression, 12.0f,
                                        8.0f, 0.15f, 0.10f, true))
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kDeception, 9.0f, 7.5f,
                                        0.90f, 0.90f, false))
          .Build());

  const output::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const extension::control::RadarControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_GT(CountJammingFlaggedTracks(cycle_2_frame), 0U);
  EXPECT_TRUE(cycle_2_profile.enable_sidelobe_canceller);
  EXPECT_TRUE(cycle_2_profile.enable_agility_frequency);
  EXPECT_TRUE(cycle_2_profile.enable_eccm_rejitter);
  EXPECT_GT(cycle_2_profile.eccm_burnthrough_gain, 1.0f);

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const output::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto track_map = BuildTrackMapByExternalId(cycle_3_frame);

  ASSERT_EQ(cycle_3_frame.published_track_count, targets.size());
  ASSERT_EQ(cycle_3_frame.confirmed_track_count, targets.size());
  EXPECT_GT(CountJammingFlaggedTracks(cycle_3_frame), 0U);
  ExpectFrameContainsTargets(cycle_3_frame, targets);
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const model::DecisionTrackSnapshot& track = *track_map.at(targets[i].external_target_id);
    EXPECT_GT(track.state.speed, 0.0f);
    EXPECT_GT(track.state.position_z, 0.0f);
    EXPECT_TRUE(track.state.jamming_detected);
  }
}

TEST(RadarJointIntegrationTest, MediumScaleStaticSearchMaintainsStableOutputAcrossTargetTiers) {
  const std::vector<std::size_t> target_tiers{10U, 50U, 100U};
  for (std::size_t tier_index = 0; tier_index < target_tiers.size(); ++tier_index) {
    signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    extension::RadarController controller(radar_context, signal_pipeline, environment_service);

    const model::TargetFeatureList targets =
        BuildStaticGroundTargets(target_tiers[tier_index], 60.0f, -8.0f);
    for (std::size_t cycle = 0; cycle < 5U; ++cycle) {
      const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
      ASSERT_EQ(frame.published_track_count, target_tiers[tier_index]);
      ASSERT_EQ(frame.confirmed_track_count, target_tiers[tier_index]);
      EXPECT_FALSE(frame.contains_lost_tracks);
      ExpectFrameContainsTargets(frame, targets);
      EXPECT_EQ(CountJammingFlaggedTracks(frame), 0U);
    }
    EXPECT_TRUE(radar_context.SubmittedCommands().empty());
  }
}

TEST(RadarJointIntegrationTest, MediumScaleDynamicSearchMaintainsStableAssociationsAcrossDuration) {
  struct DynamicTier {
    std::size_t target_count;
    std::size_t cycle_count;
  };

  const std::vector<DynamicTier> tiers{{10U, 20U}, {50U, 30U}};
  for (std::size_t tier_index = 0; tier_index < tiers.size(); ++tier_index) {
    signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    radar_context.SetCycleDeltaTimeSec(1.0f);
    extension::RadarController controller(radar_context, signal_pipeline, environment_service);

    model::TargetFeatureList targets =
        BuildMixedPatrolTargets(tiers[tier_index].target_count, 120.0f, -6.0f, 6.0f);
    std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
    std::vector<CycleStats> stats;
    stats.reserve(tiers[tier_index].cycle_count);

    for (std::size_t cycle = 0; cycle < tiers[tier_index].cycle_count; ++cycle) {
      const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
      const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
      stats.push_back(
          CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count));
      ASSERT_EQ(frame.published_track_count, tiers[tier_index].target_count);
      ASSERT_EQ(frame.confirmed_track_count, tiers[tier_index].target_count);
      ExpectFrameContainsTargets(frame, targets);

      const auto track_map = BuildTrackMapByExternalId(frame);
      for (std::size_t i = 0; i < targets.size(); ++i) {
        const std::uint64_t target_id = targets[i].external_target_id;
        ASSERT_NE(track_map.count(target_id), 0U);
        if (cycle > 0U) {
          EXPECT_EQ(track_map.at(target_id)->state.association_key, previous_keys[target_id]);
        }
        previous_keys[target_id] = track_map.at(target_id)->state.association_key;
      }
      AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
    }

    ExpectNoZeroPublishedCycles(stats);
    ExpectBoundedCommandBurst(stats, 5U);
    EXPECT_TRUE(radar_context.SubmittedCommands().empty());
  }
}

TEST(RadarJointIntegrationTest,
     FrequentSceneSwitchingKeepsOutputReadableAndFreezesPerCycleEnvironment) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);
  model::TargetFeatureList targets = BuildMixedPatrolTargets(20U, 100.0f, -5.0f, 6.0f);

  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false},   {MakeNoiseScene(), true},  {MakeDeceptionScene(), true},
      {MakeClearScene(), false},   {MakeMixedScene(), true},  {MakeClearScene(), false},
      {MakeRepeaterScene(), true}, {MakeClearScene(), false},
  };

  std::vector<CycleStats> stats;
  stats.reserve(script.size());
  std::size_t changed_profile_cycles = 0U;
  std::uint64_t previous_profile_version = 0U;

  for (std::size_t cycle = 0; cycle < script.size(); ++cycle) {
    environment_service.UpdateSceneState(script[cycle].scene_state);
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    stats.push_back(
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count));

    ASSERT_GE(frame.published_track_count, targets.size());
    ASSERT_GE(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));
    if (script[cycle].expect_jamming) {
      EXPECT_GT(stats.back().jamming_track_count, 0U);
    } else {
      EXPECT_EQ(stats.back().jamming_track_count, 0U);
    }

    const std::uint64_t current_profile_version = radar_context.LatestControlProfile().version;
    if (current_profile_version != previous_profile_version) {
      ++changed_profile_cycles;
    }
    previous_profile_version = current_profile_version;
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
  EXPECT_GT(changed_profile_cycles, 0U);
}

TEST(RadarJointIntegrationTest, LongDurationMediumLoadPatrolKeepsMetricsBoundedWithoutDivergence) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);
  model::TargetFeatureList targets = BuildMixedPatrolTargets(32U, 140.0f, -8.0f, 8.0f);

  const std::size_t cycle_count = 120U;
  std::vector<CycleStats> stats;
  stats.reserve(cycle_count);
  float max_association_stress = 0.0f;
  float min_match_rate = std::numeric_limits<float>::max();

  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    if ((cycle % 24U) == 0U) {
      const std::size_t phase = (cycle / 24U) % 5U;
      switch (phase) {
        case 1U:
          environment_service.UpdateSceneState(MakeNoiseScene());
          break;
        case 2U:
          environment_service.UpdateSceneState(MakeMixedScene());
          break;
        case 3U:
          environment_service.UpdateSceneState(MakeClearScene());
          break;
        case 4U:
          environment_service.UpdateSceneState(MakeDeceptionScene());
          break;
        case 0U:
        default:
          environment_service.UpdateSceneState(MakeClearScene());
          break;
      }
    }

    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);
    max_association_stress = std::max(max_association_stress, cycle_stats.association_stress);
    min_match_rate = std::min(min_match_rate, cycle_stats.match_rate);

    EXPECT_GE(frame.published_track_count, 24U);
    EXPECT_GE(frame.confirmed_track_count, 24U);
    EXPECT_LE(cycle_stats.jamming_track_count, targets.size());
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
  EXPECT_LE(max_association_stress, 1.0f);
  EXPECT_GE(min_match_rate, 0.0f);
}

TEST(RadarJointIntegrationTest, EmptySearchAreaKeepsTrackOutputReadableWithoutSpuriousCommands) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  const model::TargetFeatureList targets;
  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    EXPECT_EQ(frame.published_track_count, 0U);
    EXPECT_EQ(frame.confirmed_track_count, 0U);
    EXPECT_TRUE(frame.tracks.empty());
    EXPECT_FALSE(frame.contains_lost_tracks);
  }

  EXPECT_FALSE(
      ContainsCommandType(radar_context, extension::control::RadarCommandType::SET_AGILITY_FREQ));
  EXPECT_FALSE(
      ContainsCommandType(radar_context, extension::control::RadarCommandType::SET_ECCM_REJITTER));
  EXPECT_FALSE(ContainsCommandType(
      radar_context, extension::control::RadarCommandType::ENABLE_SIDELOBE_CANCELLER));
}

TEST(RadarJointIntegrationTest, DuplicateExternalTargetIdsAreRejectedAndPreviousFrameIsRetained) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList valid_targets{
      BuildAirTarget(9001u, 42.0f, 0.4f, 0.0f, 0.9f, 120.0f, -6.0f, 12.0f),
      BuildAirTarget(9002u, 71.0f, 0.1f, 0.0f, 1.1f, 240.0f, 1.0f, 18.0f),
  };
  const output::TrackOutputFrame previous_frame =
      RunScenarioCycle(&controller, &radar_context, valid_targets);
  ASSERT_EQ(previous_frame.published_track_count, valid_targets.size());
  ASSERT_FALSE(controller.HasValidationError());

  model::TargetFeatureList duplicate_targets{
      BuildAirTarget(9001u, 42.0f, 0.4f, 0.0f, 0.9f, 120.0f, -6.0f, 12.0f),
      BuildAirTarget(9001u, 58.0f, -0.2f, 0.0f, 1.0f, 175.0f, 9.0f, 15.0f),
      BuildAirTarget(9002u, 71.0f, 0.1f, 0.0f, 1.1f, 240.0f, 1.0f, 18.0f),
  };
  radar_context.SetTargetFeatures(duplicate_targets);
  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kDuplicateExternalTargetId;
              }) != issues.end());
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const output::TrackOutputFrame& retained_frame = controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(retained_frame.cycle_index, previous_frame.cycle_index);
  EXPECT_EQ(retained_frame.batch_id, previous_frame.batch_id);
  EXPECT_EQ(retained_frame.published_track_count, previous_frame.published_track_count);
  EXPECT_EQ(retained_frame.confirmed_track_count, previous_frame.confirmed_track_count);
}

TEST(RadarJointIntegrationTest, ExtremeRangeTargetsKeepFiniteStableOutputAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildGroundTarget(9101u, 2.5f, -0.5f, 0.8f),
      BuildAirTarget(9102u, 35.0f, 0.0f, 0.0f, 0.9f, 250.0f, 4.0f, 10.0f),
      BuildAirTarget(9103u, 65.0f, -0.3f, 0.0f, 1.0f, 3200.0f, -18.0f, 28.0f),
      BuildAirTarget(9104u, 80.0f, 0.2f, 0.0f, 1.1f, 7800.0f, 25.0f, 35.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  for (std::size_t cycle = 0; cycle < 4U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    EXPECT_FALSE(frame.contains_lost_tracks);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const model::DecisionTrackSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_FALSE(track.state.jamming_detected);
      EXPECT_GT(track.state.association_key, 0U);
      EXPECT_GT(track.state.position_x, 0.0f);
      if (cycle > 0U) {
        EXPECT_EQ(track.state.association_key, previous_keys[targets[i].external_target_id]);
      }
      previous_keys[targets[i].external_target_id] = track.state.association_key;
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest,
     StationaryAndHighSpeedTargetsRemainTrackableWithoutAssociationCollapse) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildGroundTarget(9201u, 35.0f, -4.0f, 0.9f),
      BuildAirTarget(9202u, 0.6f, 0.0f, 0.0f, 0.9f, 95.0f, 2.0f, 9.0f),
      BuildAirTarget(9203u, 140.0f, 0.8f, 0.0f, 1.0f, 180.0f, -3.0f, 16.0f),
      BuildAirTarget(9204u, 175.0f, -0.4f, 12.0f, 1.1f, 260.0f, 5.0f, 20.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  float previous_stationary_x = 0.0f;
  float previous_fast_x = 0.0f;
  for (std::size_t cycle = 0; cycle < 5U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    EXPECT_FALSE(frame.contains_lost_tracks);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    const model::DecisionTrackSnapshot& stationary_track = *track_map.at(9201u);
    const model::DecisionTrackSnapshot& fast_track = *track_map.at(9204u);
    ExpectFiniteTrackState(stationary_track);
    ExpectFiniteTrackState(fast_track);
    EXPECT_FALSE(stationary_track.state.jamming_detected);
    EXPECT_FALSE(fast_track.state.jamming_detected);
    EXPECT_NEAR(stationary_track.state.speed, 0.0f, 0.5f);
    EXPECT_GT(fast_track.state.speed, 150.0f);

    if (cycle > 0U) {
      EXPECT_EQ(stationary_track.state.association_key, previous_keys[9201u]);
      EXPECT_EQ(fast_track.state.association_key, previous_keys[9204u]);
      EXPECT_NEAR(stationary_track.state.position_x, previous_stationary_x, 3.0f);
      EXPECT_GT(fast_track.state.position_x, previous_fast_x);
    }
    previous_keys[9201u] = stationary_track.state.association_key;
    previous_keys[9204u] = fast_track.state.association_key;
    previous_stationary_x = stationary_track.state.position_x;
    previous_fast_x = fast_track.state.position_x;
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  EXPECT_FALSE(
      ContainsCommandType(radar_context, extension::control::RadarCommandType::SET_AGILITY_FREQ));
  EXPECT_FALSE(
      ContainsCommandType(radar_context, extension::control::RadarCommandType::SET_ECCM_REJITTER));
}

TEST(RadarJointIntegrationTest,
     TargetBurstGrowthAndShrinkKeepsSurvivingTargetsReadableAcrossScaleSteps) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  const model::TargetFeatureList initial_targets =
      BuildMixedPatrolTargetsWithBaseId(10U, 10000u, 120.0f, -8.0f, 8.0f);
  const model::TargetFeatureList burst_targets =
      BuildMixedPatrolTargetsWithBaseId(20U, 10100u, 320.0f, 10.0f, 10.0f);
  model::TargetFeatureList expanded_targets = initial_targets;
  expanded_targets.insert(expanded_targets.end(), burst_targets.begin(), burst_targets.end());
  model::TargetFeatureList shrunk_targets(initial_targets.begin(), initial_targets.begin() + 6);

  const output::TrackOutputFrame cycle_1_frame =
      RunScenarioCycle(&controller, &radar_context, initial_targets);
  ASSERT_EQ(cycle_1_frame.published_track_count, initial_targets.size());
  ASSERT_EQ(cycle_1_frame.confirmed_track_count, initial_targets.size());
  ExpectFrameContainsTargets(cycle_1_frame, initial_targets);

  const output::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, expanded_targets);
  ASSERT_GE(cycle_2_frame.published_track_count, expanded_targets.size());
  ASSERT_GE(cycle_2_frame.confirmed_track_count, expanded_targets.size());
  ExpectFrameContainsTargets(cycle_2_frame, expanded_targets);
  const auto cycle_2_track_map = BuildTrackMapByExternalId(cycle_2_frame);
  for (std::size_t i = 0; i < shrunk_targets.size(); ++i) {
    const model::DecisionTrackSnapshot& track =
        *cycle_2_track_map.at(shrunk_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.state.association_key, 0U);
  }

  const output::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, shrunk_targets);
  EXPECT_GE(cycle_3_frame.published_track_count, shrunk_targets.size());
  EXPECT_GE(cycle_3_frame.confirmed_track_count, shrunk_targets.size());
  const auto cycle_3_track_map = BuildTrackMapByExternalId(cycle_3_frame);
  for (std::size_t i = 0; i < shrunk_targets.size(); ++i) {
    const model::DecisionTrackSnapshot& track =
        *cycle_3_track_map.at(shrunk_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.state.association_key, 0U);
  }
}

TEST(RadarJointIntegrationTest,
     PartialDropoutThenRecoveryRestoresTrackOutputWithoutReadabilityCollapse) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList full_targets =
      BuildMixedPatrolTargetsWithBaseId(12U, 11000u, 150.0f, -10.0f, 8.0f);
  model::TargetFeatureList dropout_targets(full_targets.begin(), full_targets.begin() + 6);

  const output::TrackOutputFrame cycle_1_frame =
      RunScenarioCycle(&controller, &radar_context, full_targets);
  ASSERT_EQ(cycle_1_frame.published_track_count, full_targets.size());
  std::vector<std::uint64_t> full_target_ids;
  full_target_ids.reserve(full_targets.size());
  for (std::size_t i = 0; i < full_targets.size(); ++i) {
    full_target_ids.push_back(full_targets[i].external_target_id);
  }

  const output::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, dropout_targets);
  EXPECT_GE(cycle_2_frame.published_track_count, dropout_targets.size());
  EXPECT_GE(cycle_2_frame.confirmed_track_count, dropout_targets.size());
  const auto cycle_2_track_map = BuildTrackMapByExternalId(cycle_2_frame);
  for (std::size_t i = 0; i < dropout_targets.size(); ++i) {
    const model::DecisionTrackSnapshot& track =
        *cycle_2_track_map.at(dropout_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.state.association_key, 0U);
  }

  const output::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, full_targets);
  EXPECT_GE(cycle_3_frame.published_track_count, full_targets.size());
  EXPECT_GE(cycle_3_frame.confirmed_track_count, full_targets.size());
  ExpectFrameContainsTargetIds(cycle_3_frame, full_target_ids);
  const auto cycle_3_track_map = BuildTrackMapByExternalId(cycle_3_frame);
  for (std::size_t i = 0; i < dropout_targets.size(); ++i) {
    const model::DecisionTrackSnapshot& track =
        *cycle_3_track_map.at(dropout_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.state.association_key, 0U);
  }
}

TEST(RadarJointIntegrationTest, InputOrderingPermutationKeepsExternalIdentityStableAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets =
      BuildMixedPatrolTargetsWithBaseId(15U, 12000u, 180.0f, -6.0f, 8.0f);
  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  for (std::size_t cycle = 0; cycle < 4U; ++cycle) {
    if (cycle == 1U) {
      std::reverse(targets.begin(), targets.end());
    } else if (cycle == 2U) {
      std::rotate(targets.begin(), targets.begin() + 5, targets.end());
    } else if (cycle == 3U) {
      std::rotate(targets.rbegin(), targets.rbegin() + 4, targets.rend());
    }

    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_GE(frame.published_track_count, targets.size());
    ASSERT_GE(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const std::uint64_t target_id = targets[i].external_target_id;
      const model::DecisionTrackSnapshot& track = *track_map.at(target_id);
      ExpectFiniteTrackState(track);
      if (cycle > 0U) {
        EXPECT_EQ(track.state.association_key, previous_keys[target_id]);
      }
      previous_keys[target_id] = track.state.association_key;
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest,
     PulsedInterferenceKeepsOutputRecoverableAcrossSingleCycleJammingBursts) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);
  model::TargetFeatureList targets =
      BuildMixedPatrolTargetsWithBaseId(18U, 13000u, 200.0f, -4.0f, 8.0f);

  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false}, {MakeNoiseScene(), true},  {MakeClearScene(), false},
      {MakeMixedScene(), true},  {MakeClearScene(), false}, {MakeRepeaterScene(), true},
      {MakeClearScene(), false},
  };

  std::vector<CycleStats> stats;
  stats.reserve(script.size());
  for (std::size_t cycle = 0; cycle < script.size(); ++cycle) {
    environment_service.UpdateSceneState(script[cycle].scene_state);
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_EQ(frame.published_track_count, targets.size());
    EXPECT_EQ(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargets(frame, targets);
    if (script[cycle].expect_jamming) {
      EXPECT_GT(cycle_stats.jamming_track_count, 0U);
    } else {
      EXPECT_EQ(cycle_stats.jamming_track_count, 0U);
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarJointIntegrationTest,
     MixedUnknownExternalIdsKeepKnownTargetsRecoverableAndUnknownTracksFinite) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(0u, 42.0f, 0.0f, 0.0f, 0.9f, 110.0f, -4.0f, 12.0f),
      BuildGroundTarget(14001u, 150.0f, 6.0f, 0.8f),
      BuildAirTarget(0u, 58.0f, 0.2f, 0.0f, 1.0f, 190.0f, 8.0f, 16.0f),
      BuildAirTarget(14002u, 64.0f, -0.1f, 0.0f, 1.1f, 250.0f, -3.0f, 18.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_known_keys;
  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    ASSERT_EQ(CollectTracksByExternalId(frame, 0u).size(), 2U);
    ExpectFrameContainsTargetIds(frame, std::vector<std::uint64_t>{14001u, 14002u});

    const auto known_track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < 2U; ++i) {
      const std::uint64_t target_id = (i == 0U) ? 14001u : 14002u;
      const model::DecisionTrackSnapshot& track = *known_track_map.at(target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.state.association_key, 0U);
      if (cycle > 0U) {
        EXPECT_EQ(track.state.association_key, previous_known_keys[target_id]);
      }
      previous_known_keys[target_id] = track.state.association_key;
    }

    const std::vector<const model::DecisionTrackSnapshot*> unknown_tracks =
        CollectTracksByExternalId(frame, 0u);
    for (std::size_t i = 0; i < unknown_tracks.size(); ++i) {
      ExpectFiniteTrackState(*unknown_tracks[i]);
      EXPECT_NE(unknown_tracks[i]->state.association_key, 0U);
    }

    const std::vector<signal::tracking::TrackMeasurement> measurements =
        signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(measurements.size(), targets.size());
    std::size_t unknown_measurement_count = 0U;
    for (std::size_t i = 0; i < measurements.size(); ++i) {
      ASSERT_LT(measurements[i].raw_measurement.source_index, targets.size());
      if (measurements[i].raw_measurement.external_target_id == 0U) {
        ++unknown_measurement_count;
      }
    }
    EXPECT_EQ(unknown_measurement_count, 2U);
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest, CoLocatedTargetsWithDistinctIdsRemainSeparateAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(15001u, 38.0f, 0.0f, 0.0f, 0.9f, 160.0f, 2.0f, 14.0f),
      BuildAirTarget(15002u, 44.0f, 0.3f, 0.0f, 1.0f, 160.0f, 2.0f, 14.0f),
      BuildAirTarget(15003u, 51.0f, -0.2f, 0.0f, 1.1f, 160.0f, 2.0f, 14.0f),
  };

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_GE(frame.published_track_count, targets.size());
    ASSERT_GE(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    const std::uint64_t key_1 = track_map.at(15001u)->state.association_key;
    const std::uint64_t key_2 = track_map.at(15002u)->state.association_key;
    const std::uint64_t key_3 = track_map.at(15003u)->state.association_key;
    EXPECT_NE(key_1, 0U);
    EXPECT_NE(key_2, 0U);
    EXPECT_NE(key_3, 0U);
    EXPECT_NE(key_1, key_2);
    EXPECT_NE(key_1, key_3);
    EXPECT_NE(key_2, key_3);

    const std::vector<signal::tracking::TrackMeasurement> measurements =
        signal_pipeline.GetLastTrackMeasurements();
    ASSERT_EQ(measurements.size(), targets.size());
    for (std::size_t i = 0; i < measurements.size(); ++i) {
      ASSERT_LT(measurements[i].raw_measurement.source_index, targets.size());
      EXPECT_EQ(measurements[i].raw_measurement.external_target_id,
                targets[measurements[i].raw_measurement.source_index].external_target_id);
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest, SuddenVelocityMutationKeepsKnownTargetReadableAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(16001u, 28.0f, 0.0f, 0.0f, 1.0f, 140.0f, -2.0f, 14.0f),
      BuildAirTarget(16002u, 54.0f, 0.2f, 0.0f, 0.9f, 210.0f, 4.0f, 18.0f),
  };

  const output::TrackOutputFrame cycle_1_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto cycle_1_track_map = BuildTrackMapByExternalId(cycle_1_frame);
  const std::uint64_t mutated_key = cycle_1_track_map.at(16001u)->state.association_key;
  const float cycle_1_speed = cycle_1_track_map.at(16001u)->state.speed;
  const float cycle_1_position_x = cycle_1_track_map.at(16001u)->state.position_x;
  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);

  targets[0].current_track_velocity_x = 220.0f;
  targets[0].current_track_velocity_y = 2.0f;
  targets[0].current_track_velocity_z = 6.0f;
  targets[0].range_m = ComputeRange(targets[0]);
  const output::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto cycle_2_track_map = BuildTrackMapByExternalId(cycle_2_frame);
  const model::DecisionTrackSnapshot& cycle_2_track = *cycle_2_track_map.at(16001u);
  ExpectFiniteTrackState(cycle_2_track);
  EXPECT_EQ(cycle_2_track.state.association_key, mutated_key);
  EXPECT_GE(cycle_2_track.state.speed, cycle_1_speed);
  EXPECT_GT(cycle_2_track.state.position_x, cycle_1_position_x);
  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);

  targets[0].current_track_velocity_x = 35.0f;
  targets[0].current_track_velocity_y = -0.5f;
  targets[0].current_track_velocity_z = 0.0f;
  targets[0].range_m = ComputeRange(targets[0]);
  const output::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto cycle_3_track_map = BuildTrackMapByExternalId(cycle_3_frame);
  const model::DecisionTrackSnapshot& cycle_3_track = *cycle_3_track_map.at(16001u);
  ExpectFiniteTrackState(cycle_3_track);
  EXPECT_EQ(cycle_3_track.state.association_key, mutated_key);
  EXPECT_GT(cycle_3_track.state.speed, 20.0f);
  EXPECT_FALSE(cycle_3_frame.tracks.empty());
}

TEST(RadarJointIntegrationTest,
     FullBatchReplacementKeepsCurrentEnemySetReadableWithoutPipelineStall) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  const std::vector<model::TargetFeatureList> batches{
      BuildMixedPatrolTargetsWithBaseId(8U, 17000u, 160.0f, -8.0f, 8.0f),
      BuildMixedPatrolTargetsWithBaseId(8U, 17100u, 420.0f, 12.0f, 10.0f),
      BuildMixedPatrolTargetsWithBaseId(8U, 17200u, 220.0f, -14.0f, 12.0f),
  };

  for (std::size_t cycle = 0; cycle < batches.size(); ++cycle) {
    const output::TrackOutputFrame frame =
        RunScenarioCycle(&controller, &radar_context, batches[cycle]);
    EXPECT_GE(frame.published_track_count, batches[cycle].size());
    EXPECT_GE(frame.confirmed_track_count, batches[cycle].size());
    std::vector<std::uint64_t> current_target_ids;
    current_target_ids.reserve(batches[cycle].size());
    for (std::size_t i = 0; i < batches[cycle].size(); ++i) {
      current_target_ids.push_back(batches[cycle][i].external_target_id);
    }
    ExpectFrameContainsTargetIds(frame, current_target_ids);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < batches[cycle].size(); ++i) {
      const model::DecisionTrackSnapshot& track =
          *track_map.at(batches[cycle][i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.state.association_key, 0U);
    }
  }
}

TEST(RadarJointIntegrationTest, LongDurationPulsedInterferenceRecoversOnEveryClearWindow) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);
  model::TargetFeatureList targets =
      BuildMixedPatrolTargetsWithBaseId(20U, 18000u, 220.0f, -6.0f, 8.0f);

  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false},   {MakeNoiseScene(), true},     {MakeClearScene(), false},
      {MakeClearScene(), false},   {MakeMixedScene(), true},     {MakeClearScene(), false},
      {MakeRepeaterScene(), true}, {MakeClearScene(), false},    {MakeNoiseScene(), true},
      {MakeClearScene(), false},   {MakeDeceptionScene(), true}, {MakeClearScene(), false},
  };

  std::vector<CycleStats> stats;
  stats.reserve(script.size());
  std::size_t clear_recovery_cycles = 0U;
  for (std::size_t cycle = 0; cycle < script.size(); ++cycle) {
    environment_service.UpdateSceneState(script[cycle].scene_state);
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_EQ(frame.published_track_count, targets.size());
    EXPECT_EQ(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargets(frame, targets);
    if (script[cycle].expect_jamming) {
      EXPECT_GT(cycle_stats.jamming_track_count, 0U);
    } else {
      EXPECT_EQ(cycle_stats.jamming_track_count, 0U);
      if ((cycle > 0U) && script[cycle - 1U].expect_jamming) {
        ++clear_recovery_cycles;
      }
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
  EXPECT_GE(clear_recovery_cycles, 4U);
}

TEST(RadarJointIntegrationTest, InvalidCycleDeltaFallsBackWithoutBreakingTrackContinuity) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);
  model::TargetFeatureList targets =
      BuildMixedPatrolTargetsWithBaseId(6U, 19000u, 180.0f, -5.0f, 8.0f);

  radar_context.SetCycleDeltaTimeSec(1.0f);
  const output::TrackOutputFrame initial_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_GE(initial_frame.published_track_count, targets.size());
  ASSERT_GE(initial_frame.confirmed_track_count, targets.size());
  ExpectFrameContainsTargetIds(initial_frame, ExtractTargetIds(targets));

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  std::unordered_map<std::uint64_t, float> previous_x;
  const auto initial_track_map = BuildTrackMapByExternalId(initial_frame);
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const std::uint64_t target_id = targets[i].external_target_id;
    previous_keys[target_id] = initial_track_map.at(target_id)->state.association_key;
    previous_x[target_id] = initial_track_map.at(target_id)->state.position_x;
  }

  const std::vector<float> invalid_dts{0.0f, -1.0f, 0.0f, -3.0f};
  for (std::size_t cycle = 0; cycle < invalid_dts.size(); ++cycle) {
    radar_context.SetCycleDeltaTimeSec(invalid_dts[cycle]);
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.cycle_index, initial_frame.cycle_index);
    ASSERT_EQ(frame.batch_id, initial_frame.batch_id);
    ASSERT_EQ(frame.published_track_count, initial_frame.published_track_count);
    ASSERT_EQ(frame.confirmed_track_count, initial_frame.confirmed_track_count);
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const std::uint64_t target_id = targets[i].external_target_id;
      const model::DecisionTrackSnapshot& track = *track_map.at(target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.state.association_key, 0U);
      EXPECT_EQ(track.state.association_key, previous_keys[target_id]);
      EXPECT_FLOAT_EQ(track.state.position_x, previous_x[target_id]);
    }
    AdvanceTargets(1.0f, &targets);
  }
}

TEST(RadarJointIntegrationTest, NonPositiveRangeAndNearOriginInputsRemainFiniteAndReadable) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(20001u, 12.0f, 0.0f, 0.0f, 0.8f, 0.01f, 0.0f, 0.0f),
      BuildAirTarget(20002u, 8.0f, 0.1f, 0.0f, 0.9f, 0.05f, 0.02f, 0.01f),
      BuildAirTarget(20003u, 4.0f, 0.0f, 0.0f, 0.7f, 0.12f, -0.03f, 0.0f),
  };
  targets[0].range_m = 0.0f;
  targets[1].range_m = -5.0f;
  targets[2].range_m = 0.0f;

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const model::DecisionTrackSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.state.association_key, 0U);
    }
    AdvanceTargets(1.0f, &targets);
    targets[0].range_m = 0.0f;
    targets[1].range_m = -5.0f;
    targets[2].range_m = 0.0f;
  }
}

TEST(RadarJointIntegrationTest,
     MissingCartesianPositionWithNonPositiveRangeSurfacesValidationError) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildGroundTarget(20011u, 0.0f, 0.0f, 0.7f),
  };
  targets[0].has_cartesian_position = false;
  targets[0].range_m = 0.0f;
  radar_context.SetTargetFeatures(targets);

  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kMissingRangeAndCartesianPosition;
              }) != issues.end());
  EXPECT_FALSE(controller.HasLatestTrackOutputFrame());
  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
}

TEST(RadarJointIntegrationTest, ExtremeRcsSpreadKeepsAllTracksFiniteAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(21001u, 30.0f, 0.0f, 0.0f, 0.001f, 180.0f, -6.0f, 12.0f),
      BuildAirTarget(21002u, 42.0f, 0.1f, 0.0f, 0.05f, 230.0f, 4.0f, 15.0f),
      BuildAirTarget(21003u, 55.0f, -0.2f, 0.0f, 5.0f, 310.0f, -3.0f, 18.0f),
      BuildAirTarget(21004u, 68.0f, 0.3f, 0.0f, 50.0f, 390.0f, 8.0f, 22.0f),
  };

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const model::DecisionTrackSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_TRUE(std::isfinite(track.state.rcs));
      EXPECT_GT(track.state.association_key, 0U);
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest, UltraHighAltitudeTargetsRemainTrackableAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(22001u, 48.0f, 0.0f, 4.0f, 0.9f, 260.0f, -8.0f, 1200.0f),
      BuildAirTarget(22002u, 55.0f, -0.2f, 3.0f, 1.0f, 340.0f, 10.0f, 6000.0f),
      BuildAirTarget(22003u, 62.0f, 0.1f, -2.0f, 1.1f, 420.0f, -4.0f, 15000.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  for (std::size_t cycle = 0; cycle < 4U; ++cycle) {
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const model::DecisionTrackSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_GT(track.state.position_z, 500.0f);
      if (cycle > 0U) {
        EXPECT_EQ(track.state.association_key, previous_keys[targets[i].external_target_id]);
      }
      previous_keys[targets[i].external_target_id] = track.state.association_key;
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest, SimultaneousTargetAndJammerVolatilityKeepsCurrentEnemySetReadable) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  const std::vector<model::TargetFeatureList> batches{
      BuildMixedPatrolTargetsWithBaseId(6U, 23000u, 180.0f, -6.0f, 8.0f),
      BuildMixedPatrolTargetsWithBaseId(18U, 23100u, 320.0f, 10.0f, 10.0f),
      BuildMixedPatrolTargetsWithBaseId(10U, 23200u, 220.0f, -12.0f, 12.0f),
      BuildMixedPatrolTargetsWithBaseId(20U, 23300u, 420.0f, 8.0f, 9.0f),
      BuildMixedPatrolTargetsWithBaseId(8U, 23400u, 160.0f, -14.0f, 7.0f),
  };
  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false}, {MakeNoiseScene(), true},    {MakeClearScene(), false},
      {MakeMixedScene(), true},  {MakeRepeaterScene(), true},
  };

  std::vector<CycleStats> stats;
  for (std::size_t cycle = 0; cycle < batches.size(); ++cycle) {
    environment_service.UpdateSceneState(script[cycle].scene_state);
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame =
        RunScenarioCycle(&controller, &radar_context, batches[cycle]);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_GE(frame.published_track_count, batches[cycle].size());
    EXPECT_GE(frame.confirmed_track_count, batches[cycle].size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(batches[cycle]));
    if (script[cycle].expect_jamming) {
      EXPECT_GT(cycle_stats.jamming_track_count, 0U);
    }
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarJointIntegrationTest, LongDurationCycleDeltaAndGeometryVolatilityKeepsOutputReadable) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);
  model::TargetFeatureList targets =
      BuildMixedPatrolTargetsWithBaseId(10U, 24000u, 200.0f, -8.0f, 8.0f);

  const std::vector<float> dt_pattern{1.0f, 0.0f, -1.0f, 0.5f, 2.0f, 0.0f};
  std::vector<CycleStats> stats;
  for (std::size_t cycle = 0; cycle < 24U; ++cycle) {
    radar_context.SetCycleDeltaTimeSec(dt_pattern[cycle % dt_pattern.size()]);
    if ((cycle % 4U) == 1U) {
      targets[0].range_m = 0.0f;
      targets[1].range_m = -3.0f;
    }
    if ((cycle % 6U) == 2U) {
      targets[2].position_x = 0.08f;
      targets[2].position_y = 0.01f;
      targets[2].position_z = 0.0f;
      targets[2].range_m = 0.0f;
    }

    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    stats.push_back(
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count));

    ASSERT_GE(frame.published_track_count, targets.size());
    ASSERT_GE(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const std::uint64_t target_id = targets[i].external_target_id;
      const model::DecisionTrackSnapshot& track = *track_map.at(target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.state.association_key, 0U);
    }

    AdvanceTargets(1.0f, &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarJointIntegrationTest, BatchReplacementAndPulsedInterferenceKeepCurrentEnemySetVisible) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  const std::vector<model::TargetFeatureList> batches{
      BuildMixedPatrolTargetsWithBaseId(9U, 25000u, 180.0f, -8.0f, 8.0f),
      BuildMixedPatrolTargetsWithBaseId(14U, 25100u, 320.0f, 10.0f, 10.0f),
      BuildMixedPatrolTargetsWithBaseId(7U, 25200u, 220.0f, -14.0f, 12.0f),
      BuildMixedPatrolTargetsWithBaseId(16U, 25300u, 420.0f, 12.0f, 9.0f),
      BuildMixedPatrolTargetsWithBaseId(8U, 25400u, 160.0f, -10.0f, 7.0f),
      BuildMixedPatrolTargetsWithBaseId(15U, 25500u, 380.0f, 6.0f, 11.0f),
  };
  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false}, {MakeNoiseScene(), true},  {MakeClearScene(), false},
      {MakeMixedScene(), true},  {MakeClearScene(), false}, {MakeRepeaterScene(), true},
  };

  std::vector<CycleStats> stats;
  for (std::size_t cycle = 0; cycle < batches.size(); ++cycle) {
    environment_service.UpdateSceneState(script[cycle].scene_state);
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame =
        RunScenarioCycle(&controller, &radar_context, batches[cycle]);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_GE(frame.published_track_count, batches[cycle].size());
    EXPECT_GE(frame.confirmed_track_count, batches[cycle].size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(batches[cycle]));
    if (script[cycle].expect_jamming) {
      EXPECT_GT(cycle_stats.jamming_track_count, 0U);
    }
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarJointIntegrationTest, LongDurationExtremeRcsAndAltitudeMixKeepsMetricsControlled) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::RadarController controller(radar_context, signal_pipeline, environment_service);

  model::TargetFeatureList targets{
      BuildAirTarget(26001u, 34.0f, 0.0f, 0.0f, 0.001f, 180.0f, -5.0f, 20.0f),
      BuildAirTarget(26002u, 41.0f, 0.1f, 0.0f, 0.02f, 240.0f, 4.0f, 1200.0f),
      BuildAirTarget(26003u, 53.0f, -0.2f, 1.0f, 0.3f, 320.0f, -3.0f, 3000.0f),
      BuildAirTarget(26004u, 62.0f, 0.2f, -1.0f, 5.0f, 410.0f, 7.0f, 9000.0f),
      BuildAirTarget(26005u, 74.0f, -0.1f, 2.0f, 30.0f, 520.0f, -9.0f, 15000.0f),
  };

  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false},    {MakeNoiseScene(), true},  {MakeClearScene(), false},
      {MakeDeceptionScene(), true}, {MakeClearScene(), false}, {MakeMixedScene(), true},
  };

  std::vector<CycleStats> stats;
  float max_association_stress = 0.0f;
  for (std::size_t cycle = 0; cycle < 18U; ++cycle) {
    const std::size_t step = cycle % script.size();
    environment_service.UpdateSceneState(script[step].scene_state);
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);
    max_association_stress = std::max(max_association_stress, cycle_stats.association_stress);

    ASSERT_EQ(frame.published_track_count, targets.size());
    ASSERT_EQ(frame.confirmed_track_count, targets.size());
    ExpectFrameContainsTargets(frame, targets);
    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const model::DecisionTrackSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_TRUE(std::isfinite(track.state.rcs));
      EXPECT_GT(track.state.position_z, 10.0f);
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 6U);
  EXPECT_LE(max_association_stress, 1.0f);
}

TEST(RadarJointIntegrationTest, PhysicsPulseWidthRetuningStaysWithinBoundedWindow) {
  const model::TargetFeatureList base_targets =
      BuildMixedPatrolTargetsWithBaseId(24U, 27000u, 260.0f, -6.0f, 10.0f);
  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false},
      {MakeNoiseScene(), true},
      {MakeMixedScene(), true},
      {MakeClearScene(), false},
  };

  auto run_metrics = [&](float pulse_width_s) {
    signal::pipeline::SignalPipeline signal_pipeline(
        MakeJointIntegrationPhysicsPipelineConfig(pulse_width_s));
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    radar_context.SetCycleDeltaTimeSec(1.0f);
    extension::RadarController controller(radar_context, signal_pipeline, environment_service);

    model::TargetFeatureList targets = base_targets;
    std::vector<float> association_stress_series;
    std::size_t published_sum = 0U;
    for (std::size_t cycle = 0; cycle < 20U; ++cycle) {
      const SceneScriptStep& step = script[cycle % script.size()];
      environment_service.UpdateSceneState(step.scene_state);
      const output::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
      const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
      const CycleStats stats =
          CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
      published_sum += stats.published_track_count;
      association_stress_series.push_back(stats.association_stress);
      AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
    }

    std::sort(association_stress_series.begin(), association_stress_series.end());
    const std::size_t p95_index =
        static_cast<std::size_t>(0.95f * static_cast<float>(association_stress_series.size() - 1U));
    const float p95_stress = association_stress_series[p95_index];
    const float mean_published = static_cast<float>(published_sum) / 20.0f;
    return std::make_pair(mean_published, p95_stress);
  };

  const std::pair<float, float> baseline_metrics = run_metrics(13.0e-6f);
  const std::pair<float, float> retuned_metrics = run_metrics(14.3e-6f);

  const float mean_published_rel_delta = std::fabs(retuned_metrics.first - baseline_metrics.first) /
                                         std::max(baseline_metrics.first, 1.0f);
  const float p95_rel_delta = std::fabs(retuned_metrics.second - baseline_metrics.second) /
                              std::max(std::fabs(baseline_metrics.second), 1.0e-6f);

  EXPECT_LE(mean_published_rel_delta, 0.15f);
  EXPECT_LE(p95_rel_delta, 0.20f);
}

}  // namespace tests
}  // namespace airborne_radar
