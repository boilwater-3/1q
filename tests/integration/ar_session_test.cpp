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

#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "airborne_radar/runtime/ArController.h"
#include "airborne_radar/session/MutableArContext.h"
#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar {
namespace tests {

namespace {

using ScenarioRadarContext = session::MutableArContext;

struct CycleStats {
  std::size_t published_track_count{0U};
  std::size_t confirmed_track_count{0U};
  std::size_t jamming_track_count{0U};
  std::size_t command_delta_count{0U};
  float association_stress{0.0f};
  float match_rate{0.0f};
};

struct SceneScriptStep {
  session::EnvironmentSceneState scene_state{};
  bool expect_jamming{false};

  SceneScriptStep(const session::EnvironmentSceneState& scene_state_in, bool expect_jamming_in)
      : scene_state(scene_state_in), expect_jamming(expect_jamming_in) {}
};

config::ArSessionConfig MakeJointIntegrationSessionConfig() {
  return config::ArSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .EnableTrackingFilter(true)
      .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

config::ArSessionConfig MakeJointIntegrationPhysicsSessionConfig(float pulse_width_s) {
  config::ArSessionConfig config = MakeJointIntegrationSessionConfig();
  config.hardware.enable_physics_detection = true;
  config.hardware.transmitter.pulse_width_s = pulse_width_s;
  if (pulse_width_s > 15e-6f) {
    config.hardware.transmitter.peak_power_w = 5.0e6f;
    config.hardware.transmitter.frequency_hz = 9.3e9f;
    config.hardware.transmitter.bandwidth_hz = 3.0e6f;
    config.hardware.transmitter.prf_hz = 220.0f;
    config.hardware.antenna.main_beam_gain_db = 38.0f;
    config.hardware.receiver.noise_figure_db = 3.0f;
  }
  return config;
}

float ComputeRange(const session::ArSceneTarget& target) {
  return std::sqrt(target.position_x * target.position_x + target.position_y * target.position_y +
                   target.position_z * target.position_z);
}

session::ArSceneTarget BuildTarget(std::uint64_t external_target_id, float velocity_x,
                                      float velocity_y, float velocity_z, float rcs,
                                      float position_x, float position_y, float position_z) {
  session::ArSceneTarget target(velocity_x, velocity_y, velocity_z, rcs, 0.0f, 0,
                                   external_target_id);

  target.position_x = position_x;
  target.position_y = position_y;
  target.position_z = position_z;
  target.range_m = ComputeRange(target);
  return target;
}

session::ArSceneTarget BuildGroundTarget(std::uint64_t external_target_id, float position_x,
                                            float position_y, float rcs = 0.8f) {
  return BuildTarget(external_target_id, 0.0f, 0.0f, 0.0f, rcs, position_x, position_y, 0.0f);
}

session::ArSceneTarget BuildAirTarget(std::uint64_t external_target_id, float velocity_x,
                                         float velocity_y, float velocity_z, float rcs,
                                         float position_x, float position_y, float position_z) {
  return BuildTarget(external_target_id, velocity_x, velocity_y, velocity_z, rcs, position_x,
                     position_y, position_z);
}

session::ArSceneTargetList BuildMixedPatrolTargetsWithBaseId(std::size_t count,
                                                                std::uint64_t base_target_id,
                                                                float x_bias, float y_bias,
                                                                float z_bias);

session::ArSceneTargetList BuildStaticGroundTargets(std::size_t count, float x_bias,
                                                       float y_bias) {
  session::ArSceneTargetList targets;
  targets.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    targets.push_back(BuildGroundTarget(1000u + static_cast<std::uint64_t>(i),
                                        x_bias + static_cast<float>(i) * 15.0f,
                                        y_bias + static_cast<float>(i % 7) * 2.0f - 6.0f));
  }
  return targets;
}

session::ArSceneTargetList BuildMixedPatrolTargets(std::size_t count, float x_bias, float y_bias,
                                                      float z_bias) {
  return BuildMixedPatrolTargetsWithBaseId(count, 5000u, x_bias, y_bias, z_bias);
}

session::ArSceneTargetList BuildMixedPatrolTargetsWithBaseId(std::size_t count,
                                                                std::uint64_t base_target_id,
                                                                float x_bias, float y_bias,
                                                                float z_bias) {
  session::ArSceneTargetList targets;
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

session::ArSceneTargetList BuildRunwayPatrolScene(float aircraft_progress_m,
                                                     bool include_ground_targets) {
  session::ArSceneTargetList targets;
  targets.reserve(include_ground_targets ? 5U : 2U);

  if (include_ground_targets) {
    const float ground_shift_m = -aircraft_progress_m * 0.02f;
    session::ArSceneTarget ground_1 =
        BuildGroundTarget(19001u, 120.0f + ground_shift_m, -18.0f, 1.0f);
    session::ArSceneTarget ground_2 =
        BuildGroundTarget(19002u, 142.0f + ground_shift_m, 4.0f, 0.9f);
    session::ArSceneTarget slow_ground =
        BuildTarget(19003u, 0.35f, 0.0f, 0.0f, 0.95f, 168.0f + ground_shift_m, 16.0f, 0.0f);
    targets.push_back(ground_1);
    targets.push_back(ground_2);
    targets.push_back(slow_ground);
  }

  targets.push_back(BuildAirTarget(19011u, 0.8f, 0.2f, 0.0f, 1.1f, 42.0f, -34.0f, 16.0f));
  targets.push_back(BuildAirTarget(19012u, -0.5f, -0.2f, 0.0f, 1.0f, 58.0f, 30.0f, 15.0f));
  return targets;
}

void AdvanceTargets(float dt_sec, session::ArSceneTargetList* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    session::ArSceneTarget& target = (*targets)[i];
    target.position_x += target.velocity_x * dt_sec;
    target.position_y += target.velocity_y * dt_sec;
    target.position_z += target.velocity_z * dt_sec;
    target.range_m = ComputeRange(target);
  }
}

std::unordered_map<std::uint64_t, const session::TrackStateSnapshot*> BuildTrackMapByExternalId(
    const session::TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, const session::TrackStateSnapshot*> track_map;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const session::TrackStateSnapshot& track = frame.tracks[i];
    if (track.external_target_id != 0U) {
      track_map[track.external_target_id] = &track;
    }
  }
  return track_map;
}

std::vector<const session::TrackStateSnapshot*> CollectTracksByExternalId(
    const session::TrackOutputFrame& frame, std::uint64_t external_target_id) {
  std::vector<const session::TrackStateSnapshot*> matching_tracks;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].external_target_id == external_target_id) {
      matching_tracks.push_back(&frame.tracks[i]);
    }
  }
  return matching_tracks;
}

bool ContainsCommandType(const ScenarioRadarContext& radar_context,
                         session::ArCommandType type) {
  for (std::size_t i = 0; i < radar_context.SubmittedCommands().size(); ++i) {
    if (radar_context.SubmittedCommands()[i].type == type) {
      return true;
    }
  }
  return false;
}

std::size_t CountJammingFlaggedTracks(const session::TrackOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].jamming_detected) {
      ++count;
    }
  }
  return count;
}

config::JammerEmitterState BuildJammerEmitter(config::JammingTechnique technique,
                                                   float power_db, float js_db,
                                                   float frequency_overlap_ratio,
                                                   float prf_lock_risk, bool in_sidelobe) {
  config::JammerEmitterState emitter;
  emitter.technique = technique;
  emitter.power_db = power_db;
  emitter.confidence = 1.0f;
  emitter.js_db = js_db;
  emitter.position_x = in_sidelobe ? 3420.20f : 0.0f;  // sin(20 deg)*10000 or 0
  emitter.position_y = in_sidelobe ? 9396.93f : 10000.0f;  // cos(20 deg)*10000 or 10000
  emitter.position_z = 0.0f;  // elevation 0 deg
  emitter.angular_span_deg = 6.0f + 16.0f * frequency_overlap_ratio + 8.0f * prf_lock_risk;
  return emitter;
}

session::EnvironmentSceneState MakeClearScene() {
  return session::EnvironmentSceneState{};
}

session::EnvironmentSceneState MakeNoiseScene() {
  session::EnvironmentSceneState scene;
  scene.jammer_emitters.push_back(
      BuildJammerEmitter(config::JammingTechnique::kNoiseSuppression, 11.0f, 8.0f, 0.20f, 0.10f,
                         true));
  return scene;
}

session::EnvironmentSceneState MakeDeceptionScene() {
  session::EnvironmentSceneState scene;
  scene.jammer_emitters.push_back(BuildJammerEmitter(config::JammingTechnique::kDeception, 8.0f,
                                                     8.0f, 0.90f, 0.90f, false));
  return scene;
}

session::EnvironmentSceneState MakeRepeaterScene() {
  session::EnvironmentSceneState scene;
  scene.jammer_emitters.push_back(BuildJammerEmitter(config::JammingTechnique::kRepeater, 8.5f,
                                                     7.0f, 0.15f, 0.95f, false));
  return scene;
}

session::EnvironmentSceneState MakeMixedScene() {
  session::EnvironmentSceneState scene;
  scene.jammer_emitters.push_back(
      BuildJammerEmitter(config::JammingTechnique::kNoiseSuppression, 12.0f, 8.0f, 0.18f, 0.10f,
                         true));
  scene.jammer_emitters.push_back(BuildJammerEmitter(config::JammingTechnique::kDeception, 9.0f,
                                                     7.5f, 0.90f, 0.90f, false));
  return scene;
}

void ExpectFrameContainsTargets(const session::TrackOutputFrame& frame,
                                const session::ArSceneTargetList& targets) {
  const auto track_map = BuildTrackMapByExternalId(frame);
  ASSERT_EQ(track_map.size(), targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const std::uint64_t external_target_id = targets[i].external_target_id;
    ASSERT_NE(track_map.count(external_target_id), 0U);
  }
}

session::TrackOutputFrame RunScenarioCycle(extension::ArController* controller,
                                           ScenarioRadarContext* radar_context,
                                           const session::ArSceneTargetList& targets) {
  EXPECT_NE(controller, nullptr);
  EXPECT_NE(radar_context, nullptr);
  if (controller == nullptr || radar_context == nullptr) {
    return session::TrackOutputFrame();
  }
  radar_context->SetSceneTargets(targets);
  controller->RunOnce();
  EXPECT_TRUE(controller->HasLatestTrackOutputFrame());
  return controller->GetLatestTrackOutputFrame();
}

CycleStats CaptureCycleStats(const session::TrackOutputFrame& frame,
                             const ScenarioRadarContext& radar_context,
                             const signal::pipeline::SignalPipeline& signal_pipeline,
                             std::size_t previous_command_count) {
  const session::AssociationQualityMetrics metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  CycleStats stats;
  stats.published_track_count = frame.tracks.size();
  stats.confirmed_track_count = session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed);
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

void ExpectFiniteTrackState(const session::TrackStateSnapshot& track) {
  EXPECT_TRUE(std::isfinite(track.position_x));
  EXPECT_TRUE(std::isfinite(track.position_y));
  EXPECT_TRUE(std::isfinite(track.position_z));
  EXPECT_TRUE(std::isfinite(track.velocity_x));
  EXPECT_TRUE(std::isfinite(track.velocity_y));
  EXPECT_TRUE(std::isfinite(track.velocity_z));
  EXPECT_TRUE(std::isfinite(track.speed));
}

void ExpectFrameContainsTargetIds(const session::TrackOutputFrame& frame,
                                  const std::vector<std::uint64_t>& target_ids) {
  for (std::size_t i = 0; i < target_ids.size(); ++i) {
    EXPECT_FALSE(CollectTracksByExternalId(frame, target_ids[i]).empty());
  }
}

void ExpectFrameHasNoDuplicateKnownExternalIds(const session::TrackOutputFrame& frame) {
  std::unordered_map<std::uint64_t, std::size_t> counts;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const std::uint64_t target_id = frame.tracks[i].external_target_id;
    if (target_id == 0U) {
      continue;
    }
    ++counts[target_id];
  }
  for (std::unordered_map<std::uint64_t, std::size_t>::const_iterator it = counts.begin();
       it != counts.end(); ++it) {
    EXPECT_EQ(it->second, 1U) << "external_target_id=" << it->first;
  }
}

void ExpectReadableTrackOutputFrame(const session::TrackOutputFrame& frame,
                                    std::uint32_t expected_cycle_index,
                                    std::uint64_t expected_batch_id) {
  EXPECT_EQ(frame.cycle_index, expected_cycle_index);
  EXPECT_EQ(frame.batch_id, expected_batch_id);
  ExpectFrameHasNoDuplicateKnownExternalIds(frame);
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    ExpectFiniteTrackState(frame.tracks[i]);
    EXPECT_NE(frame.tracks[i].association_key, 0U);
    EXPECT_FALSE(frame.tracks[i].jamming_detected);
  }
}

std::vector<std::uint64_t> ExtractTargetIds(const session::ArSceneTargetList& targets) {
  std::vector<std::uint64_t> target_ids;
  target_ids.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    target_ids.push_back(targets[i].external_target_id);
  }
  return target_ids;
}

session::TrackOutputFrame RunScenarioCycleAt(extension::ArController* controller,
                                             ScenarioRadarContext* radar_context,
                                             std::uint32_t cycle_index,
                                             const session::ArSceneTargetList& targets) {
  EXPECT_NE(radar_context, nullptr);
  if (radar_context != nullptr) {
    radar_context->SetCycleIndex(cycle_index);
  }
  return RunScenarioCycle(controller, radar_context, targets);
}

}  // namespace

TEST(RadarJointIntegrationTest, StageOneGroundTargetsRemainStableWithoutInterference) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildGroundTarget(101u, 20.0f, -5.0f),
      BuildGroundTarget(102u, 45.0f, 8.0f),
      BuildGroundTarget(103u, 72.0f, -12.0f),
  };

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);

    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    EXPECT_FALSE(session::CountTracksByStatus(frame, session::TrackStatus::kLost) > 0U);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const session::TrackStateSnapshot& track = *track_map.at(targets[i].external_target_id);
      EXPECT_NEAR(track.position_z, 0.0f, 1e-5f);
      EXPECT_NEAR(track.speed, 0.0f, 1e-5f);
      EXPECT_FALSE(track.jamming_detected);
    }
  }

  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
}

TEST(RadarJointIntegrationTest, StageOneMovingAirTargetsKeepStableEnemyOutputWithoutInterference) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(201u, 70.0f, 0.0f, 0.0f, 0.9f, 200.0f, -20.0f, 18.0f),
      BuildAirTarget(202u, 85.0f, 2.0f, 0.0f, 1.0f, 280.0f, 10.0f, 22.0f),
      BuildAirTarget(203u, 100.0f, -1.0f, 0.0f, 1.1f, 360.0f, -8.0f, 26.0f),
      BuildAirTarget(204u, 110.0f, 1.5f, 0.0f, 0.95f, 440.0f, 14.0f, 30.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  std::unordered_map<std::uint64_t, float> previous_position_x;

  for (std::size_t cycle = 0; cycle < 4U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);

    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    EXPECT_FALSE(session::CountTracksByStatus(frame, session::TrackStatus::kLost) > 0U);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const session::ArSceneTarget& target = targets[i];
      const session::TrackStateSnapshot& track = *track_map.at(target.external_target_id);
      const float expected_speed =
          std::sqrt(target.velocity_x * target.velocity_x + target.velocity_y * target.velocity_y +
                    target.velocity_z * target.velocity_z);
      EXPECT_GT(track.position_z, 0.0f);
      EXPECT_NEAR(track.speed, expected_speed, 1e-4f);
      EXPECT_FALSE(track.jamming_detected);

      if (cycle > 0U) {
        EXPECT_EQ(track.association_key, previous_keys[target.external_target_id]);
        EXPECT_GT(track.position_x, previous_position_x[target.external_target_id]);
      }
      previous_keys[target.external_target_id] = track.association_key;
      previous_position_x[target.external_target_id] = track.position_x;
    }

    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  EXPECT_TRUE(radar_context.SubmittedCommands().empty());
}

TEST(RadarJointIntegrationTest,
     StageTwoNoiseSuppressionInterferenceKeepsEnemyOutputAndEnablesEccm) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(301u, 60.0f, 0.0f, 0.0f, 1.0f, 180.0f, -6.0f, 16.0f),
      BuildAirTarget(302u, 75.0f, 0.5f, 0.0f, 1.0f, 260.0f, 12.0f, 20.0f),
      BuildAirTarget(303u, 90.0f, -0.5f, 0.0f, 1.1f, 340.0f, -10.0f, 24.0f),
  };

  const session::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(session::CountTracksByStatus(baseline_frame, session::TrackStatus::kConfirmed),
            targets.size());

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  environment_service.UpdateSceneState(
      [&] {
        session::EnvironmentSceneState s;
        s.jammer_emitters.push_back(BuildJammerEmitter(
            config::JammingTechnique::kNoiseSuppression, 12.0f, 8.0f, 0.20f, 0.10f, true));
        return s;
      }());

  const session::TrackOutputFrame jammed_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  EXPECT_GT(CountJammingFlaggedTracks(jammed_frame), 0U);
  ExpectFrameContainsTargets(jammed_frame, targets);

  const session::ArControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_TRUE(cycle_2_profile.enable_sidelobe_canceller);
  EXPECT_TRUE(cycle_2_profile.enable_adaptive_beamforming);
  EXPECT_GT(cycle_2_profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_TRUE(ContainsCommandType(radar_context,
                                  session::ArCommandType::ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsCommandType(radar_context,
                                  session::ArCommandType::SET_ECCM_BURNTHROUGH_GAIN));

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const session::TrackOutputFrame protected_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  EXPECT_GT(protected_frame.tracks.size(), 0U);
  EXPECT_GT(session::CountTracksByStatus(protected_frame, session::TrackStatus::kConfirmed), 0U);
  EXPECT_GT(CountJammingFlaggedTracks(protected_frame), 0U);
  ExpectFrameContainsTargets(protected_frame, targets);
}

TEST(RadarJointIntegrationTest,
     StageTwoDeceptionInterferenceImprovesAssociationAfterProfileApplies) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(401u, 1.0f, 0.0f, 0.0f, 1.0f, 4.0f, 0.0f, 3.0f),
  };

  const session::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(session::CountTracksByStatus(baseline_frame, session::TrackStatus::kConfirmed), 1U);

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  environment_service.UpdateSceneState(
      [&] {
        session::EnvironmentSceneState s;
        s.jammer_emitters.push_back(BuildJammerEmitter(
            config::JammingTechnique::kDeception, 8.0f, 8.0f, 0.90f, 0.90f, false));
        return s;
      }());

  const session::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const session::AssociationQualityMetrics cycle_2_metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();
  EXPECT_GT(CountJammingFlaggedTracks(cycle_2_frame), 0U);
  EXPECT_EQ(cycle_2_metrics.dominant_jamming_semantic, config::JammingSemantic::kDeception);

  const session::ArControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_TRUE(cycle_2_profile.enable_agility_frequency);
  EXPECT_TRUE(cycle_2_profile.enable_eccm_rejitter);
  EXPECT_TRUE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_AGILITY_FREQ));
  EXPECT_TRUE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_ECCM_REJITTER));

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const session::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const session::AssociationQualityMetrics cycle_3_metrics =
      signal_pipeline.GetLastAssociationQualityMetrics();

  EXPECT_GT(session::CountTracksByStatus(cycle_3_frame, session::TrackStatus::kConfirmed), 0U);
  ExpectFrameContainsTargets(cycle_3_frame, targets);
  EXPECT_LE(cycle_3_metrics.association_stress, cycle_2_metrics.association_stress);
  EXPECT_GE(cycle_3_metrics.match_rate, cycle_2_metrics.match_rate);
}

TEST(RadarJointIntegrationTest, StageTwoRepeaterInterferenceKeepsTrackOutputAndSustainsRejitter) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(501u, 2.0f, 0.0f, 0.0f, 1.0f, 8.0f, -1.0f, 3.0f),
      BuildAirTarget(502u, 2.5f, 0.2f, 0.0f, 1.0f, 12.0f, 1.0f, 4.0f),
  };

  const session::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(session::CountTracksByStatus(baseline_frame, session::TrackStatus::kConfirmed),
            targets.size());

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  environment_service.UpdateSceneState(
      [&] {
        session::EnvironmentSceneState s;
        s.jammer_emitters.push_back(BuildJammerEmitter(
            config::JammingTechnique::kRepeater, 7.5f, 7.0f, 0.10f, 0.95f, false));
        return s;
      }());

  const session::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const session::ArControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_GT(CountJammingFlaggedTracks(cycle_2_frame), 0U);
  EXPECT_TRUE(cycle_2_profile.enable_eccm_rejitter);
  EXPECT_TRUE(cycle_2_profile.enable_adaptive_beamforming);
  EXPECT_TRUE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_ECCM_REJITTER));
  EXPECT_TRUE(ContainsCommandType(
      radar_context, session::ArCommandType::ENABLE_ADAPTIVE_BEAMFORMING));

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const session::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  EXPECT_GT(cycle_3_frame.tracks.size(), 0U);
  EXPECT_GT(session::CountTracksByStatus(cycle_3_frame, session::TrackStatus::kConfirmed), 0U);
  ExpectFrameContainsTargets(cycle_3_frame, targets);
}

TEST(RadarJointIntegrationTest,
     StageTwoMixedInterferenceStacksCountermeasuresAndPreservesEnemyInfo) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(601u, 4.0f, 0.0f, 0.0f, 1.0f, 14.0f, -4.0f, 4.0f),
      BuildAirTarget(602u, 4.5f, 0.1f, 0.0f, 1.1f, 20.0f, 6.0f, 5.0f),
      BuildAirTarget(603u, 5.0f, -0.1f, 0.0f, 1.0f, 28.0f, -3.0f, 6.0f),
  };

  const session::TrackOutputFrame baseline_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_EQ(session::CountTracksByStatus(baseline_frame, session::TrackStatus::kConfirmed),
            targets.size());

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  {
    session::EnvironmentSceneState s;
    s.jammer_emitters.push_back(BuildJammerEmitter(
        config::JammingTechnique::kNoiseSuppression, 12.0f, 8.0f, 0.15f, 0.10f, true));
    s.jammer_emitters.push_back(BuildJammerEmitter(
        config::JammingTechnique::kDeception, 9.0f, 7.5f, 0.90f, 0.90f, false));
    environment_service.UpdateSceneState(s);
  }

  const session::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const session::ArControlProfile cycle_2_profile =
      radar_context.LatestControlProfile();
  EXPECT_GT(CountJammingFlaggedTracks(cycle_2_frame), 0U);
  EXPECT_TRUE(cycle_2_profile.enable_sidelobe_canceller);
  EXPECT_TRUE(cycle_2_profile.enable_agility_frequency);
  EXPECT_TRUE(cycle_2_profile.enable_eccm_rejitter);
  EXPECT_GT(cycle_2_profile.eccm_burnthrough_gain, 1.0f);

  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  const session::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto track_map = BuildTrackMapByExternalId(cycle_3_frame);

  ASSERT_EQ(cycle_3_frame.tracks.size(), targets.size());
  ASSERT_EQ(session::CountTracksByStatus(cycle_3_frame, session::TrackStatus::kConfirmed),
            targets.size());
  EXPECT_GT(CountJammingFlaggedTracks(cycle_3_frame), 0U);
  ExpectFrameContainsTargets(cycle_3_frame, targets);
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const session::TrackStateSnapshot& track = *track_map.at(targets[i].external_target_id);
    EXPECT_GT(track.speed, 0.0f);
    EXPECT_GT(track.position_z, 0.0f);
    EXPECT_TRUE(track.jamming_detected);
  }
}

TEST(RadarJointIntegrationTest, MediumScaleStaticSearchMaintainsStableOutputAcrossTargetTiers) {
  const std::vector<std::size_t> target_tiers{10U, 50U, 100U};
  for (std::size_t tier_index = 0; tier_index < target_tiers.size(); ++tier_index) {
    signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    extension::ArController controller(radar_context, signal_pipeline, environment_service);

    const session::ArSceneTargetList targets =
        BuildStaticGroundTargets(target_tiers[tier_index], 60.0f, -8.0f);
    for (std::size_t cycle = 0; cycle < 5U; ++cycle) {
      const session::TrackOutputFrame frame =
          RunScenarioCycle(&controller, &radar_context, targets);
      ASSERT_EQ(frame.tracks.size(), target_tiers[tier_index]);
      ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed),
                target_tiers[tier_index]);
      EXPECT_FALSE(session::CountTracksByStatus(frame, session::TrackStatus::kLost) > 0U);
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
    signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    radar_context.SetCycleDeltaTimeSec(1.0f);
    extension::ArController controller(radar_context, signal_pipeline, environment_service);

    session::ArSceneTargetList targets =
        BuildMixedPatrolTargets(tiers[tier_index].target_count, 120.0f, -6.0f, 6.0f);
    std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
    std::vector<CycleStats> stats;
    stats.reserve(tiers[tier_index].cycle_count);

    for (std::size_t cycle = 0; cycle < tiers[tier_index].cycle_count; ++cycle) {
      const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
      const session::TrackOutputFrame frame =
          RunScenarioCycle(&controller, &radar_context, targets);
      stats.push_back(
          CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count));
      ASSERT_EQ(frame.tracks.size(), tiers[tier_index].target_count);
      ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed),
                tiers[tier_index].target_count);
      ExpectFrameContainsTargets(frame, targets);

      const auto track_map = BuildTrackMapByExternalId(frame);
      for (std::size_t i = 0; i < targets.size(); ++i) {
        const std::uint64_t target_id = targets[i].external_target_id;
        ASSERT_NE(track_map.count(target_id), 0U);
        if (cycle > 0U) {
          EXPECT_EQ(track_map.at(target_id)->association_key, previous_keys[target_id]);
        }
        previous_keys[target_id] = track_map.at(target_id)->association_key;
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
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);
  session::ArSceneTargetList targets = BuildMixedPatrolTargets(20U, 100.0f, -5.0f, 6.0f);

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
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    stats.push_back(
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count));

    ASSERT_GE(frame.tracks.size(), targets.size());
    ASSERT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
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
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);
  session::ArSceneTargetList targets = BuildMixedPatrolTargets(32U, 140.0f, -8.0f, 8.0f);

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
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);
    max_association_stress = std::max(max_association_stress, cycle_stats.association_stress);
    min_match_rate = std::min(min_match_rate, cycle_stats.match_rate);

    EXPECT_GE(frame.tracks.size(), 24U);
    EXPECT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), 24U);
    EXPECT_LE(cycle_stats.jamming_track_count, targets.size());
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
  EXPECT_LE(max_association_stress, 1.0f);
  EXPECT_GE(min_match_rate, 0.0f);
}

TEST(RadarJointIntegrationTest, EmptySearchAreaKeepsTrackOutputReadableWithoutSpuriousCommands) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  const session::ArSceneTargetList targets;
  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    EXPECT_EQ(frame.tracks.size(), 0U);
    EXPECT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), 0U);
    EXPECT_TRUE(frame.tracks.empty());
    EXPECT_FALSE(session::CountTracksByStatus(frame, session::TrackStatus::kLost) > 0U);
  }

  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_AGILITY_FREQ));
  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_ECCM_REJITTER));
  EXPECT_FALSE(ContainsCommandType(
      radar_context, session::ArCommandType::ENABLE_SIDELOBE_CANCELLER));
}

TEST(RadarJointIntegrationTest, DuplicateExternalTargetIdsAreRejectedAndPreviousFrameIsRetained) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList valid_targets{
      BuildAirTarget(9001u, 42.0f, 0.4f, 0.0f, 0.9f, 120.0f, -6.0f, 12.0f),
      BuildAirTarget(9002u, 71.0f, 0.1f, 0.0f, 1.1f, 240.0f, 1.0f, 18.0f),
  };
  const session::TrackOutputFrame previous_frame =
      RunScenarioCycle(&controller, &radar_context, valid_targets);
  ASSERT_EQ(previous_frame.tracks.size(), valid_targets.size());
  ASSERT_FALSE(controller.HasValidationError());

  session::ArSceneTargetList duplicate_targets{
      BuildAirTarget(9001u, 42.0f, 0.4f, 0.0f, 0.9f, 120.0f, -6.0f, 12.0f),
      BuildAirTarget(9001u, 58.0f, -0.2f, 0.0f, 1.0f, 175.0f, 9.0f, 15.0f),
      BuildAirTarget(9002u, 71.0f, 0.1f, 0.0f, 1.1f, 240.0f, 1.0f, 18.0f),
  };
  radar_context.SetSceneTargets(duplicate_targets);
  controller.RunOnce();

  EXPECT_TRUE(controller.HasValidationError());
  const session::ValidationIssueList& issues = controller.GetLastValidationIssues();
  EXPECT_TRUE(std::find_if(issues.begin(), issues.end(), [](const session::ValidationIssue& issue) {
                return issue.code == session::ValidationCode::kDuplicateExternalTargetId;
              }) != issues.end());
  ASSERT_TRUE(controller.HasLatestTrackOutputFrame());
  const session::TrackOutputFrame& retained_frame = controller.GetLatestTrackOutputFrame();
  EXPECT_EQ(retained_frame.cycle_index, previous_frame.cycle_index);
  EXPECT_EQ(retained_frame.batch_id, previous_frame.batch_id);
  EXPECT_EQ(retained_frame.tracks.size(), previous_frame.tracks.size());
  EXPECT_EQ(session::CountTracksByStatus(retained_frame, session::TrackStatus::kConfirmed),
            session::CountTracksByStatus(previous_frame, session::TrackStatus::kConfirmed));
}

TEST(RadarJointIntegrationTest, ExtremeRangeTargetsKeepFiniteStableOutputAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildGroundTarget(9101u, 2.5f, -0.5f, 0.8f),
      BuildAirTarget(9102u, 35.0f, 0.0f, 0.0f, 0.9f, 250.0f, 4.0f, 10.0f),
      BuildAirTarget(9103u, 65.0f, -0.3f, 0.0f, 1.0f, 3200.0f, -18.0f, 28.0f),
      BuildAirTarget(9104u, 80.0f, 0.2f, 0.0f, 1.1f, 7800.0f, 25.0f, 35.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  for (std::size_t cycle = 0; cycle < 4U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    EXPECT_FALSE(session::CountTracksByStatus(frame, session::TrackStatus::kLost) > 0U);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const session::TrackStateSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_FALSE(track.jamming_detected);
      EXPECT_GT(track.association_key, 0U);
      EXPECT_GT(track.position_x, 0.0f);
      if (cycle > 0U) {
        EXPECT_EQ(track.association_key, previous_keys[targets[i].external_target_id]);
      }
      previous_keys[targets[i].external_target_id] = track.association_key;
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest,
     StationaryAndHighSpeedTargetsRemainTrackableWithoutAssociationCollapse) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildGroundTarget(9201u, 35.0f, -4.0f, 0.9f),
      BuildAirTarget(9202u, 0.6f, 0.0f, 0.0f, 0.9f, 95.0f, 2.0f, 9.0f),
      BuildAirTarget(9203u, 140.0f, 0.8f, 0.0f, 1.0f, 180.0f, -3.0f, 16.0f),
      BuildAirTarget(9204u, 175.0f, -0.4f, 12.0f, 1.1f, 260.0f, 5.0f, 20.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  float previous_stationary_x = 0.0f;
  float previous_fast_x = 0.0f;
  for (std::size_t cycle = 0; cycle < 5U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    EXPECT_FALSE(session::CountTracksByStatus(frame, session::TrackStatus::kLost) > 0U);
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    const session::TrackStateSnapshot& stationary_track = *track_map.at(9201u);
    const session::TrackStateSnapshot& fast_track = *track_map.at(9204u);
    ExpectFiniteTrackState(stationary_track);
    ExpectFiniteTrackState(fast_track);
    EXPECT_FALSE(stationary_track.jamming_detected);
    EXPECT_FALSE(fast_track.jamming_detected);
    EXPECT_NEAR(stationary_track.speed, 0.0f, 0.5f);
    EXPECT_GT(fast_track.speed, 150.0f);

    if (cycle > 0U) {
      EXPECT_EQ(stationary_track.association_key, previous_keys[9201u]);
      EXPECT_EQ(fast_track.association_key, previous_keys[9204u]);
      EXPECT_NEAR(stationary_track.position_x, previous_stationary_x, 3.0f);
      EXPECT_GT(fast_track.position_x, previous_fast_x);
    }
    previous_keys[9201u] = stationary_track.association_key;
    previous_keys[9204u] = fast_track.association_key;
    previous_stationary_x = stationary_track.position_x;
    previous_fast_x = fast_track.position_x;
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_AGILITY_FREQ));
  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_ECCM_REJITTER));
}

TEST(RadarJointIntegrationTest,
     TargetBurstGrowthAndShrinkKeepsSurvivingTargetsReadableAcrossScaleSteps) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  const session::ArSceneTargetList initial_targets =
      BuildMixedPatrolTargetsWithBaseId(10U, 10000u, 120.0f, -8.0f, 8.0f);
  const session::ArSceneTargetList burst_targets =
      BuildMixedPatrolTargetsWithBaseId(20U, 10100u, 320.0f, 10.0f, 10.0f);
  session::ArSceneTargetList expanded_targets = initial_targets;
  expanded_targets.insert(expanded_targets.end(), burst_targets.begin(), burst_targets.end());
  session::ArSceneTargetList shrunk_targets(initial_targets.begin(),
                                               initial_targets.begin() + 6);

  const session::TrackOutputFrame cycle_1_frame =
      RunScenarioCycle(&controller, &radar_context, initial_targets);
  ASSERT_EQ(cycle_1_frame.tracks.size(), initial_targets.size());
  ASSERT_EQ(session::CountTracksByStatus(cycle_1_frame, session::TrackStatus::kConfirmed),
            initial_targets.size());
  ExpectFrameContainsTargets(cycle_1_frame, initial_targets);

  const session::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, expanded_targets);
  ASSERT_GE(cycle_2_frame.tracks.size(), expanded_targets.size());
  ASSERT_GE(session::CountTracksByStatus(cycle_2_frame, session::TrackStatus::kConfirmed),
            expanded_targets.size());
  ExpectFrameContainsTargets(cycle_2_frame, expanded_targets);
  const auto cycle_2_track_map = BuildTrackMapByExternalId(cycle_2_frame);
  for (std::size_t i = 0; i < shrunk_targets.size(); ++i) {
    const session::TrackStateSnapshot& track =
        *cycle_2_track_map.at(shrunk_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.association_key, 0U);
  }

  const session::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, shrunk_targets);
  EXPECT_GE(cycle_3_frame.tracks.size(), shrunk_targets.size());
  EXPECT_GE(session::CountTracksByStatus(cycle_3_frame, session::TrackStatus::kConfirmed),
            shrunk_targets.size());
  const auto cycle_3_track_map = BuildTrackMapByExternalId(cycle_3_frame);
  for (std::size_t i = 0; i < shrunk_targets.size(); ++i) {
    const session::TrackStateSnapshot& track =
        *cycle_3_track_map.at(shrunk_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.association_key, 0U);
  }
}

TEST(RadarJointIntegrationTest,
     PartialDropoutThenRecoveryRestoresTrackOutputWithoutReadabilityCollapse) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList full_targets =
      BuildMixedPatrolTargetsWithBaseId(12U, 11000u, 150.0f, -10.0f, 8.0f);
  session::ArSceneTargetList dropout_targets(full_targets.begin(), full_targets.begin() + 6);

  const session::TrackOutputFrame cycle_1_frame =
      RunScenarioCycle(&controller, &radar_context, full_targets);
  ASSERT_EQ(cycle_1_frame.tracks.size(), full_targets.size());
  std::vector<std::uint64_t> full_target_ids;
  full_target_ids.reserve(full_targets.size());
  for (std::size_t i = 0; i < full_targets.size(); ++i) {
    full_target_ids.push_back(full_targets[i].external_target_id);
  }

  const session::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, dropout_targets);
  EXPECT_GE(cycle_2_frame.tracks.size(), dropout_targets.size());
  EXPECT_GE(session::CountTracksByStatus(cycle_2_frame, session::TrackStatus::kConfirmed),
            dropout_targets.size());
  const auto cycle_2_track_map = BuildTrackMapByExternalId(cycle_2_frame);
  for (std::size_t i = 0; i < dropout_targets.size(); ++i) {
    const session::TrackStateSnapshot& track =
        *cycle_2_track_map.at(dropout_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.association_key, 0U);
  }

  const session::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, full_targets);
  EXPECT_GE(cycle_3_frame.tracks.size(), full_targets.size());
  EXPECT_GE(session::CountTracksByStatus(cycle_3_frame, session::TrackStatus::kConfirmed),
            full_targets.size());
  ExpectFrameContainsTargetIds(cycle_3_frame, full_target_ids);
  const auto cycle_3_track_map = BuildTrackMapByExternalId(cycle_3_frame);
  for (std::size_t i = 0; i < dropout_targets.size(); ++i) {
    const session::TrackStateSnapshot& track =
        *cycle_3_track_map.at(dropout_targets[i].external_target_id);
    ExpectFiniteTrackState(track);
    EXPECT_NE(track.association_key, 0U);
  }
}

TEST(RadarJointIntegrationTest,
     RunwayPatrolDropoutAndTurnbackRecoveryKeepsEveryOutputFrameReadable) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  struct PatrolCycle {
    float aircraft_progress_m;
    bool include_ground_targets;
  };

  const std::vector<PatrolCycle> patrol_script{
      {0.0f, true},    {70.0f, true},   {145.0f, true}, {230.0f, false},
      {315.0f, false}, {220.0f, false}, {135.0f, true}, {60.0f, true},
  };
  const std::vector<std::uint64_t> ground_target_ids{19001u, 19002u, 19003u};
  const std::vector<std::uint64_t> patrol_target_ids{19011u, 19012u};

  std::unordered_map<std::uint64_t, std::uint64_t> previous_patrol_keys;
  std::unordered_map<std::uint64_t, std::uint64_t> first_ground_keys;
  bool observed_ground_dropout = false;
  bool observed_ground_lost = false;
  bool observed_ground_recovery = false;

  for (std::size_t cycle = 0; cycle < patrol_script.size(); ++cycle) {
    const session::ArSceneTargetList targets = BuildRunwayPatrolScene(
        patrol_script[cycle].aircraft_progress_m, patrol_script[cycle].include_ground_targets);
    const session::TrackOutputFrame frame =
        RunScenarioCycleAt(&controller, &radar_context, static_cast<std::uint32_t>(cycle), targets);

    ExpectReadableTrackOutputFrame(frame, static_cast<std::uint32_t>(cycle),
                                   static_cast<std::uint64_t>(cycle + 1U));
    EXPECT_GE(frame.tracks.size(), targets.size());
    EXPECT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargetIds(frame, patrol_target_ids);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < patrol_target_ids.size(); ++i) {
      const std::uint64_t target_id = patrol_target_ids[i];
      ASSERT_NE(track_map.count(target_id), 0U);
      const session::TrackStateSnapshot& track = *track_map.at(target_id);
      EXPECT_GT(track.position_z, 0.0f);
      EXPECT_LT(track.speed, 5.0f);
      if (previous_patrol_keys.count(target_id) != 0U) {
        EXPECT_EQ(track.association_key, previous_patrol_keys[target_id]);
      }
      previous_patrol_keys[target_id] = track.association_key;
    }

    if (patrol_script[cycle].include_ground_targets) {
      ExpectFrameContainsTargetIds(frame, ground_target_ids);
      for (std::size_t i = 0; i < ground_target_ids.size(); ++i) {
        const std::uint64_t target_id = ground_target_ids[i];
        ASSERT_NE(track_map.count(target_id), 0U);
        const session::TrackStateSnapshot& track = *track_map.at(target_id);
        EXPECT_NEAR(track.position_z, 0.0f, 1e-5f);
        if (target_id == 19003u) {
          EXPECT_LT(track.speed, 2.0f);
        } else {
          EXPECT_LT(track.speed, 2.0f);
        }
        if (first_ground_keys.count(target_id) == 0U) {
          first_ground_keys[target_id] = track.association_key;
        } else if (observed_ground_dropout) {
          observed_ground_recovery = true;
          EXPECT_NE(track.association_key, 0U);
        } else {
          EXPECT_EQ(track.association_key, first_ground_keys[target_id]);
        }
      }
    } else {
      observed_ground_dropout = true;
      for (std::size_t i = 0; i < ground_target_ids.size(); ++i) {
        const std::vector<const session::TrackStateSnapshot*> lost_tracks =
            CollectTracksByExternalId(frame, ground_target_ids[i]);
        ASSERT_EQ(lost_tracks.size(), 1U);
        EXPECT_TRUE(lost_tracks[0]->status == session::TrackStatus::kConfirmed ||
                    lost_tracks[0]->status == session::TrackStatus::kLost);
        EXPECT_GT(lost_tracks[0]->miss_count, 0U);
        if (lost_tracks[0]->status == session::TrackStatus::kLost) {
          observed_ground_lost = true;
        }
      }
    }
  }

  EXPECT_TRUE(observed_ground_dropout);
  EXPECT_TRUE(observed_ground_lost);
  EXPECT_TRUE(observed_ground_recovery);
  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_AGILITY_FREQ));
  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_ECCM_REJITTER));
}

TEST(RadarJointIntegrationTest,
     RunwayPatrolLongBlindWindowRecyclesStaleGroundTracksBeforeTurnbackRecovery) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  struct PatrolCycle {
    float aircraft_progress_m;
    bool include_ground_targets;
  };

  const std::vector<PatrolCycle> patrol_script{
      {0.0f, true},    {60.0f, true},   {120.0f, true},  {220.0f, false}, {310.0f, false},
      {400.0f, false}, {490.0f, false}, {580.0f, false}, {130.0f, true},  {50.0f, true},
  };
  const std::vector<std::uint64_t> ground_target_ids{19001u, 19002u, 19003u};
  const std::vector<std::uint64_t> patrol_target_ids{19011u, 19012u};

  bool observed_ground_lost = false;
  bool observed_ground_recycled_window = false;
  bool observed_ground_recovery_after_recycle = false;

  for (std::size_t cycle = 0; cycle < patrol_script.size(); ++cycle) {
    const session::ArSceneTargetList targets = BuildRunwayPatrolScene(
        patrol_script[cycle].aircraft_progress_m, patrol_script[cycle].include_ground_targets);
    const session::TrackOutputFrame frame =
        RunScenarioCycleAt(&controller, &radar_context, static_cast<std::uint32_t>(cycle), targets);

    ExpectReadableTrackOutputFrame(frame, static_cast<std::uint32_t>(cycle),
                                   static_cast<std::uint64_t>(cycle + 1U));
    ExpectFrameContainsTargetIds(frame, patrol_target_ids);

    if (patrol_script[cycle].include_ground_targets) {
      ExpectFrameContainsTargetIds(frame, ground_target_ids);
      const auto track_map = BuildTrackMapByExternalId(frame);
      for (std::size_t i = 0; i < ground_target_ids.size(); ++i) {
        const session::TrackStateSnapshot& track = *track_map.at(ground_target_ids[i]);
        EXPECT_LT(track.speed, 2.0f);
      }
      if (observed_ground_recycled_window) {
        observed_ground_recovery_after_recycle = true;
      }
      continue;
    }

    std::size_t visible_ground_track_count = 0U;
    for (std::size_t i = 0; i < ground_target_ids.size(); ++i) {
      const std::vector<const session::TrackStateSnapshot*> ground_tracks =
          CollectTracksByExternalId(frame, ground_target_ids[i]);
      ASSERT_LE(ground_tracks.size(), 1U);
      if (!ground_tracks.empty()) {
        ++visible_ground_track_count;
        EXPECT_GT(ground_tracks[0]->miss_count, 0U);
        if (ground_tracks[0]->status == session::TrackStatus::kLost) {
          observed_ground_lost = true;
        }
      }
    }

    if (cycle >= 6U) {
      EXPECT_EQ(visible_ground_track_count, 0U);
      observed_ground_recycled_window = true;
    } else {
      EXPECT_EQ(visible_ground_track_count, ground_target_ids.size());
    }
  }

  EXPECT_TRUE(observed_ground_lost);
  EXPECT_TRUE(observed_ground_recycled_window);
  EXPECT_TRUE(observed_ground_recovery_after_recycle);
  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_AGILITY_FREQ));
  EXPECT_FALSE(
      ContainsCommandType(radar_context, session::ArCommandType::SET_ECCM_REJITTER));
}

TEST(RadarJointIntegrationTest, CommonPatrolScenariosKeepRecoveredTargetSpeedsBounded) {
  struct PatrolScenarioStep {
    float progress_m;
    bool include_ground_targets;
    float dt_sec;
  };

  struct PatrolScenario {
    std::uint64_t base_target_id;
    float ground_shift_scale;
    float companion_speed_x;
    float companion_lateral_speed;
    float companion_altitude_m;
    std::vector<PatrolScenarioStep> steps;
  };

  const std::vector<PatrolScenario> scenarios{
      {20000u,
       0.015f,
       0.7f,
       0.1f,
       14.0f,
       {{0.0f, true, 1.0f},
        {60.0f, true, 1.0f},
        {130.0f, true, 1.0f},
        {240.0f, false, 1.0f},
        {310.0f, false, 1.0f},
        {170.0f, true, 1.0f},
        {80.0f, true, 1.0f}}},
      {20100u,
       0.020f,
       -0.4f,
       0.3f,
       18.0f,
       {{0.0f, true, 0.5f},
        {45.0f, true, 0.5f},
        {110.0f, true, 0.75f},
        {190.0f, false, 1.0f},
        {260.0f, false, 1.0f},
        {125.0f, true, 0.75f},
        {40.0f, true, 0.5f}}},
      {20200u,
       0.010f,
       1.2f,
       -0.2f,
       12.0f,
       {{0.0f, true, 1.0f},
        {90.0f, true, 1.0f},
        {180.0f, false, 1.0f},
        {270.0f, true, 1.0f},
        {180.0f, false, 1.0f},
        {90.0f, true, 1.0f}}},
      {20300u,
       0.025f,
       0.2f,
       0.0f,
       16.0f,
       {{0.0f, true, 1.0f},
        {70.0f, true, 1.0f},
        {140.0f, true, 1.0f},
        {230.0f, false, 1.0f},
        {320.0f, false, 1.0f},
        {410.0f, false, 1.0f},
        {500.0f, false, 1.0f},
        {590.0f, false, 1.0f},
        {150.0f, true, 1.0f}}},
      {20400u,
       0.018f,
       1.5f,
       0.5f,
       20.0f,
       {{0.0f, true, 1.0f},
        {55.0f, true, 1.0f},
        {115.0f, true, 1.0f},
        {180.0f, true, 1.0f},
        {245.0f, true, 1.0f},
        {310.0f, true, 1.0f},
        {245.0f, true, 1.0f},
        {180.0f, true, 1.0f}}},
      {20500u,
       0.030f,
       -1.0f,
       -0.4f,
       13.0f,
       {{0.0f, true, 1.0f},
        {80.0f, true, 1.0f},
        {160.0f, false, 1.0f},
        {240.0f, false, 1.0f},
        {320.0f, true, 1.0f},
        {240.0f, false, 1.0f},
        {160.0f, false, 1.0f},
        {80.0f, true, 1.0f}}},
  };

  for (std::size_t scenario_index = 0; scenario_index < scenarios.size(); ++scenario_index) {
    const PatrolScenario& scenario = scenarios[scenario_index];
    signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    extension::ArController controller(radar_context, signal_pipeline, environment_service);

    const std::vector<std::uint64_t> ground_target_ids{
        scenario.base_target_id + 1U, scenario.base_target_id + 2U, scenario.base_target_id + 3U};
    const std::vector<std::uint64_t> companion_target_ids{scenario.base_target_id + 11U,
                                                          scenario.base_target_id + 12U};
    bool observed_dropout = false;
    bool observed_recovery = false;

    for (std::size_t cycle = 0; cycle < scenario.steps.size(); ++cycle) {
      const PatrolScenarioStep& step = scenario.steps[cycle];
      radar_context.SetCycleDeltaTimeSec(step.dt_sec);

      session::ArSceneTargetList targets;
      targets.reserve(step.include_ground_targets ? 5U : 2U);
      if (step.include_ground_targets) {
        const float shift_m = -step.progress_m * scenario.ground_shift_scale;
        targets.push_back(BuildGroundTarget(ground_target_ids[0], 90.0f + shift_m, -22.0f, 0.9f));
        targets.push_back(BuildGroundTarget(ground_target_ids[1], 125.0f + shift_m, 6.0f, 1.0f));
        targets.push_back(BuildTarget(ground_target_ids[2], 0.4f, 0.05f, 0.0f, 0.95f,
                                      155.0f + shift_m, 18.0f, 0.0f));
      }
      targets.push_back(BuildAirTarget(companion_target_ids[0], scenario.companion_speed_x,
                                       scenario.companion_lateral_speed, 0.0f, 1.1f, 45.0f, -35.0f,
                                       scenario.companion_altitude_m));
      targets.push_back(BuildAirTarget(companion_target_ids[1], scenario.companion_speed_x * 0.8f,
                                       -scenario.companion_lateral_speed, 0.0f, 1.0f, 62.0f, 32.0f,
                                       scenario.companion_altitude_m + 1.5f));

      const session::TrackOutputFrame frame = RunScenarioCycleAt(
          &controller, &radar_context, static_cast<std::uint32_t>(cycle), targets);
      ExpectReadableTrackOutputFrame(frame, static_cast<std::uint32_t>(cycle),
                                     static_cast<std::uint64_t>(cycle + 1U));
      ExpectFrameContainsTargetIds(frame, companion_target_ids);

      const auto track_map = BuildTrackMapByExternalId(frame);
      for (std::size_t i = 0; i < companion_target_ids.size(); ++i) {
        ASSERT_NE(track_map.count(companion_target_ids[i]), 0U);
        const session::TrackStateSnapshot& track = *track_map.at(companion_target_ids[i]);
        EXPECT_GT(track.position_z, 0.0f);
        EXPECT_LT(track.speed, 8.0f) << "scenario=" << scenario_index << " cycle=" << cycle;
      }

      if (step.include_ground_targets) {
        ExpectFrameContainsTargetIds(frame, ground_target_ids);
        for (std::size_t i = 0; i < ground_target_ids.size(); ++i) {
          ASSERT_NE(track_map.count(ground_target_ids[i]), 0U);
          const session::TrackStateSnapshot& track = *track_map.at(ground_target_ids[i]);
          EXPECT_NEAR(track.position_z, 0.0f, 1e-5f);
          EXPECT_LT(track.speed, 5.0f) << "scenario=" << scenario_index << " cycle=" << cycle
                                       << " target_id=" << ground_target_ids[i];
        }
        if (observed_dropout) {
          observed_recovery = true;
        }
      } else {
        observed_dropout = true;
        for (std::size_t i = 0; i < ground_target_ids.size(); ++i) {
          const std::vector<const session::TrackStateSnapshot*> ground_tracks =
              CollectTracksByExternalId(frame, ground_target_ids[i]);
          ASSERT_LE(ground_tracks.size(), 1U);
          if (!ground_tracks.empty()) {
            EXPECT_GT(ground_tracks[0]->miss_count, 0U);
            EXPECT_LT(ground_tracks[0]->speed, 8.0f)
                << "scenario=" << scenario_index << " cycle=" << cycle
                << " target_id=" << ground_target_ids[i];
          }
        }
      }
    }

    if (observed_dropout) {
      EXPECT_TRUE(observed_recovery) << "scenario=" << scenario_index;
    }
    EXPECT_FALSE(
        ContainsCommandType(radar_context, session::ArCommandType::SET_AGILITY_FREQ));
    EXPECT_FALSE(ContainsCommandType(radar_context,
                                     session::ArCommandType::SET_ECCM_REJITTER));
  }
}

TEST(RadarJointIntegrationTest, InputOrderingPermutationKeepsExternalIdentityStableAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets =
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

    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_GE(frame.tracks.size(), targets.size());
    ASSERT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const std::uint64_t target_id = targets[i].external_target_id;
      const session::TrackStateSnapshot& track = *track_map.at(target_id);
      ExpectFiniteTrackState(track);
      if (cycle > 0U) {
        EXPECT_EQ(track.association_key, previous_keys[target_id]);
      }
      previous_keys[target_id] = track.association_key;
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest,
     PulsedInterferenceKeepsOutputRecoverableAcrossSingleCycleJammingBursts) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);
  session::ArSceneTargetList targets =
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
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_EQ(frame.tracks.size(), targets.size());
    EXPECT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
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
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(0u, 42.0f, 0.0f, 0.0f, 0.9f, 110.0f, -4.0f, 12.0f),
      BuildGroundTarget(14001u, 150.0f, 6.0f, 0.8f),
      BuildAirTarget(0u, 58.0f, 0.2f, 0.0f, 1.0f, 190.0f, 8.0f, 16.0f),
      BuildAirTarget(14002u, 64.0f, -0.1f, 0.0f, 1.1f, 250.0f, -3.0f, 18.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_known_keys;
  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ASSERT_EQ(CollectTracksByExternalId(frame, 0u).size(), 2U);
    ExpectFrameContainsTargetIds(frame, std::vector<std::uint64_t>{14001u, 14002u});

    const auto known_track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < 2U; ++i) {
      const std::uint64_t target_id = (i == 0U) ? 14001u : 14002u;
      const session::TrackStateSnapshot& track = *known_track_map.at(target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.association_key, 0U);
      if (cycle > 0U) {
        EXPECT_EQ(track.association_key, previous_known_keys[target_id]);
      }
      previous_known_keys[target_id] = track.association_key;
    }

    const std::vector<const session::TrackStateSnapshot*> unknown_tracks =
        CollectTracksByExternalId(frame, 0u);
    for (std::size_t i = 0; i < unknown_tracks.size(); ++i) {
      ExpectFiniteTrackState(*unknown_tracks[i]);
      EXPECT_NE(unknown_tracks[i]->association_key, 0U);
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
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(15001u, 38.0f, 0.0f, 0.0f, 0.9f, 160.0f, 2.0f, 14.0f),
      BuildAirTarget(15002u, 44.0f, 0.3f, 0.0f, 1.0f, 160.0f, 2.0f, 14.0f),
      BuildAirTarget(15003u, 51.0f, -0.2f, 0.0f, 1.1f, 160.0f, 2.0f, 14.0f),
  };

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_GE(frame.tracks.size(), targets.size());
    ASSERT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    const std::uint64_t key_1 = track_map.at(15001u)->association_key;
    const std::uint64_t key_2 = track_map.at(15002u)->association_key;
    const std::uint64_t key_3 = track_map.at(15003u)->association_key;
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
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(16001u, 28.0f, 0.0f, 0.0f, 1.0f, 140.0f, -2.0f, 14.0f),
      BuildAirTarget(16002u, 54.0f, 0.2f, 0.0f, 0.9f, 210.0f, 4.0f, 18.0f),
  };

  const session::TrackOutputFrame cycle_1_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto cycle_1_track_map = BuildTrackMapByExternalId(cycle_1_frame);
  const std::uint64_t mutated_key = cycle_1_track_map.at(16001u)->association_key;
  const float cycle_1_speed = cycle_1_track_map.at(16001u)->speed;
  const float cycle_1_position_x = cycle_1_track_map.at(16001u)->position_x;
  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);

  targets[0].velocity_x = 220.0f;
  targets[0].velocity_y = 2.0f;
  targets[0].velocity_z = 6.0f;
  targets[0].range_m = ComputeRange(targets[0]);
  const session::TrackOutputFrame cycle_2_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto cycle_2_track_map = BuildTrackMapByExternalId(cycle_2_frame);
  const session::TrackStateSnapshot& cycle_2_track = *cycle_2_track_map.at(16001u);
  ExpectFiniteTrackState(cycle_2_track);
  EXPECT_EQ(cycle_2_track.association_key, mutated_key);
  EXPECT_GE(cycle_2_track.speed, cycle_1_speed);
  EXPECT_GT(cycle_2_track.position_x, cycle_1_position_x);
  AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);

  targets[0].velocity_x = 35.0f;
  targets[0].velocity_y = -0.5f;
  targets[0].velocity_z = 0.0f;
  targets[0].range_m = ComputeRange(targets[0]);
  const session::TrackOutputFrame cycle_3_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  const auto cycle_3_track_map = BuildTrackMapByExternalId(cycle_3_frame);
  const session::TrackStateSnapshot& cycle_3_track = *cycle_3_track_map.at(16001u);
  ExpectFiniteTrackState(cycle_3_track);
  EXPECT_EQ(cycle_3_track.association_key, mutated_key);
  EXPECT_GT(cycle_3_track.speed, 20.0f);
  EXPECT_FALSE(cycle_3_frame.tracks.empty());
}

TEST(RadarJointIntegrationTest,
     FullBatchReplacementKeepsCurrentEnemySetReadableWithoutPipelineStall) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  const std::vector<session::ArSceneTargetList> batches{
      BuildMixedPatrolTargetsWithBaseId(8U, 17000u, 160.0f, -8.0f, 8.0f),
      BuildMixedPatrolTargetsWithBaseId(8U, 17100u, 420.0f, 12.0f, 10.0f),
      BuildMixedPatrolTargetsWithBaseId(8U, 17200u, 220.0f, -14.0f, 12.0f),
  };

  for (std::size_t cycle = 0; cycle < batches.size(); ++cycle) {
    const session::TrackOutputFrame frame =
        RunScenarioCycle(&controller, &radar_context, batches[cycle]);
    EXPECT_GE(frame.tracks.size(), batches[cycle].size());
    EXPECT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed),
              batches[cycle].size());
    std::vector<std::uint64_t> current_target_ids;
    current_target_ids.reserve(batches[cycle].size());
    for (std::size_t i = 0; i < batches[cycle].size(); ++i) {
      current_target_ids.push_back(batches[cycle][i].external_target_id);
    }
    ExpectFrameContainsTargetIds(frame, current_target_ids);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < batches[cycle].size(); ++i) {
      const session::TrackStateSnapshot& track = *track_map.at(batches[cycle][i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.association_key, 0U);
    }
  }
}

TEST(RadarJointIntegrationTest, LongDurationPulsedInterferenceRecoversOnEveryClearWindow) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);
  session::ArSceneTargetList targets =
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
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_EQ(frame.tracks.size(), targets.size());
    EXPECT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
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
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);
  session::ArSceneTargetList targets =
      BuildMixedPatrolTargetsWithBaseId(6U, 19000u, 180.0f, -5.0f, 8.0f);

  radar_context.SetCycleDeltaTimeSec(1.0f);
  const session::TrackOutputFrame initial_frame =
      RunScenarioCycle(&controller, &radar_context, targets);
  ASSERT_GE(initial_frame.tracks.size(), targets.size());
  ASSERT_GE(session::CountTracksByStatus(initial_frame, session::TrackStatus::kConfirmed),
            targets.size());
  ExpectFrameContainsTargetIds(initial_frame, ExtractTargetIds(targets));

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  std::unordered_map<std::uint64_t, float> previous_x;
  const auto initial_track_map = BuildTrackMapByExternalId(initial_frame);
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const std::uint64_t target_id = targets[i].external_target_id;
    previous_keys[target_id] = initial_track_map.at(target_id)->association_key;
    previous_x[target_id] = initial_track_map.at(target_id)->position_x;
  }

  const std::vector<float> invalid_dts{0.0f, -1.0f, 0.0f, -3.0f};
  for (std::size_t cycle = 0; cycle < invalid_dts.size(); ++cycle) {
    radar_context.SetCycleDeltaTimeSec(invalid_dts[cycle]);
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.cycle_index, initial_frame.cycle_index);
    ASSERT_EQ(frame.batch_id, initial_frame.batch_id);
    ASSERT_EQ(frame.tracks.size(), initial_frame.tracks.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed),
              session::CountTracksByStatus(initial_frame, session::TrackStatus::kConfirmed));
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const std::uint64_t target_id = targets[i].external_target_id;
      const session::TrackStateSnapshot& track = *track_map.at(target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.association_key, 0U);
      EXPECT_EQ(track.association_key, previous_keys[target_id]);
      EXPECT_FLOAT_EQ(track.position_x, previous_x[target_id]);
    }
    AdvanceTargets(1.0f, &targets);
  }
}

TEST(RadarJointIntegrationTest, NonPositiveRangeAndNearOriginInputsRemainFiniteAndReadable) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(20001u, 12.0f, 0.0f, 0.0f, 0.8f, 0.01f, 0.0f, 0.0f),
      BuildAirTarget(20002u, 8.0f, 0.1f, 0.0f, 0.9f, 0.05f, 0.02f, 0.01f),
      BuildAirTarget(20003u, 4.0f, 0.0f, 0.0f, 0.7f, 0.12f, -0.03f, 0.0f),
  };
  targets[0].range_m = 0.0f;
  targets[1].range_m = -5.0f;
  targets[2].range_m = 0.0f;

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const session::TrackStateSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.association_key, 0U);
    }
    AdvanceTargets(1.0f, &targets);
    targets[0].range_m = 0.0f;
    targets[1].range_m = -5.0f;
    targets[2].range_m = 0.0f;
  }
}

TEST(RadarJointIntegrationTest,
     MissingCartesianPositionWithNonPositiveRangeSurfacesValidationError) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildGroundTarget(20011u, 0.0f, 0.0f, 0.7f),
  };

  targets[0].range_m = 0.0f;
  radar_context.SetSceneTargets(targets);

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
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(21001u, 30.0f, 0.0f, 0.0f, 0.001f, 180.0f, -6.0f, 12.0f),
      BuildAirTarget(21002u, 42.0f, 0.1f, 0.0f, 0.05f, 230.0f, 4.0f, 15.0f),
      BuildAirTarget(21003u, 55.0f, -0.2f, 0.0f, 5.0f, 310.0f, -3.0f, 18.0f),
      BuildAirTarget(21004u, 68.0f, 0.3f, 0.0f, 50.0f, 390.0f, 8.0f, 22.0f),
  };

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const session::TrackStateSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_TRUE(std::isfinite(track.rcs));
      EXPECT_GT(track.association_key, 0U);
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest, UltraHighAltitudeTargetsRemainTrackableAcrossCycles) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
      BuildAirTarget(22001u, 48.0f, 0.0f, 4.0f, 0.9f, 260.0f, -8.0f, 1200.0f),
      BuildAirTarget(22002u, 55.0f, -0.2f, 3.0f, 1.0f, 340.0f, 10.0f, 6000.0f),
      BuildAirTarget(22003u, 62.0f, 0.1f, -2.0f, 1.1f, 420.0f, -4.0f, 15000.0f),
  };

  std::unordered_map<std::uint64_t, std::uint64_t> previous_keys;
  for (std::size_t cycle = 0; cycle < 4U; ++cycle) {
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargets(frame, targets);

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const session::TrackStateSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_GT(track.position_z, 500.0f);
      if (cycle > 0U) {
        EXPECT_EQ(track.association_key, previous_keys[targets[i].external_target_id]);
      }
      previous_keys[targets[i].external_target_id] = track.association_key;
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }
}

TEST(RadarJointIntegrationTest, SimultaneousTargetAndJammerVolatilityKeepsCurrentEnemySetReadable) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  const std::vector<session::ArSceneTargetList> batches{
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
    const session::TrackOutputFrame frame =
        RunScenarioCycle(&controller, &radar_context, batches[cycle]);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_GE(frame.tracks.size(), batches[cycle].size());
    EXPECT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed),
              batches[cycle].size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(batches[cycle]));
    if (script[cycle].expect_jamming) {
      EXPECT_GT(cycle_stats.jamming_track_count, 0U);
    }
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarJointIntegrationTest, LongDurationCycleDeltaAndGeometryVolatilityKeepsOutputReadable) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  extension::ArController controller(radar_context, signal_pipeline, environment_service);
  session::ArSceneTargetList targets =
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
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    stats.push_back(
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count));

    ASSERT_GE(frame.tracks.size(), targets.size());
    ASSERT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(targets));

    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const std::uint64_t target_id = targets[i].external_target_id;
      const session::TrackStateSnapshot& track = *track_map.at(target_id);
      ExpectFiniteTrackState(track);
      EXPECT_NE(track.association_key, 0U);
    }

    AdvanceTargets(1.0f, &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarJointIntegrationTest, BatchReplacementAndPulsedInterferenceKeepCurrentEnemySetVisible) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  const std::vector<session::ArSceneTargetList> batches{
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
    const session::TrackOutputFrame frame =
        RunScenarioCycle(&controller, &radar_context, batches[cycle]);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);

    EXPECT_GE(frame.tracks.size(), batches[cycle].size());
    EXPECT_GE(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed),
              batches[cycle].size());
    ExpectFrameContainsTargetIds(frame, ExtractTargetIds(batches[cycle]));
    if (script[cycle].expect_jamming) {
      EXPECT_GT(cycle_stats.jamming_track_count, 0U);
    }
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarJointIntegrationTest, LongDurationExtremeRcsAndAltitudeMixKeepsMetricsControlled) {
  signal::pipeline::SignalPipeline signal_pipeline(MakeJointIntegrationSessionConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  extension::ArController controller(radar_context, signal_pipeline, environment_service);

  session::ArSceneTargetList targets{
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
    const session::TrackOutputFrame frame = RunScenarioCycle(&controller, &radar_context, targets);
    const CycleStats cycle_stats =
        CaptureCycleStats(frame, radar_context, signal_pipeline, previous_command_count);
    stats.push_back(cycle_stats);
    max_association_stress = std::max(max_association_stress, cycle_stats.association_stress);

    ASSERT_EQ(frame.tracks.size(), targets.size());
    ASSERT_EQ(session::CountTracksByStatus(frame, session::TrackStatus::kConfirmed), targets.size());
    ExpectFrameContainsTargets(frame, targets);
    const auto track_map = BuildTrackMapByExternalId(frame);
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const session::TrackStateSnapshot& track = *track_map.at(targets[i].external_target_id);
      ExpectFiniteTrackState(track);
      EXPECT_TRUE(std::isfinite(track.rcs));
      EXPECT_GT(track.position_z, 10.0f);
    }
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 6U);
  EXPECT_LE(max_association_stress, 1.0f);
}

TEST(RadarJointIntegrationTest, PhysicsPulseWidthRetuningStaysWithinBoundedWindow) {
  const session::ArSceneTargetList base_targets =
      BuildMixedPatrolTargetsWithBaseId(24U, 27000u, 260.0f, -6.0f, 10.0f);
  const std::vector<SceneScriptStep> script{
      {MakeClearScene(), false},
      {MakeNoiseScene(), true},
      {MakeMixedScene(), true},
      {MakeClearScene(), false},
  };

  auto run_metrics = [&](float pulse_width_s) {
    signal::pipeline::SignalPipeline signal_pipeline(
        MakeJointIntegrationPhysicsSessionConfig(pulse_width_s));
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    radar_context.SetCycleDeltaTimeSec(1.0f);
    extension::ArController controller(radar_context, signal_pipeline, environment_service);

    session::ArSceneTargetList targets = base_targets;
    std::vector<float> association_stress_series;
    std::size_t published_sum = 0U;
    for (std::size_t cycle = 0; cycle < 20U; ++cycle) {
      const SceneScriptStep& step = script[cycle % script.size()];
      environment_service.UpdateSceneState(step.scene_state);
      const session::TrackOutputFrame frame =
          RunScenarioCycle(&controller, &radar_context, targets);
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
