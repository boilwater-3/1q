// Copyright 2026. All Rights Reserved.
//
// @file radar_detection_stress_test.cpp
// @brief 验证探测任务在大规模与长时场景下的稳健性趋势。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/signal/config/SignalPipelineConfig.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "environment_test_fixture.h"

namespace airborne_radar {
namespace tests {

namespace {

class ScenarioRadarContext : public core::context::IRadarContext {
 public:
  explicit ScenarioRadarContext(common::model::TargetFeatureList target_features = {})
      : target_features_(std::move(target_features)) {}

  void BeginCycle(const core::context::RadarCycleInput& input) override {
    target_features_ = input.target_features;
    platform_attitude_deg_ = input.platform_attitude_deg;
    cycle_dt_sec_ = input.dt_sec;
    submitted_commands_.clear();
  }

  const common::model::TargetFeatureList& GetTargetFeatures() const override { return target_features_; }

  common::model::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_deg_;
  }

  float GetCycleDeltaTimeSec() const override { return cycle_dt_sec_; }

  void SubmitControlCommand(common::control::RadarCommand command) override {
    submitted_commands_.push_back(command);
  }

  void UpdateRadarControlProfile(const common::control::RadarControlProfile& profile) override {
    latest_control_profile_ = profile;
    has_latest_control_profile_ = true;
  }

  const std::vector<common::control::RadarCommand>& GetSubmittedCommands() const override {
    return submitted_commands_;
  }

  bool HasLatestControlProfile() const override { return has_latest_control_profile_; }

  const common::control::RadarControlProfile& GetLatestControlProfile() const override {
    return latest_control_profile_;
  }

  void SetTargetFeatures(common::model::TargetFeatureList target_features) {
    target_features_ = std::move(target_features);
  }

  void SetCycleDeltaTimeSec(float cycle_dt_sec) { cycle_dt_sec_ = cycle_dt_sec; }

  const std::vector<common::control::RadarCommand>& SubmittedCommands() const { return submitted_commands_; }

  const common::control::RadarControlProfile& LatestControlProfile() const {
    return latest_control_profile_;
  }

 private:
  common::model::TargetFeatureList target_features_;
  common::model::PlatformAttitudeDeg platform_attitude_deg_{};
  float cycle_dt_sec_{1.0f};
  std::vector<common::control::RadarCommand> submitted_commands_;
  common::control::RadarControlProfile latest_control_profile_{};
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

signal::config::SignalPipelineConfig MakeStressPipelineConfig() {
  signal::config::SignalPipelineConfig config;
  config.detection.min_detection_margin_db = -100.0f;
  config.lifecycle.enable_auto_lifecycle_manager = true;
  config.lifecycle.lifecycle_config.confirm_hits = 1u;
  config.tracking.kalman_measurement_noise_std = 1.0f;
  return config;
}

signal::config::SignalPipelineConfig MakeStressPhysicsPipelineConfig(float pulse_width_s) {
  signal::config::SignalPipelineConfig config = MakeStressPipelineConfig();
  config.detection.enable_physics_detection = true;
  config.detection.pulse_count = 64;
  config.detection.detection_policy.cfar_pfa = 0.5f;
  config.detection.detection_policy.min_snr_db = -50.0f;
  config.detection.transmitter.pulse_width_s = pulse_width_s;
  return config;
}

float ComputeRange(const common::model::TargetFeature& target) {
  return std::sqrt(target.position_x * target.position_x + target.position_y * target.position_y +
                   target.position_z * target.position_z);
}

common::model::TargetFeature BuildTarget(std::uint64_t external_target_id, float velocity_x,
                                  float velocity_y, float velocity_z, float rcs, float position_x,
                                  float position_y, float position_z) {
  common::model::TargetFeature target(velocity_x, velocity_y, velocity_z, rcs, 0.0f, 0,
                                      external_target_id);
  target.has_cartesian_position = true;
  target.position_x = position_x;
  target.position_y = position_y;
  target.position_z = position_z;
  target.range_m = ComputeRange(target);
  return target;
}

common::model::TargetFeatureList BuildBatchTargets(std::size_t count, float x_bias, float y_bias,
                                            float z_bias) {
  common::model::TargetFeatureList targets;
  targets.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    targets.push_back(BuildTarget(
        300000u + static_cast<std::uint64_t>(i), 120.0f + static_cast<float>(i % 5), 0.0f, 0.0f,
        0.9f + static_cast<float>(i % 3) * 0.1f, x_bias + static_cast<float>(i) * 8.0f,
        y_bias + static_cast<float>(i % 9) * 1.5f, z_bias + static_cast<float>(i % 4) * 1.0f));
  }
  return targets;
}

void AdvanceTargets(float dt_sec, common::model::TargetFeatureList* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    common::model::TargetFeature& target = (*targets)[i];
    target.position_x += target.current_track_velocity_x * dt_sec;
    target.position_y += target.current_track_velocity_y * dt_sec;
    target.position_z += target.current_track_velocity_z * dt_sec;
    target.range_m = ComputeRange(target);
  }
}

std::size_t CountJammingTracks(const common::output::TrackOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    if (frame.tracks[i].state.jamming_detected) {
      ++count;
    }
  }
  return count;
}

std::unordered_set<std::uint64_t> BuildExternalIdSet(const common::output::TrackOutputFrame& frame) {
  std::unordered_set<std::uint64_t> ids;
  ids.reserve(frame.tracks.size());
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    ids.insert(frame.tracks[i].state.external_target_id);
  }
  return ids;
}

std::unordered_set<std::uint64_t> BuildAssociationKeySet(const common::output::TrackOutputFrame& frame) {
  std::unordered_set<std::uint64_t> keys;
  keys.reserve(frame.tracks.size());
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    keys.insert(frame.tracks[i].state.association_key);
  }
  return keys;
}

environment::JammerEmitterState BuildJammerEmitter(environment::JammingTechnique technique,
                                                   float power_db, float js_db,
                                                   float frequency_overlap_ratio,
                                                   float prf_lock_risk, bool in_sidelobe) {
  environment::JammerEmitterState emitter =
      environment_test::MakeJammerEmitter(technique, power_db);
  emitter.js_db = js_db;
  emitter.frequency_overlap_ratio = frequency_overlap_ratio;
  emitter.prf_lock_risk = prf_lock_risk;
  emitter.in_sidelobe = in_sidelobe;
  return emitter;
}

environment::EnvironmentSceneState MakeSceneForIndex(std::size_t cycle_index) {
  switch (cycle_index % 4U) {
    case 1U:
      return environment_test::MemorySceneBuilder()
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kNoiseSuppression, 10.0f,
                                        8.0f, 0.20f, 0.10f, true))
          .BuildSceneState();
    case 2U:
      return environment_test::MemorySceneBuilder()
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kDeception, 8.0f, 8.0f,
                                        0.90f, 0.90f, false))
          .BuildSceneState();
    case 3U:
      return environment_test::MemorySceneBuilder()
          .AddJammer(BuildJammerEmitter(environment::JammingTechnique::kRepeater, 8.5f, 7.0f, 0.15f,
                                        0.95f, false))
          .BuildSceneState();
    case 0U:
    default:
      return environment_test::MemorySceneBuilder().BuildSceneState();
  }
}

CycleStats RunCycleAndCaptureStats(core::controller::RadarController* controller,
                                   ScenarioRadarContext* radar_context,
                                   signal::pipeline::SignalPipeline* signal_pipeline,
                                   const common::model::TargetFeatureList& targets,
                                   std::size_t previous_command_count,
                                   common::output::TrackOutputFrame* out_frame) {
  EXPECT_NE(controller, nullptr);
  EXPECT_NE(radar_context, nullptr);
  EXPECT_NE(signal_pipeline, nullptr);
  if (controller == nullptr || radar_context == nullptr || signal_pipeline == nullptr) {
    return CycleStats();
  }

  radar_context->SetTargetFeatures(targets);
  controller->RunOnce();
  EXPECT_TRUE(controller->HasLatestTrackOutputFrame());

  const common::output::TrackOutputFrame& frame = controller->GetLatestTrackOutputFrame();
  if (out_frame != nullptr) {
    *out_frame = frame;
  }

  const signal::pipeline::AssociationQualityMetrics metrics =
      signal_pipeline->GetLastAssociationQualityMetrics();
  CycleStats stats;
  stats.published_track_count = frame.published_track_count;
  stats.confirmed_track_count = frame.confirmed_track_count;
  stats.jamming_track_count = CountJammingTracks(frame);
  stats.command_delta_count = radar_context->SubmittedCommands().size() - previous_command_count;
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

}  // namespace

TEST(RadarDetectionStressTest, LargeBatchSingleCyclePublishesStableTrackOutput) {
  const std::vector<std::size_t> target_tiers{1000U, 5000U, 10000U};
  for (std::size_t i = 0; i < target_tiers.size(); ++i) {
    const std::size_t target_count = target_tiers[i];
    signal::pipeline::SignalPipeline signal_pipeline(MakeStressPipelineConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    core::controller::RadarController controller(radar_context, signal_pipeline,
                                                 environment_service);

    const common::model::TargetFeatureList targets = BuildBatchTargets(target_count, 500.0f, 10.0f, 3.0f);

    common::output::TrackOutputFrame frame;
    const CycleStats stats =
        RunCycleAndCaptureStats(&controller, &radar_context, &signal_pipeline, targets, 0U, &frame);

    ASSERT_EQ(stats.published_track_count, target_count);
    ASSERT_EQ(stats.confirmed_track_count, target_count);
    EXPECT_EQ(BuildExternalIdSet(frame).size(), target_count);
    EXPECT_EQ(BuildAssociationKeySet(frame).size(), target_count);
    EXPECT_EQ(frame.tracks.size(), target_count);
  }
}

TEST(RadarDetectionStressTest, LargeBatchMultiCyclePreservesOutputSetAcrossDuration) {
  const std::vector<std::size_t> target_tiers{500U, 1000U, 2000U};
  for (std::size_t i = 0; i < target_tiers.size(); ++i) {
    const std::size_t target_count = target_tiers[i];
    signal::pipeline::SignalPipeline signal_pipeline(MakeStressPipelineConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    radar_context.SetCycleDeltaTimeSec(1.0f);
    core::controller::RadarController controller(radar_context, signal_pipeline,
                                                 environment_service);

    common::model::TargetFeatureList targets = BuildBatchTargets(target_count, 800.0f, 16.0f, 4.0f);
    std::vector<CycleStats> stats;
    stats.reserve(10U);

    for (std::size_t cycle = 0; cycle < 10U; ++cycle) {
      const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
      common::output::TrackOutputFrame frame;
      stats.push_back(RunCycleAndCaptureStats(&controller, &radar_context, &signal_pipeline,
                                              targets, previous_command_count, &frame));
      EXPECT_EQ(BuildExternalIdSet(frame).size(), target_count);
      EXPECT_EQ(BuildAssociationKeySet(frame).size(), target_count);
      AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
    }

    ExpectNoZeroPublishedCycles(stats);
    ExpectBoundedCommandBurst(stats, 5U);
  }
}

TEST(RadarDetectionStressTest, LargeBatchSceneSwitchingMaintainsReadableOutput) {
  const std::size_t target_count = 1000U;
  signal::pipeline::SignalPipeline signal_pipeline(MakeStressPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

  common::model::TargetFeatureList targets = BuildBatchTargets(target_count, 700.0f, 10.0f, 2.0f);
  std::vector<CycleStats> stats;
  stats.reserve(12U);

  for (std::size_t cycle = 0; cycle < 12U; ++cycle) {
    environment_service.UpdateSceneState(MakeSceneForIndex(cycle));
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    common::output::TrackOutputFrame frame;
    stats.push_back(RunCycleAndCaptureStats(&controller, &radar_context, &signal_pipeline, targets,
                                            previous_command_count, &frame));
    EXPECT_EQ(frame.tracks.size(), target_count);
    EXPECT_EQ(BuildExternalIdSet(frame).size(), target_count);
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarDetectionStressTest, LongDurationPatrolKeepsMetricsWithinReasonableBounds) {
  const std::size_t target_count = 500U;
  const std::size_t cycle_count = 300U;
  signal::pipeline::SignalPipeline signal_pipeline(MakeStressPipelineConfig());
  environment::EnvironmentService environment_service;
  ScenarioRadarContext radar_context;
  radar_context.SetCycleDeltaTimeSec(1.0f);
  core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

  common::model::TargetFeatureList targets = BuildBatchTargets(target_count, 900.0f, 12.0f, 3.0f);
  std::vector<CycleStats> stats;
  stats.reserve(cycle_count);
  float max_association_stress = 0.0f;
  float min_match_rate = std::numeric_limits<float>::max();

  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    if ((cycle % 25U) == 0U) {
      environment_service.UpdateSceneState(MakeSceneForIndex(cycle / 25U));
    }
    const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
    CycleStats cycle_stats = RunCycleAndCaptureStats(&controller, &radar_context, &signal_pipeline,
                                                     targets, previous_command_count, nullptr);
    max_association_stress = std::max(max_association_stress, cycle_stats.association_stress);
    min_match_rate = std::min(min_match_rate, cycle_stats.match_rate);
    stats.push_back(cycle_stats);
    AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
  EXPECT_LE(max_association_stress, 1.0f);
  EXPECT_GE(min_match_rate, 0.0f);
  for (std::size_t i = 0; i < stats.size(); ++i) {
    EXPECT_GE(stats[i].published_track_count, target_count * 9U / 10U);
    EXPECT_GE(stats[i].confirmed_track_count, target_count * 9U / 10U);
  }
}

TEST(RadarDetectionStressTest, LoadStaircaseRemainsStableAcrossScaleTransitions) {
  const std::vector<std::size_t> target_tiers{10U, 100U, 500U, 1000U};
  std::vector<CycleStats> stats;
  for (std::size_t tier_index = 0; tier_index < target_tiers.size(); ++tier_index) {
    signal::pipeline::SignalPipeline signal_pipeline(MakeStressPipelineConfig());
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    radar_context.SetCycleDeltaTimeSec(1.0f);
    core::controller::RadarController controller(radar_context, signal_pipeline,
                                                 environment_service);
    common::model::TargetFeatureList targets = BuildBatchTargets(
        target_tiers[tier_index], 600.0f + static_cast<float>(tier_index) * 20.0f, 8.0f, 2.0f);
    for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
      const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
      common::output::TrackOutputFrame frame;
      const CycleStats cycle_stats = RunCycleAndCaptureStats(
          &controller, &radar_context, &signal_pipeline, targets, previous_command_count, &frame);
      stats.push_back(cycle_stats);
      EXPECT_EQ(frame.tracks.size(), target_tiers[tier_index]);
      EXPECT_EQ(BuildExternalIdSet(frame).size(), target_tiers[tier_index]);
      AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
    }
  }

  ExpectNoZeroPublishedCycles(stats);
  ExpectBoundedCommandBurst(stats, 5U);
}

TEST(RadarDetectionStressTest, PhysicsPulseWidthRetuningRemainsWithinBoundedStressWindow) {
  auto run_metrics = [](float pulse_width_s) {
    signal::pipeline::SignalPipeline signal_pipeline(MakeStressPhysicsPipelineConfig(pulse_width_s));
    environment::EnvironmentService environment_service;
    ScenarioRadarContext radar_context;
    radar_context.SetCycleDeltaTimeSec(1.0f);
    core::controller::RadarController controller(radar_context, signal_pipeline, environment_service);

    common::model::TargetFeatureList targets = BuildBatchTargets(300U, 850.0f, 14.0f, 4.0f);
    std::vector<float> stress_series;
    std::size_t published_sum = 0U;
    for (std::size_t cycle = 0; cycle < 40U; ++cycle) {
      if ((cycle % 8U) == 0U) {
        environment_service.UpdateSceneState(MakeSceneForIndex(cycle / 8U));
      }
      const std::size_t previous_command_count = radar_context.SubmittedCommands().size();
      CycleStats stats = RunCycleAndCaptureStats(&controller, &radar_context, &signal_pipeline,
                                                 targets, previous_command_count, nullptr);
      published_sum += stats.published_track_count;
      stress_series.push_back(stats.association_stress);
      AdvanceTargets(radar_context.GetCycleDeltaTimeSec(), &targets);
    }
    std::sort(stress_series.begin(), stress_series.end());
    const std::size_t p95_index =
        static_cast<std::size_t>(0.95f * static_cast<float>(stress_series.size() - 1U));
    const float p95_stress = stress_series[p95_index];
    const float mean_published = static_cast<float>(published_sum) / 40.0f;
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
