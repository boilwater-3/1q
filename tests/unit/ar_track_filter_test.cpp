// Copyright 2026. All Rights Reserved.
//
// @file ar_track_filter_test.cpp
// @brief 验证航迹滤波衰减策略的基础行为。

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <vector>

#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {

namespace {

float SpeedOf(const session::RadarSceneTarget& target) {
  return std::sqrt(target.velocity_x * target.velocity_x + target.velocity_y * target.velocity_y +
                   target.velocity_z * target.velocity_z);
}

signal::tracking::CycleContext MakeLifecycleCycle(std::uint32_t cycle_index,
                                                  std::uint64_t batch_id) {
  signal::tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = 1.0f;
  cycle.extra_miss_tolerance = 0u;
  return cycle;
}

session::EnvironmentCycleContext MakeEnvironmentCycle(std::uint32_t cycle_index) {
  session::EnvironmentCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.dt_sec = 1.0f;
  return cycle;
}

template <typename PipelineType>
session::SignalCycleResult RunPipelineCycle(PipelineType* pipeline,
                                              const session::RadarSceneTargetList& input_state,
                                              environment::EnvironmentService* environment_service,
                                              std::uint32_t cycle_index = 1u) {
  environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index));
  return pipeline->RunCycle(input_state, *environment_service);
}

config::JammerEmitterState MakeJammerEmitter(config::JammingTechnique technique,
                                                  float power_db) {
  config::JammerEmitterState jammer;
  jammer.technique = technique;
  jammer.power_db = power_db;
  jammer.confidence = 1.0f;
  return jammer;
}

config::EnvironmentModelConfig MakeEnvironmentConfigWithJammers(
    std::initializer_list<config::JammerEmitterState> jammer_sources) {
  config::EnvironmentModelConfig config;
  config.jammer_sources.insert(config.jammer_sources.end(), jammer_sources.begin(),
                               jammer_sources.end());
  return config;
}

void ApplyDetectionIntentProfile(config::RadarSessionConfig* config,
                                 config::profiles::DetectionIntentProfile profile) {
  if (config == nullptr) {
    return;
  }
  auto& d = config->hardware;
  switch (profile) {
    case config::profiles::DetectionIntentProfile::kDetectionPriority:
      d.pulse_count = 16;
      d.detection_policy.cfar_pfa = 2e-6f;
      d.detection_policy.min_snr_db = -12.0f;
      d.min_detection_margin_db = -100.0f;
      break;
    case config::profiles::DetectionIntentProfile::kTrackStabilityPriority:
      d.pulse_count = 8;
      d.detection_policy.cfar_pfa = 5e-7f;
      d.detection_policy.min_snr_db = -8.0f;
      d.min_detection_margin_db = -20.0f;
      break;
    case config::profiles::DetectionIntentProfile::kBalanced:
    default:
      d.min_detection_margin_db = -2.0f;
      break;
  }
}

void ApplyLifecyclePolicyProfile(config::RadarSessionConfig* config,
                                 config::profiles::LifecyclePolicyProfile profile) {
  if (config == nullptr) {
    return;
  }
  auto& lc = config->policy.lifecycle;
  switch (profile) {
    case config::profiles::LifecyclePolicyProfile::kFastConfirm:
      lc.confirm_hits = 1U;
      lc.max_miss_before_lost = 1U;
      lc.max_lost_cycles = 3U;
      break;
    case config::profiles::LifecyclePolicyProfile::kHighPersistence:
      lc.confirm_hits = 3U;
      lc.max_miss_before_lost = 3U;
      lc.max_lost_cycles = 8U;
      break;
    case config::profiles::LifecyclePolicyProfile::kBalanced:
    default:
      break;
  }
}

}  // namespace

TEST(TrackFilterTest, KeepsStateWhenDetectionIsStable) {
  signal::tracking::TrackFilter filter;
  const session::RadarSceneTarget input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = true;
  context.jamming_detected = false;
  context.detection_margin_db = 0.0f;

  const session::RadarSceneTarget output = filter.Filter(input, context);

  EXPECT_FLOAT_EQ(SpeedOf(output), SpeedOf(input));
  EXPECT_FLOAT_EQ(output.rcs, input.rcs);
}

TEST(TrackFilterTest, AppliesLossDecayAndJammingPenalty) {
  signal::tracking::TrackFilter filter;
  const session::RadarSceneTarget input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = false;
  context.jamming_detected = true;
  context.detection_margin_db = -10.0f;

  const session::RadarSceneTarget output = filter.Filter(input, context);

  EXPECT_LT(SpeedOf(output), SpeedOf(input));
  EXPECT_LT(output.rcs, input.rcs);
}

TEST(TrackFilterTest, DeceptionJammingRetainsMoreTrackEnergyThanNoiseSuppression) {
  signal::tracking::TrackFilter filter;
  const session::RadarSceneTarget input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext noise_context;
  noise_context.detection_succeeded = false;
  noise_context.jamming_detected = true;
  noise_context.dominant_jamming_semantic = model::JammingSemantic::kNoiseSuppression;
  noise_context.jamming_severity = 0.8f;
  noise_context.detection_margin_db = -10.0f;

  signal::tracking::TrackFilterContext deception_context = noise_context;
  deception_context.dominant_jamming_semantic = model::JammingSemantic::kDeception;

  const session::RadarSceneTarget noise_output = filter.Filter(input, noise_context);
  const session::RadarSceneTarget deception_output = filter.Filter(input, deception_context);

  EXPECT_GT(SpeedOf(deception_output), SpeedOf(noise_output));
  EXPECT_GT(deception_output.rcs, noise_output.rcs);
}

TEST(TrackFilterTest, DetectionSuccessPreservesSpeedAndRcs) {
  signal::tracking::TrackFilter filter;

  session::RadarSceneTarget input(300.0f, 0.0f, 0.0f, 2.0f);

  input.position_x = 1000.0f;
  input.range_m = 1000.0f;

  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = true;

  const session::RadarSceneTarget output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(SpeedOf(output), 300.0f);
  EXPECT_FLOAT_EQ(output.rcs, 2.0f);
}

/// @brief 检测失配时，速度按配置系数衰减（speed = input * ratio）。

TEST(TrackFilterTest, DetectionMissDecaysSpeedByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.80f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;  // RCS 不衰减，隔离速度分支
  signal::tracking::TrackFilter filter(cfg);

  session::RadarSceneTarget input(500.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::RadarSceneTarget output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(SpeedOf(output), 400.0f);  // 500 * 0.8
}

/// @brief 检测失配时，RCS 按配置系数衰减，且不低于 0.05。

TEST(TrackFilterTest, DetectionMissDecaysRcsByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.70f;
  signal::tracking::TrackFilter filter(cfg);

  session::RadarSceneTarget input(100.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::RadarSceneTarget output = filter.Filter(input, ctx);

  EXPECT_NEAR(output.rcs, 1.40f, 1e-4f);  // 2.0 * 0.7
}

/// @brief 连续多次失配时，速度单调递减。

TEST(TrackFilterTest, ConsecutiveMissesMonotonicallyReduceSpeed) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.90f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  signal::tracking::TrackFilterContext miss_ctx;
  miss_ctx.detection_succeeded = false;

  session::RadarSceneTarget state(500.0f, 0.0f, 0.0f, 1.0f);
  float prev_speed = 500.0f;
  for (int i = 0; i < 5; ++i) {
    state = filter.Filter(state, miss_ctx);
    EXPECT_LT(SpeedOf(state), prev_speed)
        << "Speed should decrease at miss #" << (i + 1);
    prev_speed = SpeedOf(state);
  }
}

/// @brief 衰减系数为 1.0 时，失配不改变速度。

TEST(TrackFilterTest, DecayRatioOnePreservesSpeedOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  session::RadarSceneTarget input(400.0f, 0.0f, 0.0f, 1.5f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::RadarSceneTarget output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(SpeedOf(output), 400.0f);
}

/// @brief 速度不会因失配变为负数（钳位到 0）。

TEST(TrackFilterTest, SpeedNeverGoesNegativeOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  session::RadarSceneTarget input(300.0f, 0.0f, 0.0f, 1.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::RadarSceneTarget output = filter.Filter(input, ctx);

  EXPECT_GE(SpeedOf(output), 0.0f);
}

/// @brief RCS 不会因失配低于最小值 0.05。

TEST(TrackFilterTest, RcsNeverGoesBelowMinimumOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  signal::tracking::TrackFilter filter(cfg);

  session::RadarSceneTarget input(100.0f, 0.0f, 0.0f, 0.01f);  // 极小 RCS
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::RadarSceneTarget output = filter.Filter(input, ctx);

  EXPECT_GE(output.rcs, 0.05f);
}

}  // namespace tests
}  // namespace airborne_radar
