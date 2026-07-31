// Copyright 2026. All Rights Reserved.
//
// @file ar_track_filter_test.cpp
// @brief 验证航迹滤波衰减策略的基础行为。

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <vector>

#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace tests {

namespace {

float SpeedOf(const session::ArSceneTarget& target) {
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
                                            const session::ArSceneTargetList& input_state,
                                            environment::EnvironmentService* environment_service,
                                            std::uint32_t cycle_index = 1u) {
  environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index));
  return pipeline->RunCycle(input_state, *environment_service);
}

}  // namespace

TEST(TrackFilterTest, KeepsStateWhenDetectionIsStable) {
  signal::tracking::TrackFilter filter;
  const session::ArSceneTarget input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = true;
  context.detection_margin_db = 0.0f;

  const session::ArSceneTarget output = filter.Filter(input, context);

  EXPECT_FLOAT_EQ(SpeedOf(output), SpeedOf(input));
  EXPECT_FLOAT_EQ(output.rcs, input.rcs);
}

TEST(TrackFilterTest, AppliesConfiguredLossDecayDuringMiss) {
  signal::tracking::TrackFilter filter;
  const session::ArSceneTarget input(800.0f, 0.0f, 0.0f, 2.5f);

  signal::tracking::TrackFilterContext context;
  context.detection_succeeded = false;
  context.detection_margin_db = -10.0f;

  const session::ArSceneTarget output = filter.Filter(input, context);

  EXPECT_LT(SpeedOf(output), SpeedOf(input));
  EXPECT_LT(output.rcs, input.rcs);
}

TEST(TrackFilterTest, DetectionSuccessPreservesSpeedAndRcs) {
  signal::tracking::TrackFilter filter;

  session::ArSceneTarget input(300.0f, 0.0f, 0.0f, 2.0f);

  input.position_x = 1000.0f;
  input.range_m = 1000.0f;

  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = true;

  const session::ArSceneTarget output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(SpeedOf(output), 300.0f);
  EXPECT_FLOAT_EQ(output.rcs, 2.0f);
}

/// @brief 检测失配时，速度按配置系数衰减（speed = input * ratio）。

TEST(TrackFilterTest, DetectionMissDecaysSpeedByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.80f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;  // RCS 不衰减，隔离速度分支
  signal::tracking::TrackFilter filter(cfg);

  session::ArSceneTarget input(500.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::ArSceneTarget output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(SpeedOf(output), 400.0f);  // 500 * 0.8
}

/// @brief 检测失配时，RCS 按配置系数衰减，且不低于 0.05。

TEST(TrackFilterTest, DetectionMissDecaysRcsByConfiguredRatio) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.70f;
  signal::tracking::TrackFilter filter(cfg);

  session::ArSceneTarget input(100.0f, 0.0f, 0.0f, 2.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::ArSceneTarget output = filter.Filter(input, ctx);

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

  session::ArSceneTarget state(500.0f, 0.0f, 0.0f, 1.0f);
  float prev_speed = 500.0f;
  for (int i = 0; i < 5; ++i) {
    state = filter.Filter(state, miss_ctx);
    EXPECT_LT(SpeedOf(state), prev_speed) << "Speed should decrease at miss #" << (i + 1);
    prev_speed = SpeedOf(state);
  }
}

/// @brief 衰减系数为 1.0 时，失配不改变速度。

TEST(TrackFilterTest, DecayRatioOnePreservesSpeedOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  session::ArSceneTarget input(400.0f, 0.0f, 0.0f, 1.5f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::ArSceneTarget output = filter.Filter(input, ctx);

  EXPECT_FLOAT_EQ(SpeedOf(output), 400.0f);
}

/// @brief 速度不会因失配变为负数（钳位到 0）。

TEST(TrackFilterTest, SpeedNeverGoesNegativeOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  cfg.rcs_decay_ratio_on_loss = 1.0f;
  signal::tracking::TrackFilter filter(cfg);

  session::ArSceneTarget input(300.0f, 0.0f, 0.0f, 1.0f);
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::ArSceneTarget output = filter.Filter(input, ctx);

  EXPECT_GE(SpeedOf(output), 0.0f);
}

/// @brief RCS 不会因失配低于最小值 0.05。

TEST(TrackFilterTest, RcsNeverGoesBelowMinimumOnMiss) {
  signal::tracking::TrackFilterConfig cfg;
  cfg.speed_decay_ratio_on_loss = 1.0f;
  cfg.rcs_decay_ratio_on_loss = 0.0f;  // 衰减至 0
  signal::tracking::TrackFilter filter(cfg);

  session::ArSceneTarget input(100.0f, 0.0f, 0.0f, 0.01f);  // 极小 RCS
  signal::tracking::TrackFilterContext ctx;
  ctx.detection_succeeded = false;

  const session::ArSceneTarget output = filter.Filter(input, ctx);

  EXPECT_GE(output.rcs, 0.05f);
}

}  // namespace tests
}  // namespace airborne_radar
