// Copyright 2026. All Rights Reserved.
//
// @file ar_environment_service_test.cpp
// @brief 验证环境服务与场景管理的基础行为。

#include <gtest/gtest.h>

#include <initializer_list>
#include <vector>

#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/environment/SceneManager.h"

namespace airborne_radar {
namespace tests {

namespace {

environment::JammerEmitterState MakeJammerEmitter(environment::JammingTechnique technique,
                                                  float power_db) {
  environment::JammerEmitterState jammer;
  jammer.technique = technique;
  jammer.power_db = power_db;
  jammer.confidence = 1.0f;
  return jammer;
}

environment::EnvironmentModelConfig MakeEnvironmentConfigWithJammers(
    std::initializer_list<environment::JammerEmitterState> jammer_sources) {
  environment::EnvironmentModelConfig config;
  config.jammer_sources.insert(config.jammer_sources.end(), jammer_sources.begin(),
                               jammer_sources.end());
  return config;
}

}  // namespace

TEST(EnvironmentServiceTest, DetectsJammingByConfiguredThreshold) {
  environment::JammerEmitterState jammer_source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 7.0f);

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({jammer_source}));
  service.SetJammingSensitivityProfile(environment::ResolveJammingSensitivityProfile(6.0f));

  const auto snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 7.0f);
  EXPECT_GE(snapshot.jammer_sources[0].frequency_overlap_ratio, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
  EXPECT_GE(snapshot.jammer_sources[0].prf_lock_risk, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].prf_lock_risk, 1.0f);
}

TEST(EnvironmentServiceTest, KeepsDirectionUnknownWhenExternalDirectionIsMissing) {
  environment::JammerEmitterState jammer_source =
      MakeJammerEmitter(environment::JammingTechnique::kDeception, 10.0f);
  jammer_source.js_db = 6.0f;
  jammer_source.angular_span_deg = 15.0f;
  jammer_source.has_direction_deg = false;

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({jammer_source}));
  const auto snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FALSE(snapshot.jammer_sources[0].has_direction_deg);
  EXPECT_GE(snapshot.jammer_sources[0].frequency_overlap_ratio, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
  EXPECT_GE(snapshot.jammer_sources[0].prf_lock_risk, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].prf_lock_risk, 1.0f);
}

TEST(EnvironmentServiceTest, ModelConfigAtmosphericPhysicsAffectsDefaultSnapshot) {
  environment::EnvironmentModelConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_physics.relative_humidity = 0.7f;

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_GT(snapshot.propagation_loss_db, 6.5f);
}

TEST(EnvironmentServiceTest, DerivesAtmosphericInputsFromObservationAndTimestamp) {
  environment::EnvironmentModelConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_physics.pressure_hpa = 950.0f;
  config.atmospheric_physics.temperature_k = 300.0f;
  config.atmospheric_physics.relative_humidity = 0.9f;
  config.atmospheric_context.has_simulation_unix_seconds = true;
  config.atmospheric_context.simulation_unix_seconds =
      1704067200;  // 2024-01-01 00:00:00 UTC, DOY=1

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_GT(snapshot.effective_k_factor, 1.0f);
  EXPECT_LT(snapshot.effective_k_factor, 2.0f);
  EXPECT_EQ(snapshot.effective_day_of_year, 1);
}

TEST(EnvironmentServiceTest, FallsBackToDefaultDayOfYearWithoutTimestamp) {
  environment::EnvironmentModelConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_context.has_simulation_unix_seconds = false;
  config.atmospheric_context.simulation_unix_seconds = 946684800;  // 2000-01-01

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_EQ(snapshot.effective_day_of_year, 172);
}

TEST(EnvironmentServiceTest, FreezesSnapshotUntilNextCycle) {
  environment::EnvironmentService service;

  environment::JammerEmitterState emitter;
  emitter.technique = environment::JammingTechnique::kNoiseSuppression;
  emitter.power_db = 8.0f;
  emitter.confidence = 1.0f;
  emitter.has_direction_deg = true;
  emitter.azimuth_deg = 18.0f;
  emitter.elevation_deg = 1.0f;
  emitter.angular_span_deg = 10.0f;
  const environment::EnvironmentSceneState scene_state =
      environment::EnvironmentSceneBuilder().AddJammer(emitter).Build();

  service.UpdateSceneState(scene_state);

  const environment::EnvironmentSnapshot pending_snapshot = service.SampleEnvironment();
  EXPECT_FALSE(pending_snapshot.jamming_detected);
  EXPECT_NEAR(pending_snapshot.propagation_loss_db, 6.5f, 1e-6f);

  {
    environment::EnvironmentCycleContext ctx;
    ctx.cycle_index = 1U;
    ctx.dt_sec = 1.0f;
    service.BeginCycle(ctx);
  }
  const environment::EnvironmentSnapshot cycle_snapshot = service.SampleEnvironment();
  const environment::EnvironmentSnapshot repeated_snapshot = service.SampleEnvironment();

  EXPECT_TRUE(cycle_snapshot.jamming_detected);
  EXPECT_FLOAT_EQ(cycle_snapshot.propagation_loss_db, 6.5f);
  EXPECT_FLOAT_EQ(cycle_snapshot.clutter_power_db, 3.0f);
  EXPECT_EQ(cycle_snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(cycle_snapshot.jammer_sources.size(), repeated_snapshot.jammer_sources.size());
  EXPECT_FLOAT_EQ(repeated_snapshot.jammer_sources[0].power_db,
                  cycle_snapshot.jammer_sources[0].power_db);
  EXPECT_EQ(repeated_snapshot.jamming_detected, cycle_snapshot.jamming_detected);
}

TEST(EnvironmentServiceTest, SupportsMultipleJammerSourcesInSnapshot) {
  environment::EnvironmentModelConfig config;

  environment::JammerEmitterState noise_source;
  noise_source.technique = environment::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 9.0f;
  noise_source.js_db = 7.0f;
  noise_source.has_direction_deg = true;
  noise_source.azimuth_deg = 24.0f;
  noise_source.elevation_deg = 7.0f;
  noise_source.angular_span_deg = 30.0f;
  noise_source.confidence = 0.9f;

  environment::JammerEmitterState deception_source;
  deception_source.technique = environment::JammingTechnique::kDeception;
  deception_source.power_db = 4.0f;
  deception_source.js_db = 5.0f;
  deception_source.has_direction_deg = true;
  deception_source.azimuth_deg = 2.0f;
  deception_source.elevation_deg = 1.0f;
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 0.8f;

  config.jammer_sources.push_back(noise_source);
  config.jammer_sources.push_back(deception_source);

  environment::EnvironmentService service(config);
  service.SetJammingSensitivityProfile(environment::ResolveJammingSensitivityProfile(6.0f));

  const auto snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 2u);
  EXPECT_TRUE(snapshot.jamming_detected);
  EXPECT_EQ(snapshot.jammer_sources[0].technique, environment::JammingTechnique::kNoiseSuppression);
  EXPECT_EQ(snapshot.jammer_sources[1].technique, environment::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 9.0f);
  EXPECT_TRUE(snapshot.jammer_sources[0].in_sidelobe);
}

TEST(EnvironmentServiceTest, AppliesPendingSceneJammerOnNextCycleOnly) {
  environment::EnvironmentService service;

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  service.UpdateSceneState(
      environment::EnvironmentSceneBuilder()
          .AddJammer(MakeJammerEmitter(environment::JammingTechnique::kUnknown, 7.0f))
          .Build());

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  environment::EnvironmentCycleContext cycle_3;
  cycle_3.cycle_index = 3U;
  cycle_3.dt_sec = 1.0f;
  service.BeginCycle(cycle_3);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources[0].technique, environment::JammingTechnique::kUnknown);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 7.0f);
}

TEST(EnvironmentServiceTest, ModelConfigVegetationScatterAffectsDefaultSnapshotClutter) {
  environment::EnvironmentModelConfig baseline_config;

  environment::EnvironmentModelConfig physics_config = baseline_config;
  physics_config.vegetation_scatter_physics.enable_physical_model = true;
  physics_config.vegetation_scatter_physics.cover_profile =
      environment::VegetationCoverProfile::kConiferousForest;

  environment::EnvironmentService baseline_service(baseline_config);
  environment::EnvironmentService physics_service(physics_config);

  const environment::EnvironmentSnapshot baseline_snapshot = baseline_service.SampleEnvironment();
  const environment::EnvironmentSnapshot physics_snapshot = physics_service.SampleEnvironment();
  EXPECT_GT(physics_snapshot.clutter_power_db, baseline_snapshot.clutter_power_db);
}

TEST(EnvironmentServiceTest, EmptyJammerSourcesProduceNoJammingFacts) {
  environment::EnvironmentService service;
  service.SetJammingSensitivityProfile(environment::ResolveJammingSensitivityProfile(0.001f));

  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jammer_sources.empty());
  EXPECT_FALSE(snapshot.jamming_detected);
}

/// @brief 仅旁瓣属性为真且功率为零的结构化输入应被保留，但不应触发干扰探测。

TEST(EnvironmentServiceTest, StructuredSidelobeFactIsPreservedWithoutDetection) {
  environment::JammerEmitterState source =
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 0.0f);
  source.has_direction_deg = true;
  source.azimuth_deg = 35.0f;
  source.elevation_deg = 8.0f;
  source.angular_span_deg = 30.0f;

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({source}));
  service.SetJammingSensitivityProfile(environment::ResolveJammingSensitivityProfile(0.001f));

  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_TRUE(snapshot.jammer_sources[0].in_sidelobe);
  EXPECT_FALSE(snapshot.jamming_detected);
}

/// @brief NormalizeEmitterState：负值 power_db 钳位到 0。

TEST(EnvironmentServiceTest, NegativeEmitterPowerIsClampedToZero) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.technique = environment::JammingTechnique::kNoiseSuppression;
  source.power_db = -10.0f;  // 负值应被钳位
  source.js_db = 3.0f;
  source.confidence = 1.0f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 0.0f);
}

/// @brief NormalizeEmitterState：负值 js_db 与 angular_span_deg 钳位到 0。

TEST(EnvironmentServiceTest, NegativeEmitterJsAndAngularSpanAreClampedToZero) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.technique = environment::JammingTechnique::kNoiseSuppression;
  source.power_db = 3.0f;
  source.js_db = -2.0f;
  source.angular_span_deg = -15.0f;
  source.confidence = 0.7f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].js_db, 0.0f);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].angular_span_deg, 0.0f);
}

/// @brief NormalizeEmitterState：派生 frequency_overlap_ratio 超过上限时钳位到 1.0。

TEST(EnvironmentServiceTest, EmitterOverlapRatioAboveOneIsClampedToOne) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.technique = environment::JammingTechnique::kDeception;
  source.power_db = 5.0f;
  source.js_db = 12.0f;
  source.has_direction_deg = true;
  source.azimuth_deg = 0.0f;
  source.elevation_deg = 0.0f;
  source.angular_span_deg = 0.0f;
  source.confidence = 0.8f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
}

/// @brief NormalizeEmitterState：confidence > 1.0 钳位到 1.0。

TEST(EnvironmentServiceTest, EmitterConfidenceAboveOneIsClampedToOne) {
  environment::EnvironmentModelConfig config;
  environment::JammerEmitterState source;
  source.power_db = 5.0f;
  source.confidence = 2.5f;  // 超出范围
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].confidence, 1.0f);
}

/// @brief 干扰功率恰好等于检测门限时，应判定为探测到干扰（>= 门限）。

TEST(EnvironmentServiceTest, JammingDetectedWhenPowerEqualsThreshold) {
  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers(
      {MakeJammerEmitter(environment::JammingTechnique::kUnknown, 6.0f)}));
  service.SetJammingSensitivityProfile(environment::JammingSensitivityProfile::kBalanced);

  EXPECT_TRUE(service.SampleEnvironment().jamming_detected);
}

/// @brief 干扰功率低于检测门限时，不判定为探测到干扰。

TEST(EnvironmentServiceTest, JammingNotDetectedWhenPowerBelowThreshold) {
  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers(
      {MakeJammerEmitter(environment::JammingTechnique::kUnknown, 5.9f)}));
  service.SetJammingSensitivityProfile(environment::JammingSensitivityProfile::kBalanced);

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);
}

/// @brief 多干扰源输入应完整保留到快照中。

TEST(EnvironmentServiceTest, KeepsAllJammerSourcesInSnapshot) {
  environment::EnvironmentModelConfig config;

  environment::JammerEmitterState low;
  low.power_db = 3.0f;
  low.technique = environment::JammingTechnique::kNoiseSuppression;
  low.confidence = 1.0f;

  environment::JammerEmitterState high;
  high.power_db = 12.0f;
  high.technique = environment::JammingTechnique::kDeception;
  high.confidence = 1.0f;

  config.jammer_sources.push_back(low);
  config.jammer_sources.push_back(high);

  environment::EnvironmentService service(config);
  service.SetJammingSensitivityProfile(environment::ResolveJammingSensitivityProfile(2.0f));

  const environment::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 2u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 3.0f);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[1].power_db, 12.0f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

// ============================================================================
// TrackFilter — 衰减语义正确性测试
// ============================================================================

/// @brief 检测成功时，速度和 RCS 保持不变。

TEST(SceneManagerTest, CommitsPendingSceneOnlyWhenBeginCycleArrives) {
  environment::EnvironmentSceneState initial_scene;
  initial_scene.jammer_emitters.push_back(
      MakeJammerEmitter(environment::JammingTechnique::kUnknown, 2.0f));

  environment::SceneManager scene_manager(initial_scene);

  environment::EnvironmentSceneState pending_scene = initial_scene;
  pending_scene.jammer_emitters[0].power_db = 15.0f;
  scene_manager.UpdatePendingScene(pending_scene);

  ASSERT_EQ(scene_manager.GetActiveScene().jammer_emitters.size(), 1U);
  ASSERT_EQ(scene_manager.GetPendingScene().jammer_emitters.size(), 1U);
  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().jammer_emitters[0].power_db, 2.0f);
  EXPECT_FLOAT_EQ(scene_manager.GetPendingScene().jammer_emitters[0].power_db, 15.0f);

  environment::EnvironmentCycleContext cycle_9;
  cycle_9.cycle_index = 9U;
  cycle_9.dt_sec = 1.0f;
  scene_manager.CommitPendingScene(cycle_9);

  ASSERT_EQ(scene_manager.GetActiveScene().jammer_emitters.size(), 1U);
  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().jammer_emitters[0].power_db, 15.0f);
  EXPECT_EQ(scene_manager.GetActiveCycleContext().cycle_index, 9U);
}

}  // namespace tests
}  // namespace airborne_radar
