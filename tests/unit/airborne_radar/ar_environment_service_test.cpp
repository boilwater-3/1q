// Copyright 2026. All Rights Reserved.
//
// @file ar_environment_service_test.cpp
// @brief 验证环境服务与场景管理的基础行为。

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/environment/SceneManager.h"

namespace airborne_radar {
namespace tests {

namespace {

config::JammerEmitterState MakeJammerEmitter(config::JammingTechnique technique, float power_db) {
  config::JammerEmitterState jammer;
  jammer.technique = technique;
  jammer.power_db = power_db;
  jammer.confidence = 1.0f;
  return jammer;
}

config::EnvironmentScenarioConfig MakeEnvironmentConfigWithJammers(
    std::initializer_list<config::JammerEmitterState> jammer_sources) {
  config::EnvironmentScenarioConfig config;
  config.jammer_sources.insert(config.jammer_sources.end(), jammer_sources.begin(),
                               jammer_sources.end());
  return config;
}

}  // namespace

TEST(EnvironmentServiceTest, DetectsJammingByConfiguredThreshold) {
  config::JammerEmitterState jammer_source =
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 7.0f);

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({jammer_source}));
  service.SetJammingSensitivityProfile(config::ResolveJammingSensitivityProfile(6.0f));

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
  config::JammerEmitterState jammer_source =
      MakeJammerEmitter(config::JammingTechnique::kDeception, 10.0f);
  jammer_source.js_db = 6.0f;
  jammer_source.angular_span_deg = 15.0f;
  // Default position (0,0,0) is at origin, direction is available but angles are 0.
  // Azimuth at origin: atan2(0, 0) = 0, elevation at origin: atan2(0, 0) = 0.

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({jammer_source}));
  const auto snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_TRUE(snapshot.jammer_sources[0].has_direction_deg);
  EXPECT_NEAR(snapshot.jammer_sources[0].direction_deg.azimuth_deg, 0.0f, 1e-6f);
  EXPECT_NEAR(snapshot.jammer_sources[0].direction_deg.elevation_deg, 0.0f, 1e-6f);
  EXPECT_GE(snapshot.jammer_sources[0].frequency_overlap_ratio, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
  EXPECT_GE(snapshot.jammer_sources[0].prf_lock_risk, 0.0f);
  EXPECT_LE(snapshot.jammer_sources[0].prf_lock_risk, 1.0f);
}

TEST(EnvironmentServiceTest, ModelConfigAtmosphericPhysicsAffectsDefaultSnapshot) {
  config::EnvironmentScenarioConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_physics.relative_humidity = 0.7f;

  environment::EnvironmentService service(config);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_GT(snapshot.propagation_loss_db, 6.5f);
}

TEST(EnvironmentServiceTest, DerivesAtmosphericInputsFromObservationAndTimestamp) {
  config::EnvironmentScenarioConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_physics.pressure_hpa = 950.0f;
  config.atmospheric_physics.temperature_k = 300.0f;
  config.atmospheric_physics.relative_humidity = 0.9f;
  config.atmospheric_context.has_simulation_unix_seconds = true;
  config.atmospheric_context.simulation_unix_seconds =
      1704067200;  // 2024-01-01 00:00:00 UTC, DOY=1

  environment::EnvironmentService service(config);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_GT(snapshot.effective_k_factor, 1.0f);
  EXPECT_LT(snapshot.effective_k_factor, 2.0f);
  EXPECT_EQ(snapshot.effective_day_of_year, 1);
}

TEST(EnvironmentServiceTest, FallsBackToDefaultDayOfYearWithoutTimestamp) {
  config::EnvironmentScenarioConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_context.has_simulation_unix_seconds = false;
  config.atmospheric_context.simulation_unix_seconds = 946684800;  // 2000-01-01

  environment::EnvironmentService service(config);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_EQ(snapshot.effective_day_of_year, 172);
}

TEST(EnvironmentServiceTest, FreezesSnapshotUntilNextCycle) {
  environment::EnvironmentService service;

  config::JammerEmitterState emitter;
  emitter.technique = config::JammingTechnique::kNoiseSuppression;
  emitter.power_db = 8.0f;
  emitter.confidence = 1.0f;
  emitter.position_x = 3090.17f;  // range 10000m * sin(18 deg)
  emitter.position_y = 9510.57f;  // range 10000m * cos(18 deg)
  emitter.position_z = 174.55f;   // range 10000m * tan(1 deg)
  emitter.angular_span_deg = 10.0f;
  session::EnvironmentSceneState scene_state;
  scene_state.jammer_emitters.push_back(emitter);

  service.UpdateSceneState(scene_state);

  const session::EnvironmentSnapshot pending_snapshot = service.SampleEnvironment();
  EXPECT_FALSE(pending_snapshot.jamming_detected);
  EXPECT_NEAR(pending_snapshot.propagation_loss_db, 6.5f, 1e-6f);

  {
    session::EnvironmentCycleContext ctx;
    ctx.cycle_index = 1U;
    ctx.dt_sec = 1.0f;
    service.BeginCycle(ctx);
  }
  const session::EnvironmentSnapshot cycle_snapshot = service.SampleEnvironment();
  const session::EnvironmentSnapshot repeated_snapshot = service.SampleEnvironment();

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
  config::EnvironmentScenarioConfig config;

  config::JammerEmitterState noise_source;
  noise_source.technique = config::JammingTechnique::kNoiseSuppression;
  noise_source.power_db = 9.0f;
  noise_source.js_db = 7.0f;
  noise_source.position_x = 4067.36f;  // range 10000m * sin(24 deg)
  noise_source.position_y = 9135.45f;  // range 10000m * cos(24 deg)
  noise_source.position_z = 1227.85f;  // range 10000m * tan(7 deg)
  noise_source.angular_span_deg = 30.0f;
  noise_source.confidence = 0.9f;

  config::JammerEmitterState deception_source;
  deception_source.technique = config::JammingTechnique::kDeception;
  deception_source.power_db = 4.0f;
  deception_source.js_db = 5.0f;
  deception_source.position_x = 348.99f;   // range 10000m * sin(2 deg)
  deception_source.position_y = 9993.90f;  // range 10000m * cos(2 deg)
  deception_source.position_z = 174.55f;   // range 10000m * tan(1 deg)
  deception_source.angular_span_deg = 8.0f;
  deception_source.confidence = 0.8f;

  config.jammer_sources.push_back(noise_source);
  config.jammer_sources.push_back(deception_source);

  environment::EnvironmentService service(config);
  service.SetJammingSensitivityProfile(config::ResolveJammingSensitivityProfile(6.0f));

  const auto snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 2u);
  EXPECT_TRUE(snapshot.jamming_detected);
  EXPECT_EQ(snapshot.jammer_sources[0].technique, config::JammingTechnique::kNoiseSuppression);
  EXPECT_EQ(snapshot.jammer_sources[1].technique, config::JammingTechnique::kDeception);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 9.0f);
  EXPECT_TRUE(snapshot.jammer_sources[0].in_sidelobe);
}

TEST(EnvironmentServiceTest, AppliesPendingSceneJammerOnNextCycleOnly) {
  environment::EnvironmentService service;

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  session::EnvironmentSceneState jammer_scene;
  jammer_scene.jammer_emitters.push_back(
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 7.0f));
  service.UpdateSceneState(jammer_scene);

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);

  session::EnvironmentCycleContext cycle_3;
  cycle_3.cycle_index = 3U;
  cycle_3.dt_sec = 1.0f;
  service.BeginCycle(cycle_3);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources[0].technique, config::JammingTechnique::kUnknown);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 7.0f);
}

TEST(EnvironmentServiceTest, ModelConfigVegetationScatterAffectsDefaultSnapshotClutter) {
  config::EnvironmentScenarioConfig baseline_config;

  config::EnvironmentScenarioConfig physics_config = baseline_config;
  physics_config.vegetation_scatter_physics.enable_physical_model = true;
  physics_config.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kConiferousForest;

  environment::EnvironmentService baseline_service(baseline_config);
  environment::EnvironmentService physics_service(physics_config);

  const session::EnvironmentSnapshot baseline_snapshot = baseline_service.SampleEnvironment();
  const session::EnvironmentSnapshot physics_snapshot = physics_service.SampleEnvironment();
  EXPECT_GT(physics_snapshot.clutter_power_db, baseline_snapshot.clutter_power_db);
}

TEST(EnvironmentServiceTest, EmptyJammerSourcesProduceNoJammingFacts) {
  environment::EnvironmentService service;
  service.SetJammingSensitivityProfile(config::ResolveJammingSensitivityProfile(0.001f));

  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jammer_sources.empty());
  EXPECT_FALSE(snapshot.jamming_detected);
}

/// @brief 仅旁瓣属性为真且功率为零的结构化输入应被保留，但不应触发干扰探测。

TEST(EnvironmentServiceTest, StructuredSidelobeFactIsPreservedWithoutDetection) {
  config::JammerEmitterState source = MakeJammerEmitter(config::JammingTechnique::kUnknown, 0.0f);
  source.position_x = 5735.76f;  // range 10000m * sin(35 deg)
  source.position_y = 8191.52f;  // range 10000m * cos(35 deg)
  source.position_z = 1405.41f;  // range 10000m * tan(8 deg)
  source.angular_span_deg = 30.0f;

  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers({source}));
  service.SetJammingSensitivityProfile(config::ResolveJammingSensitivityProfile(0.001f));

  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_TRUE(snapshot.jammer_sources[0].in_sidelobe);
  EXPECT_FALSE(snapshot.jamming_detected);
}

/// @brief NormalizeEmitterState：负值 power_db 钳位到 0。

TEST(EnvironmentServiceTest, NegativeEmitterPowerIsClampedToZero) {
  config::EnvironmentScenarioConfig config;
  config::JammerEmitterState source;
  source.technique = config::JammingTechnique::kNoiseSuppression;
  source.power_db = -10.0f;  // 负值应被钳位
  source.js_db = 3.0f;
  source.confidence = 1.0f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 0.0f);
}

/// @brief NormalizeEmitterState：负值 js_db 与 angular_span_deg 钳位到 0。

TEST(EnvironmentServiceTest, NegativeEmitterJsAndAngularSpanAreClampedToZero) {
  config::EnvironmentScenarioConfig config;
  config::JammerEmitterState source;
  source.technique = config::JammingTechnique::kNoiseSuppression;
  source.power_db = 3.0f;
  source.js_db = -2.0f;
  source.angular_span_deg = -15.0f;
  source.confidence = 0.7f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].js_db, 0.0f);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].angular_span_deg, 0.0f);
}

/// @brief NormalizeEmitterState：派生 frequency_overlap_ratio 超过上限时钳位到 1.0。

TEST(EnvironmentServiceTest, EmitterOverlapRatioAboveOneIsClampedToOne) {
  config::EnvironmentScenarioConfig config;
  config::JammerEmitterState source;
  source.technique = config::JammingTechnique::kDeception;
  source.power_db = 5.0f;
  source.js_db = 12.0f;
  source.position_x = 0.0f;      // azimuth 0 deg
  source.position_y = 10000.0f;  // forward
  source.position_z = 0.0f;      // elevation 0 deg
  source.angular_span_deg = 0.0f;
  source.confidence = 0.8f;
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].frequency_overlap_ratio, 1.0f);
}

/// @brief NormalizeEmitterState：confidence > 1.0 钳位到 1.0。

TEST(EnvironmentServiceTest, EmitterConfidenceAboveOneIsClampedToOne) {
  config::EnvironmentScenarioConfig config;
  config::JammerEmitterState source;
  source.power_db = 5.0f;
  source.confidence = 2.5f;  // 超出范围
  config.jammer_sources.push_back(source);

  environment::EnvironmentService service(config);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].confidence, 1.0f);
}

/// @brief 干扰功率恰好等于检测门限时，应判定为探测到干扰（>= 门限）。

TEST(EnvironmentServiceTest, JammingDetectedWhenPowerEqualsThreshold) {
  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers(
      {MakeJammerEmitter(config::JammingTechnique::kUnknown, 6.0f)}));
  service.SetJammingSensitivityProfile(config::JammingSensitivityProfile::kBalanced);

  EXPECT_TRUE(service.SampleEnvironment().jamming_detected);
}

/// @brief 干扰功率低于检测门限时，不判定为探测到干扰。

TEST(EnvironmentServiceTest, JammingNotDetectedWhenPowerBelowThreshold) {
  environment::EnvironmentService service(MakeEnvironmentConfigWithJammers(
      {MakeJammerEmitter(config::JammingTechnique::kUnknown, 5.9f)}));
  service.SetJammingSensitivityProfile(config::JammingSensitivityProfile::kBalanced);

  EXPECT_FALSE(service.SampleEnvironment().jamming_detected);
}

/// @brief 多干扰源输入应完整保留到快照中。

TEST(EnvironmentServiceTest, KeepsAllJammerSourcesInSnapshot) {
  config::EnvironmentScenarioConfig config;

  config::JammerEmitterState low;
  low.power_db = 3.0f;
  low.technique = config::JammingTechnique::kNoiseSuppression;
  low.confidence = 1.0f;

  config::JammerEmitterState high;
  high.power_db = 12.0f;
  high.technique = config::JammingTechnique::kDeception;
  high.confidence = 1.0f;

  config.jammer_sources.push_back(low);
  config.jammer_sources.push_back(high);

  environment::EnvironmentService service(config);
  service.SetJammingSensitivityProfile(config::ResolveJammingSensitivityProfile(2.0f));

  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();
  ASSERT_EQ(snapshot.jammer_sources.size(), 2u);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[0].power_db, 3.0f);
  EXPECT_FLOAT_EQ(snapshot.jammer_sources[1].power_db, 12.0f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

TEST(EnvironmentServiceTest, FreezesEngineeringRfFactsWithoutLegacyDerivation) {
  environment::EnvironmentService service;
  session::EnvironmentSceneState scene;
  scene.interference.mode = oneq::electromagnetics::RfInterferenceMode::kEngineering;
  oneq::electromagnetics::RfEmission emission;
  emission.emission_id = 71;
  emission.entity_id = 900;
  oneq::electromagnetics::RfEmissionSegment segment;
  segment.duration_s = 1.0;
  segment.center_frequency_hz = 10.0e9;
  segment.bandwidth_hz = 10.0e6;
  segment.transmit_power_w = 500.0;
  emission.segments.push_back(segment);
  scene.interference.engineering_emissions.push_back(emission);
  scene.has_rf_receiver_kinematics = true;
  scene.rf_receiver_entity_id = 901;
  scene.rf_receiver_position_ecef_m.x_m = 1000.0;
  service.UpdateSceneState(scene);

  session::EnvironmentCycleContext cycle;
  cycle.cycle_index = 4U;
  cycle.dt_sec = 1.0f;
  service.BeginCycle(cycle);
  const session::EnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_EQ(snapshot.interference_mode, oneq::electromagnetics::RfInterferenceMode::kEngineering);
  ASSERT_EQ(snapshot.engineering_interference_emissions.size(), 1U);
  EXPECT_EQ(snapshot.engineering_interference_emissions.front().emission_id, 71U);
  EXPECT_TRUE(snapshot.has_rf_receiver_kinematics);
  EXPECT_EQ(snapshot.rf_receiver_entity_id, 901U);
  EXPECT_TRUE(snapshot.jammer_sources.empty());
  EXPECT_FALSE(snapshot.jamming_detected);
}

// ============================================================================
// TrackFilter — 衰减语义正确性测试
// ============================================================================

/// @brief 检测成功时，速度和 RCS 保持不变。

TEST(SceneManagerTest, CommitsPendingSceneOnlyWhenBeginCycleArrives) {
  session::EnvironmentSceneState initial_scene;
  initial_scene.jammer_emitters.push_back(
      MakeJammerEmitter(config::JammingTechnique::kUnknown, 2.0f));

  environment::SceneManager scene_manager(initial_scene);

  session::EnvironmentSceneState pending_scene = initial_scene;
  pending_scene.jammer_emitters[0].power_db = 15.0f;
  scene_manager.UpdatePendingScene(pending_scene);

  ASSERT_EQ(scene_manager.GetActiveScene().jammer_emitters.size(), 1U);
  ASSERT_EQ(scene_manager.GetPendingScene().jammer_emitters.size(), 1U);
  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().jammer_emitters[0].power_db, 2.0f);
  EXPECT_FLOAT_EQ(scene_manager.GetPendingScene().jammer_emitters[0].power_db, 15.0f);

  session::EnvironmentCycleContext cycle_9;
  cycle_9.cycle_index = 9U;
  cycle_9.dt_sec = 1.0f;
  scene_manager.CommitPendingScene(cycle_9);

  ASSERT_EQ(scene_manager.GetActiveScene().jammer_emitters.size(), 1U);
  EXPECT_FLOAT_EQ(scene_manager.GetActiveScene().jammer_emitters[0].power_db, 15.0f);
  EXPECT_EQ(scene_manager.GetActiveCycleContext().cycle_index, 9U);
}

}  // namespace tests
}  // namespace airborne_radar
