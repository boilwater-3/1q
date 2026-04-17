/**
 * @file esr_environment_service_test.cpp
 * @brief 验证 ESR 环境服务的干扰技术兼容推断与分层聚合行为。
 */

#include <gtest/gtest.h>

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"

namespace electronic_surveillance_radar {
namespace environment {
namespace {

TEST(EsrEnvironmentServiceTest, UnknownTechniqueWithPositiveRiskInfersMixedAndKeepsBothChannels) {
  EsrEnvironmentService service;

  EsrEnvironmentCycleContext context;
  context.cycle_index = 1U;
  context.dt_sec = 1.0f;
  EsrJammerSource jammer;
  jammer.technique = EsrJammingTechnique::kUnknown;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 2.0e9;
  jammer.power_w = 10.0f;
  jammer.deception_risk = 0.4f;
  jammer.confidence = 0.5f;
  context.observation.jammer_sources.push_back(jammer);

  service.BeginCycle(context);
  const EsrEnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources.front().technique, EsrJammingTechnique::kMixed);
  EXPECT_NEAR(snapshot.suppression_power_w, 5.0f, 1.0e-6f);
  EXPECT_NEAR(snapshot.deception_risk, 0.2f, 1.0e-6f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

TEST(EsrEnvironmentServiceTest, UnknownTechniqueWithZeroRiskInfersSuppressionOnly) {
  EsrEnvironmentService service;

  EsrEnvironmentCycleContext context;
  context.cycle_index = 2U;
  context.dt_sec = 1.0f;
  EsrJammerSource jammer;
  jammer.technique = EsrJammingTechnique::kUnknown;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 2.0e9;
  jammer.power_w = 6.0f;
  jammer.deception_risk = 0.0f;
  jammer.confidence = 0.25f;
  context.observation.jammer_sources.push_back(jammer);

  service.BeginCycle(context);
  const EsrEnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources.front().technique, EsrJammingTechnique::kNoiseSuppression);
  EXPECT_NEAR(snapshot.suppression_power_w, 1.5f, 1.0e-6f);
  EXPECT_NEAR(snapshot.deception_risk, 0.0f, 1.0e-6f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

TEST(EsrEnvironmentServiceTest, DeceptionOnlySourceDoesNotTriggerSuppressionDetection) {
  EsrEnvironmentModelConfig config;
  config.jamming_sensitivity_policy = EsrJammingSensitivityPolicy::kRelaxed;
  EsrEnvironmentService service(config);

  EsrEnvironmentCycleContext context;
  context.cycle_index = 3U;
  context.dt_sec = 1.0f;
  EsrJammerSource jammer;
  jammer.technique = EsrJammingTechnique::kDeception;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 2.0e9;
  jammer.power_w = 100.0f;
  jammer.deception_risk = 0.8f;
  jammer.confidence = 1.0f;
  context.observation.jammer_sources.push_back(jammer);

  service.BeginCycle(context);
  const EsrEnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_NEAR(snapshot.suppression_power_w, 0.0f, 1.0e-6f);
  EXPECT_NEAR(snapshot.deception_risk, 0.8f, 1.0e-6f);
  EXPECT_FALSE(snapshot.jamming_detected);
}

TEST(EsrEnvironmentServiceTest, AtmosphericPhysicsCanIncreasePropagationLoss) {
  EsrEnvironmentService service;

  EsrEnvironmentCycleContext baseline_context;
  baseline_context.cycle_index = 5U;
  baseline_context.dt_sec = 1.0f;
  baseline_context.observation.propagation_profile =
      EsrPropagationEnvironmentProfile::kOpen;
  baseline_context.observation.atmospheric_observation.relative_humidity_ratio = 0.3f;
  baseline_context.observation.atmospheric_observation.precipitation_rate_mmph = 0.0f;

  service.BeginCycle(baseline_context);
  const EsrEnvironmentSnapshot baseline_snapshot = service.SampleEnvironment();

  EsrEnvironmentCycleContext physics_context = baseline_context;
  physics_context.cycle_index = 6U;
  physics_context.observation.atmospheric_observation.relative_humidity_ratio = 0.9f;
  physics_context.observation.atmospheric_observation.precipitation_rate_mmph = 20.0f;
  physics_context.observation.atmospheric_observation.visibility_km = 4.0f;

  service.BeginCycle(physics_context);
  const EsrEnvironmentSnapshot physics_snapshot = service.SampleEnvironment();
  EXPECT_GT(physics_snapshot.propagation_loss_db, baseline_snapshot.propagation_loss_db);
}

TEST(EsrEnvironmentServiceTest, ConfigAtmosphericPhysicsAppliesWhenSceneDoesNotOverride) {
  EsrEnvironmentModelConfig config;
  config.atmospheric_physics.enable_physical_model = true;
  config.atmospheric_physics.frequency_hz = 10.0e9f;
  config.atmospheric_physics.path_length_m = 100.0e3f;
  config.atmospheric_physics.radar_altitude_m = 2000.0f;
  config.atmospheric_physics.target_altitude_m = 1200.0f;
  config.atmospheric_physics.elevation_deg = 2.0f;
  config.atmospheric_physics.relative_humidity = 0.75f;
  EsrEnvironmentService service(config);

  EsrEnvironmentCycleContext context;
  context.cycle_index = 7U;
  context.dt_sec = 1.0f;
  context.observation.propagation_profile = EsrPropagationEnvironmentProfile::kOpen;
  context.observation.atmospheric_observation.relative_humidity_ratio = 0.75f;
  context.observation.atmospheric_observation.precipitation_rate_mmph = 10.0f;

  service.BeginCycle(context);
  const EsrEnvironmentSnapshot snapshot = service.SampleEnvironment();
  EXPECT_GT(snapshot.propagation_loss_db, 2.0f);
}

}  // namespace
}  // namespace environment
}  // namespace electronic_surveillance_radar
