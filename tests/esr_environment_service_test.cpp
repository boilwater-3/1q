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

TEST(EsrEnvironmentServiceTest,
     UnknownTechniqueWithPositiveRiskInfersMixedAndKeepsBothChannels) {
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
  context.scene_state.jammer_sources.push_back(jammer);

  service.BeginCycle(context);
  const EsrEnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources.front().technique, EsrJammingTechnique::kMixed);
  EXPECT_NEAR(snapshot.suppression_power_w, 5.0f, 1.0e-6f);
  EXPECT_NEAR(snapshot.jammer_power_w, snapshot.suppression_power_w, 1.0e-6f);
  EXPECT_NEAR(snapshot.deception_risk, 0.2f, 1.0e-6f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

TEST(EsrEnvironmentServiceTest,
     UnknownTechniqueWithZeroRiskInfersSuppressionOnly) {
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
  context.scene_state.jammer_sources.push_back(jammer);

  service.BeginCycle(context);
  const EsrEnvironmentSnapshot snapshot = service.SampleEnvironment();

  ASSERT_EQ(snapshot.jammer_sources.size(), 1U);
  EXPECT_EQ(snapshot.jammer_sources.front().technique,
            EsrJammingTechnique::kNoiseSuppression);
  EXPECT_NEAR(snapshot.suppression_power_w, 1.5f, 1.0e-6f);
  EXPECT_NEAR(snapshot.deception_risk, 0.0f, 1.0e-6f);
  EXPECT_TRUE(snapshot.jamming_detected);
}

TEST(EsrEnvironmentServiceTest,
     DeceptionOnlySourceDoesNotTriggerSuppressionDetection) {
  EsrEnvironmentModelConfig config;
  config.jamming_detection_threshold_w = 1.0e-8f;
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
  context.scene_state.jammer_sources.push_back(jammer);

  service.BeginCycle(context);
  const EsrEnvironmentSnapshot snapshot = service.SampleEnvironment();

  EXPECT_NEAR(snapshot.suppression_power_w, 0.0f, 1.0e-6f);
  EXPECT_NEAR(snapshot.jammer_power_w, 0.0f, 1.0e-6f);
  EXPECT_NEAR(snapshot.deception_risk, 0.8f, 1.0e-6f);
  EXPECT_FALSE(snapshot.jamming_detected);
}

}  // namespace
}  // namespace environment
}  // namespace electronic_surveillance_radar
