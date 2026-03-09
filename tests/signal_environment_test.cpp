// Copyright 2026. All Rights Reserved.
//
// Description: 验证真实环境建模与信号处理实现的基础行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar::tests {

TEST(EnvironmentServiceTest, DetectsJammingByConfiguredThreshold) {
  environment::EnvironmentModelConfig config;
  config.jammer_power_db = 7.0f;

  environment::EnvironmentService service(config);
  service.SetJammingDetectionThresholdDb(6.0f);

  const auto snapshot = service.SampleEnvironment();
  EXPECT_TRUE(snapshot.jamming_detected);
}

TEST(SignalPipelineTest, KeepsTrackStableWhenDetectionMarginIsEnough) {
  environment::EnvironmentModelConfig env_config;
  env_config.jammer_power_db = 0.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  const common::TargetFeatureList input_state{
      common::TargetFeature(800.0f, 2.5f, false, 0.0f)};

  const auto output_state =
      signal_pipeline.RunCycle(input_state, environment_service);

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_FLOAT_EQ(output_state[0].current_track_speed,
                  input_state[0].current_track_speed);
  EXPECT_FLOAT_EQ(output_state[0].current_track_rcs,
                  input_state[0].current_track_rcs);
  EXPECT_FALSE(output_state[0].check_jamming_detected);
}

TEST(SignalPipelineTest, DegradesTrackWhenDetectionMarginIsTooLow) {
  environment::EnvironmentModelConfig env_config;
  env_config.base_propagation_loss_db = 60.0f;
  env_config.atmospheric_attenuation_db = 25.0f;
  env_config.terrain_reflection_db = 15.0f;
  env_config.clutter_power_db = 20.0f;
  env_config.jammer_power_db = 12.0f;
  environment::EnvironmentService environment_service(env_config);

  signal::pipeline::SignalPipeline signal_pipeline;
  const common::TargetFeatureList input_state{
      common::TargetFeature(800.0f, 2.5f, false, 1.0f)};

  const auto output_state =
      signal_pipeline.RunCycle(input_state, environment_service);

  ASSERT_EQ(output_state.size(), 1u);
  EXPECT_LT(output_state[0].current_track_speed, input_state[0].current_track_speed);
  EXPECT_LT(output_state[0].current_track_rcs, input_state[0].current_track_rcs);
  EXPECT_TRUE(output_state[0].check_jamming_detected);
  EXPECT_LT(output_state[0].current_track_acceleration,
            input_state[0].current_track_acceleration);
}

} // namespace airborne_radar::tests
