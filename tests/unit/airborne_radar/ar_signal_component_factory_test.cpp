// Copyright 2026. All Rights Reserved.
//
// @file signal_component_factory_test.cpp
// @brief 验证 SignalComponentFactory 的 KF 装配与 IMM 组装行为。

#include <gtest/gtest.h>

#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace tests {

using ExecutionConfig = config::execution::InternalExecutionConfig;

TEST(SignalComponentFactoryTest, BuildOwnedComponentsCreatesStandardKalmanComponents) {
  ExecutionConfig config;
  config.tracking.policy.enable_kalman_filter = true;
  config.tracking.engineering.enable_kalman_filter = true;
  config.tracking.engineering.kalman_measurement_noise_std = 5.0f;

  signal::pipeline::OwnedSignalComponents components =
      signal::pipeline::SignalComponentFactory::BuildOwnedPipelineComponents(config);

  EXPECT_NE(dynamic_cast<signal::tracking::KalmanPredictor*>(components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::KalmanUpdater*>(components.kalman_updater.get()),
            nullptr);
  EXPECT_NE(components.signal_detector, nullptr);
}

TEST(SignalComponentFactoryTest, ImmAssemblyCreatesKalmanSubModels) {
  ExecutionConfig config;
  config.tracking.policy.enable_kalman_filter = true;
  config.lifecycle.policy.enable_imm_lifecycle = true;
  config.tracking.engineering.enable_kalman_filter = true;
  config.lifecycle.engineering.enable_imm_lifecycle = true;
  config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};

  signal::pipeline::LifecycleAssemblyArtifacts artifacts =
      signal::pipeline::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(config);

  ASSERT_EQ(artifacts.imm_predictors_owned.size(), 2U);
  ASSERT_EQ(artifacts.imm_updaters_owned.size(), 2U);
  EXPECT_NE(
      dynamic_cast<signal::tracking::KalmanPredictor*>(artifacts.imm_predictors_owned[0].get()),
      nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::KalmanUpdater*>(artifacts.imm_updaters_owned[0].get()),
            nullptr);
}

TEST(SignalComponentFactoryTest, InvalidImmAssemblyDoesNotFallbackToNonImmManager) {
  ExecutionConfig config;
  config.tracking.policy.enable_kalman_filter = true;
  config.lifecycle.policy.enable_imm_lifecycle = true;
  config.tracking.engineering.enable_kalman_filter = true;
  config.lifecycle.engineering.enable_imm_lifecycle = true;
  config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  config.lifecycle.imm_transition_probability = {1.0f, 0.0f, 0.0f};

  signal::pipeline::LifecycleAssemblyArtifacts artifacts =
      signal::pipeline::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(config);

  EXPECT_EQ(artifacts.lifecycle_manager, nullptr);
  EXPECT_TRUE(artifacts.kalman_predictor == nullptr);
  EXPECT_TRUE(artifacts.kalman_updater == nullptr);
}

}  // namespace tests
}  // namespace airborne_radar
