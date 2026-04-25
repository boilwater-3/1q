// Copyright 2026. All Rights Reserved.
//
// @file signal_component_factory_test.cpp
// @brief 验证 SignalComponentFactory 的后端成套装配行为。

#include <gtest/gtest.h>

#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/SrifPredictor.h"
#include "airborne_radar/signal/tracking/SrifUpdater.h"
#include "airborne_radar/signal/tracking/UdkfPredictor.h"
#include "airborne_radar/signal/tracking/UdkfUpdater.h"

namespace airborne_radar {
namespace tests {

using ExecutionConfig = config::execution::InternalExecutionConfig;

TEST(SignalComponentFactoryTest, BuildOwnedComponentsSelectsPredictorUpdaterFamilyByBackend) {
  ExecutionConfig config;
  config.policy_tracking.enable_kalman_filter = true;
    config.tracking_engineering.enable_kalman_filter = true;
  config.tracking_engineering.kalman_measurement_noise_std = 5.0f;

  config.tracking_engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kUdKf;
  signal::pipeline::OwnedSignalComponents ud_components =
      signal::pipeline::SignalComponentFactory::BuildOwnedPipelineComponents(
          config);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfPredictor*>(ud_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfUpdater*>(ud_components.kalman_updater.get()),
            nullptr);

  config.tracking_engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kSrif;
  signal::pipeline::OwnedSignalComponents srif_components =
      signal::pipeline::SignalComponentFactory::BuildOwnedPipelineComponents(
          config);
  EXPECT_NE(dynamic_cast<signal::tracking::SrifPredictor*>(srif_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::SrifUpdater*>(srif_components.kalman_updater.get()),
            nullptr);

  config.tracking_engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kStandardKfJoseph;
  signal::pipeline::OwnedSignalComponents standard_components =
      signal::pipeline::SignalComponentFactory::BuildOwnedPipelineComponents(
          config);
  EXPECT_NE(
      dynamic_cast<signal::tracking::KalmanPredictor*>(standard_components.kalman_predictor.get()),
      nullptr);
  EXPECT_NE(
      dynamic_cast<signal::tracking::KalmanUpdater*>(standard_components.kalman_updater.get()),
      nullptr);
}

TEST(SignalComponentFactoryTest, ImmAssemblyUsesSamePredictorUpdaterBackendFamily) {
  ExecutionConfig config;
  config.policy_tracking.enable_kalman_filter = true;
  config.policy_lifecycle.enable_imm_lifecycle = true;
    config.tracking_engineering.enable_kalman_filter = true;
    config.lifecycle_engineering.enable_imm_lifecycle = true;
  config.tracking_engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kUdKf;
  config.imm_model_noise_diff_coeffs = {0.5f, 2.0f};

  signal::pipeline::LifecycleAssemblyArtifacts artifacts =
      signal::pipeline::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
          config);

  ASSERT_EQ(artifacts.imm_predictors_owned.size(), 2U);
  ASSERT_EQ(artifacts.imm_updaters_owned.size(), 2U);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfPredictor*>(artifacts.imm_predictors_owned[0].get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfUpdater*>(artifacts.imm_updaters_owned[0].get()),
            nullptr);
}

TEST(SignalComponentFactoryTest, InvalidImmAssemblyDoesNotFallbackToNonImmManager) {
  ExecutionConfig config;
  config.policy_tracking.enable_kalman_filter = true;
  config.policy_lifecycle.enable_imm_lifecycle = true;
    config.tracking_engineering.enable_kalman_filter = true;
    config.lifecycle_engineering.enable_imm_lifecycle = true;
  config.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  config.imm_transition_probability = {1.0f, 0.0f, 0.0f};

  signal::pipeline::LifecycleAssemblyArtifacts artifacts =
      signal::pipeline::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
          config);

  EXPECT_EQ(artifacts.lifecycle_manager, nullptr);
  EXPECT_TRUE(artifacts.kalman_predictor == nullptr);
  EXPECT_TRUE(artifacts.kalman_updater == nullptr);
}

}  // namespace tests
}  // namespace airborne_radar
