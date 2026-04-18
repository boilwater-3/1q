// Copyright 2026. All Rights Reserved.
//
// @file signal_component_factory_test.cpp
// @brief 验证 SignalComponentFactory 的后端成套装配行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/PipelineConfig.h"
#include "airborne_radar/signal/pipeline/config/InternalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/assembly/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/SrifPredictor.h"
#include "airborne_radar/signal/tracking/SrifUpdater.h"
#include "airborne_radar/signal/tracking/UdkfPredictor.h"
#include "airborne_radar/signal/tracking/UdkfUpdater.h"

namespace airborne_radar {
namespace tests {

TEST(SignalComponentFactoryTest, BuildOwnedComponentsSelectsPredictorUpdaterFamilyByBackend) {
  config::PipelineConfig config;
  config.tracking.enable_tracking_filter = true;

  signal::pipeline::internal::InternalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalPipelineConfig(config);
  internal_config.tracking_runtime.engineering.kalman_measurement_noise_std = 5.0f;

  internal_config.tracking_runtime.engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kUdKf;
  signal::pipeline::assembly::internal::OwnedSignalComponents ud_components =
      signal::pipeline::assembly::internal::SignalComponentFactory::BuildOwnedPipelineComponents(
          config, internal_config);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfPredictor*>(ud_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfUpdater*>(ud_components.kalman_updater.get()),
            nullptr);

  internal_config.tracking_runtime.engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kSrif;
  signal::pipeline::assembly::internal::OwnedSignalComponents srif_components =
      signal::pipeline::assembly::internal::SignalComponentFactory::BuildOwnedPipelineComponents(
          config, internal_config);
  EXPECT_NE(dynamic_cast<signal::tracking::SrifPredictor*>(srif_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::SrifUpdater*>(srif_components.kalman_updater.get()),
            nullptr);

  internal_config.tracking_runtime.engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kStandardKfJoseph;
  signal::pipeline::assembly::internal::OwnedSignalComponents standard_components =
      signal::pipeline::assembly::internal::SignalComponentFactory::BuildOwnedPipelineComponents(
          config, internal_config);
  EXPECT_NE(dynamic_cast<signal::tracking::KalmanPredictor*>(
                standard_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::KalmanUpdater*>(standard_components.kalman_updater.get()),
            nullptr);
}

TEST(SignalComponentFactoryTest, ImmAssemblyUsesSamePredictorUpdaterBackendFamily) {
  config::PipelineConfig config;
  config.tracking.enable_tracking_filter = true;
  config.lifecycle.enable_imm_fusion = true;

  signal::pipeline::internal::InternalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalPipelineConfig(config);
  internal_config.tracking_runtime.engineering.kalman_update_backend =
      config::engineering::KalmanUpdateBackend::kUdKf;
  internal_config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};

  signal::pipeline::assembly::internal::LifecycleAssemblyArtifacts artifacts =
      signal::pipeline::assembly::internal::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
          config, internal_config);

  ASSERT_EQ(artifacts.imm_predictors_owned.size(), 2U);
  ASSERT_EQ(artifacts.imm_updaters_owned.size(), 2U);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfPredictor*>(
                artifacts.imm_predictors_owned[0].get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfUpdater*>(artifacts.imm_updaters_owned[0].get()),
            nullptr);
}

TEST(SignalComponentFactoryTest, InvalidImmAssemblyDoesNotFallbackToNonImmManager) {
  config::PipelineConfig config;
  config.tracking.enable_tracking_filter = true;
  config.lifecycle.enable_imm_fusion = true;

  signal::pipeline::internal::InternalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalPipelineConfig(config);
  internal_config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};
  internal_config.lifecycle.imm_transition_probability = {1.0f, 0.0f, 0.0f};

  signal::pipeline::assembly::internal::LifecycleAssemblyArtifacts artifacts =
      signal::pipeline::assembly::internal::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
          config, internal_config);

  EXPECT_EQ(artifacts.lifecycle_manager, nullptr);
  EXPECT_TRUE(artifacts.kalman_predictor == nullptr);
  EXPECT_TRUE(artifacts.kalman_updater == nullptr);
}

}  // namespace tests
}  // namespace airborne_radar
