// Copyright 2026. All Rights Reserved.
//
// @file signal_component_factory_test.cpp
// @brief 验证 SignalComponentFactory 的后端成套装配行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/InternalSignalPipelineConfig.h"
#include "airborne_radar/signal/runtime/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/SrifPredictor.h"
#include "airborne_radar/signal/tracking/SrifUpdater.h"
#include "airborne_radar/signal/tracking/UdkfPredictor.h"
#include "airborne_radar/signal/tracking/UdkfUpdater.h"

namespace airborne_radar {
namespace tests {

TEST(SignalComponentFactoryTest, BuildOwnedComponentsSelectsPredictorUpdaterFamilyByBackend) {
  common::config::SignalPipelineConfig config;
  config.tracking.enable_kalman_filter = true;
  config.tracking.kalman_measurement_noise_std = 5.0f;

  signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(config);

  config.tracking.kalman_update_backend = common::config::KalmanUpdateBackend::kUdKf;
  signal::runtime::internal::OwnedSignalComponents ud_components =
      signal::runtime::internal::SignalComponentFactory::BuildOwnedPipelineComponents(
          config, internal_config);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfPredictor*>(ud_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfUpdater*>(ud_components.kalman_updater.get()),
            nullptr);

  config.tracking.kalman_update_backend = common::config::KalmanUpdateBackend::kSrif;
  signal::runtime::internal::OwnedSignalComponents srif_components =
      signal::runtime::internal::SignalComponentFactory::BuildOwnedPipelineComponents(
          config, internal_config);
  EXPECT_NE(dynamic_cast<signal::tracking::SrifPredictor*>(srif_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::SrifUpdater*>(srif_components.kalman_updater.get()),
            nullptr);

  config.tracking.kalman_update_backend = common::config::KalmanUpdateBackend::kStandardKfJoseph;
  signal::runtime::internal::OwnedSignalComponents standard_components =
      signal::runtime::internal::SignalComponentFactory::BuildOwnedPipelineComponents(
          config, internal_config);
  EXPECT_NE(dynamic_cast<signal::tracking::KalmanPredictor*>(
                standard_components.kalman_predictor.get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::KalmanUpdater*>(standard_components.kalman_updater.get()),
            nullptr);
}

TEST(SignalComponentFactoryTest, ImmAssemblyUsesSamePredictorUpdaterBackendFamily) {
  common::config::SignalPipelineConfig config;
  config.tracking.enable_kalman_filter = true;
  config.tracking.kalman_update_backend = common::config::KalmanUpdateBackend::kUdKf;
  config.lifecycle.enable_imm_lifecycle = true;

  signal::pipeline::internal::InternalSignalPipelineConfig internal_config =
      signal::pipeline::internal::BuildInternalSignalPipelineConfig(config);
  internal_config.lifecycle.imm_model_noise_diff_coeffs = {0.5f, 2.0f};

  signal::runtime::internal::LifecycleAssemblyArtifacts artifacts =
      signal::runtime::internal::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
          config, internal_config);

  ASSERT_EQ(artifacts.imm_predictors_owned.size(), 2U);
  ASSERT_EQ(artifacts.imm_updaters_owned.size(), 2U);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfPredictor*>(
                artifacts.imm_predictors_owned[0].get()),
            nullptr);
  EXPECT_NE(dynamic_cast<signal::tracking::UdkfUpdater*>(artifacts.imm_updaters_owned[0].get()),
            nullptr);
}

}  // namespace tests
}  // namespace airborne_radar
