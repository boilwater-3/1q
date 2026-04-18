// Copyright 2026. All Rights Reserved.
//
// @file ar_runtime_config_resolver_test.cpp
// @brief 验证 AR 运行期配置解析器的合并优先级与原子拒绝语义。

#include <gtest/gtest.h>

#include <limits>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "airborne_radar/session/RuntimeConfigResolver.h"

namespace airborne_radar {
namespace session {
namespace internal {
namespace {

TEST(ArRuntimeConfigResolverTest, FullSignalConfigAppliedBeforeLeafPatch) {
  RuntimeConfigState current_state;
  current_state.signal_pipeline_config.beam_control.radar_orientation.scan_center_deg.az_deg = 1.0f;
  current_state.signal_pipeline_config.beam_control.radar_orientation.scan_center_deg.el_deg = 2.0f;
  current_state.signal_pipeline_config.beam_control.radar_orientation.work_sub_mode =
      model::RadarWorkSubMode::kTws;

  config::SignalPipelineConfig full_signal_config = current_state.signal_pipeline_config;
  full_signal_config.beam_control.radar_orientation.scan_center_deg.az_deg = 10.0f;
  full_signal_config.beam_control.radar_orientation.scan_center_deg.el_deg = 20.0f;
  full_signal_config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTas;

  config::RadarRuntimeConfigPatch patch;
  patch.has_signal_pipeline_config = true;
  patch.signal_pipeline_config = full_signal_config;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = 30.0f;
  patch.scan_center_deg.el_deg = 40.0f;

  const RuntimeConfigResolveResult resolved = ResolveRuntimeConfigPatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.signal_pipeline_config_changed);
  EXPECT_EQ(resolved.next_state.signal_pipeline_config.beam_control.radar_orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
  EXPECT_FLOAT_EQ(
      resolved.next_state.signal_pipeline_config.beam_control.radar_orientation.scan_center_deg.az_deg,
      30.0f);
  EXPECT_FLOAT_EQ(
      resolved.next_state.signal_pipeline_config.beam_control.radar_orientation.scan_center_deg.el_deg,
      40.0f);
}

TEST(ArRuntimeConfigResolverTest, EnvironmentPatchUpdatesModelAndThreshold) {
  RuntimeConfigState current_state;
  current_state.environment_scenario_config.atmospheric_physics.enable_physical_model = false;
    current_state.jamming_sensitivity_profile = environment::JammingSensitivityProfile::kBalanced;

  config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithEnvironmentScenarioConfig(environment::EnvironmentScenarioConfig{})
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();
  patch.environment_runtime_config.scenario_config.atmospheric_physics.enable_physical_model = true;

  const RuntimeConfigResolveResult resolved = ResolveRuntimeConfigPatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.environment_scenario_config_changed);
  EXPECT_TRUE(resolved.jamming_sensitivity_profile_changed);
  EXPECT_TRUE(
      resolved.next_state.environment_scenario_config.atmospheric_physics.enable_physical_model);
    EXPECT_EQ(resolved.next_state.jamming_sensitivity_profile,
                        environment::JammingSensitivityProfile::kStrict);
}

TEST(ArRuntimeConfigResolverTest, InvalidPatchIsRejectedAtomically) {
  RuntimeConfigState current_state;
  current_state.signal_pipeline_config.beam_control.radar_orientation.scan_center_deg.az_deg = 1.0f;
  current_state.signal_pipeline_config.beam_control.radar_orientation.scan_center_deg.el_deg = 2.0f;
    current_state.jamming_sensitivity_profile = environment::JammingSensitivityProfile::kBalanced;

  config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = std::numeric_limits<float>::quiet_NaN();
  patch.scan_center_deg.el_deg = 0.0f;

  const RuntimeConfigResolveResult resolved = ResolveRuntimeConfigPatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.signal_pipeline_config_changed);
  EXPECT_FALSE(resolved.environment_scenario_config_changed);
  EXPECT_FALSE(resolved.jamming_sensitivity_profile_changed);
  EXPECT_FLOAT_EQ(
      resolved.next_state.signal_pipeline_config.beam_control.radar_orientation.scan_center_deg.az_deg,
      1.0f);
    EXPECT_EQ(resolved.next_state.jamming_sensitivity_profile,
                        environment::JammingSensitivityProfile::kBalanced);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace airborne_radar
