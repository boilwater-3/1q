// Copyright 2026. All Rights Reserved.
//
// @file ar_runtime_patch_mapper_test.cpp
// @brief 验证 AR 运行期补丁映射器的合并优先级、原子拒绝语义与反向映射。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "airborne_radar/config/mapping/RuntimePatchMapper.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"

namespace airborne_radar {
namespace config {
namespace mapping {
namespace {

TEST(ArRuntimePatchMapperTest, MissionDomainAppliedBeforeLeafPatch) {
  RuntimeConfigState current_state;
  current_state.execution_config.detection.orientation.scan_center_deg.az_deg = 1.0f;
  current_state.execution_config.detection.orientation.scan_center_deg.el_deg = 2.0f;
  current_state.execution_config.detection.orientation.work_mode =
      config::ArWorkMode::kTws;

  ArMissionConfig mission_patch;
  mission_patch.orientation.scan_center_deg.az_deg = 10.0f;
  mission_patch.orientation.scan_center_deg.el_deg = 20.0f;
  mission_patch.orientation.work_mode = config::ArWorkMode::kTas;

  ArRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = mission_patch;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = 30.0f;
  patch.scan_center_deg.el_deg = 40.0f;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.execution_config_changed);
  EXPECT_EQ(resolved.next_state.execution_config.detection.orientation.work_mode,
            config::ArWorkMode::kTas);
  EXPECT_FLOAT_EQ(resolved.next_state.execution_config.detection.orientation.scan_center_deg.az_deg,
                  30.0f);
  EXPECT_FLOAT_EQ(resolved.next_state.execution_config.detection.orientation.scan_center_deg.el_deg,
                  40.0f);
}

TEST(ArRuntimePatchMapperTest, DwellPatchContributesToPipelinePointing) {
  RuntimeConfigState current_state;
  current_state.execution_config.detection.orientation.scan_center_deg.az_deg = 10.0f;
  current_state.execution_config.detection.orientation.scan_center_deg.el_deg = 2.0f;

  config::AzimuthElevationDeg dwell_center;
  dwell_center.az_deg = 3.0f;
  dwell_center.el_deg = -1.0f;
  const ArRuntimeConfigPatch patch =
      ArRuntimeConfigBuilder().WithDwellCenterDeg(dwell_center).Build();

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);
  const config::ArSessionConfig pipeline_config =
      MapRuntimeStateToPipelineSession(resolved.next_state);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.execution_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_state.dwell_center_deg.az_deg, 3.0f);
  EXPECT_FLOAT_EQ(resolved.next_state.dwell_center_deg.el_deg, -1.0f);
  EXPECT_FLOAT_EQ(pipeline_config.mission.orientation.scan_center_deg.az_deg, 13.0f);
  EXPECT_FLOAT_EQ(pipeline_config.mission.orientation.scan_center_deg.el_deg, 1.0f);
  EXPECT_FLOAT_EQ(resolved.next_state.execution_config.detection.orientation.scan_center_deg.az_deg,
                  10.0f);
  EXPECT_FLOAT_EQ(resolved.next_state.execution_config.detection.orientation.scan_center_deg.el_deg,
                  2.0f);
}

TEST(ArRuntimePatchMapperTest, EnvironmentPatchUpdatesNaturalModel) {
  RuntimeConfigState current_state;
  current_state.environment_scenario_config.atmospheric_physics.enable_physical_model = false;

  ArRuntimeConfigPatch patch =
      ArRuntimeConfigBuilder()
          .WithEnvironmentScenarioConfig(config::EnvironmentScenarioConfig{})
          .Build();
  patch.environment.scenario_config.atmospheric_physics.enable_physical_model = true;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.environment_scenario_config_changed);
  EXPECT_TRUE(
      resolved.next_state.environment_scenario_config.atmospheric_physics.enable_physical_model);
}

TEST(ArRuntimePatchMapperTest, InvalidPatchIsRejectedAtomically) {
  RuntimeConfigState current_state;
  current_state.execution_config.detection.orientation.scan_center_deg.az_deg = 1.0f;
  current_state.execution_config.detection.orientation.scan_center_deg.el_deg = 2.0f;

  ArRuntimeConfigPatch patch =
      ArRuntimeConfigBuilder()
          .WithEnvironmentScenarioConfig(config::EnvironmentScenarioConfig{})
          .Build();
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = std::numeric_limits<float>::quiet_NaN();
  patch.scan_center_deg.el_deg = 0.0f;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.execution_config_changed);
  EXPECT_FALSE(resolved.environment_scenario_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_state.execution_config.detection.orientation.scan_center_deg.az_deg,
                  1.0f);
}

TEST(ArRuntimePatchMapperTest, EnabledNonPositiveBeamwidthIsRejectedAtomically) {
  RuntimeConfigState current_state;
  current_state.execution_config.detection.orientation.work_mode = config::ArWorkMode::kTws;

  ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::ArWorkMode::kStt;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_commanded_beamwidth_deg = true;
  patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 0.0f;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.execution_config_changed);
  EXPECT_EQ(resolved.next_state.execution_config.detection.orientation.work_mode,
            config::ArWorkMode::kTws);
  EXPECT_FALSE(
      resolved.next_state.execution_config.detection.orientation.commanded_beamwidth_enabled);
}

TEST(ArRuntimePatchMapperTest, MapExecutionToSessionRoundTripsFields) {
  config::ArSessionConfig original;
  original.policy.detection.minimum_detection_margin_db = -3.0f;
  original.policy.detection.pulse_count = 20;
  original.mission.orientation.scan_center_deg.az_deg = 45.0f;
  original.mission.orientation.scan_center_deg.el_deg = 10.0f;
  original.policy.tracking.speed_decay_ratio_on_loss = 0.9f;
  original.policy.association.distance_gate_sigma = std::sqrt(12.0f);

  const execution::InternalExecutionConfig mapped = mapping::MapSessionToExecution(original);
  const config::ArSessionConfig round_tripped = mapping::MapExecutionToSession(mapped);

  EXPECT_FLOAT_EQ(round_tripped.policy.detection.minimum_detection_margin_db, -3.0f);
  EXPECT_EQ(round_tripped.policy.detection.pulse_count, 20);
  EXPECT_FLOAT_EQ(round_tripped.mission.orientation.scan_center_deg.az_deg, 45.0f);
  EXPECT_FLOAT_EQ(round_tripped.mission.orientation.scan_center_deg.el_deg, 10.0f);
  EXPECT_FLOAT_EQ(round_tripped.policy.tracking.speed_decay_ratio_on_loss, 0.9f);
  EXPECT_FLOAT_EQ(round_tripped.policy.association.distance_gate_sigma, std::sqrt(12.0f));
}

}  // namespace
}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar
