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

TEST(ArRuntimePatchMapperTest, MissionDomainDoesNotAffectSensorEnabled) {
  // COMMON-OQ-4 收敛：电源状态仅由 has_sensor_enabled 叶子控制；
  // mission 域在类型层面已无 power_on 字段（字段提升）。
  RuntimeConfigState current_state;
  current_state.execution_config.sensor_enabled = true;

  ArMissionConfig mission_patch;
  mission_patch.orientation.work_mode = config::ArWorkMode::kTas;

  ArRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = mission_patch;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.next_state.execution_config.sensor_enabled)
      << "has_mission must not change power state";
  EXPECT_EQ(resolved.next_state.execution_config.detection.orientation.work_mode,
            config::ArWorkMode::kTas);
}

TEST(ArRuntimePatchMapperTest, SensorEnabledLeafRemainsSolePowerControl) {
  RuntimeConfigState current_state;
  current_state.execution_config.sensor_enabled = true;

  ArRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FALSE(resolved.next_state.execution_config.sensor_enabled);
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

TEST(ArRuntimePatchMapperTest, DesignationPatchUpdatesSessionStateOnly) {
  RuntimeConfigState current_state;
  current_state.designated_external_target_id = 0U;

  ArRuntimeConfigPatch patch;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = 9001U;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  // 指定目标是会话级状态：不触发 pipeline 执行配置同步。
  EXPECT_FALSE(resolved.execution_config_changed);
  EXPECT_FALSE(resolved.environment_scenario_config_changed);
  EXPECT_EQ(resolved.next_state.designated_external_target_id, 9001U);
}

TEST(ArRuntimePatchMapperTest, DesignationPatchZeroClearsDesignation) {
  RuntimeConfigState current_state;
  current_state.designated_external_target_id = 9001U;

  ArRuntimeConfigPatch patch;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = 0U;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.next_state.designated_external_target_id, 0U);
}

// 限时指定指令：时长字段入会话状态（不触发 pipeline 同步）；任一指定相关
// 字段变更（含仅改时长）都视为新指令——生命周期阶段重置为 kNone、窗口截止
// 清零，捕获窗口在指令生效后首个周期重新起算。
TEST(ArRuntimePatchMapperTest, DesignationDurationStoredAndResetsLifecyclePhase) {
  RuntimeConfigState current_state;
  current_state.designated_external_target_id = 9001U;
  current_state.designation_duration_cycles = 5U;
  current_state.designation_phase = DesignationPhase::kAcquired;
  current_state.designation_deadline_cycle_index = 42U;

  // 仅改时长 → 阶段/截止重置，ID 与执行配置不受影响。
  ArRuntimeConfigPatch duration_patch;
  duration_patch.has_designation_duration_cycles = true;
  duration_patch.designation_duration_cycles = 7U;
  const RuntimeConfigResolveResult duration_resolved =
      ApplyRuntimePatch(current_state, duration_patch);
  ASSERT_TRUE(duration_resolved.is_valid);
  EXPECT_TRUE(duration_resolved.has_requested_update);
  EXPECT_FALSE(duration_resolved.execution_config_changed)
      << "指定/时长是会话级状态，不触发 pipeline 同步";
  EXPECT_EQ(duration_resolved.next_state.designation_duration_cycles, 7U);
  EXPECT_EQ(duration_resolved.next_state.designated_external_target_id, 9001U);
  EXPECT_EQ(duration_resolved.next_state.designation_phase, DesignationPhase::kNone);
  EXPECT_EQ(duration_resolved.next_state.designation_deadline_cycle_index, 0U);

  // 新指定（换 ID）→ 阶段重置，时长保持（未在 patch 中变更）。
  ArRuntimeConfigPatch target_patch;
  target_patch.has_designated_target_id = true;
  target_patch.designated_external_target_id = 9002U;
  const RuntimeConfigResolveResult target_resolved =
      ApplyRuntimePatch(current_state, target_patch);
  ASSERT_TRUE(target_resolved.is_valid);
  EXPECT_EQ(target_resolved.next_state.designated_external_target_id, 9002U);
  EXPECT_EQ(target_resolved.next_state.designation_duration_cycles, 5U);
  EXPECT_EQ(target_resolved.next_state.designation_phase, DesignationPhase::kNone);
  EXPECT_EQ(target_resolved.next_state.designation_deadline_cycle_index, 0U);

  // 清除指定 → 阶段重置。
  ArRuntimeConfigPatch clear_patch;
  clear_patch.has_designated_target_id = true;
  clear_patch.designated_external_target_id = 0U;
  const RuntimeConfigResolveResult clear_resolved =
      ApplyRuntimePatch(current_state, clear_patch);
  ASSERT_TRUE(clear_resolved.is_valid);
  EXPECT_EQ(clear_resolved.next_state.designated_external_target_id, 0U);
  EXPECT_EQ(clear_resolved.next_state.designation_phase, DesignationPhase::kNone);
}

TEST(ArRuntimePatchMapperTest, DesignationCombinesAtomicallyWithInvalidAnglePatch) {
  // 指定目标与非法角度字段同包：整包原子拒绝，指定不残留。
  RuntimeConfigState current_state;
  current_state.designated_external_target_id = 0U;

  ArRuntimeConfigPatch patch;
  patch.has_designated_target_id = true;
  patch.designated_external_target_id = 9001U;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = std::numeric_limits<float>::quiet_NaN();
  patch.scan_center_deg.el_deg = 0.0f;

  const RuntimeConfigResolveResult resolved = ApplyRuntimePatch(current_state, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_EQ(resolved.next_state.designated_external_target_id, 0U)
      << "原子拒绝不得残留指定目标状态";
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
  original.hardware.signal_processing.target_processing_gain_db = 4.0f;
  original.hardware.signal_processing.noise_processing_gain_db = 1.5f;
  original.hardware.signal_processing.clutter_suppression_gain_db = 12.0f;
  original.hardware.signal_processing.jamming_suppression_gain_db = 8.0f;

  const execution::InternalExecutionConfig mapped = mapping::MapSessionToExecution(original);
  const config::ArSessionConfig round_tripped = mapping::MapExecutionToSession(mapped);

  EXPECT_FLOAT_EQ(round_tripped.policy.detection.minimum_detection_margin_db, -3.0f);
  EXPECT_EQ(round_tripped.policy.detection.pulse_count, 20);
  EXPECT_FLOAT_EQ(round_tripped.mission.orientation.scan_center_deg.az_deg, 45.0f);
  EXPECT_FLOAT_EQ(round_tripped.mission.orientation.scan_center_deg.el_deg, 10.0f);
  EXPECT_FLOAT_EQ(round_tripped.policy.tracking.speed_decay_ratio_on_loss, 0.9f);
  EXPECT_FLOAT_EQ(round_tripped.policy.association.distance_gate_sigma, std::sqrt(12.0f));
  EXPECT_FLOAT_EQ(round_tripped.hardware.signal_processing.target_processing_gain_db, 4.0f);
  EXPECT_FLOAT_EQ(round_tripped.hardware.signal_processing.noise_processing_gain_db, 1.5f);
  EXPECT_FLOAT_EQ(round_tripped.hardware.signal_processing.clutter_suppression_gain_db, 12.0f);
  EXPECT_FLOAT_EQ(round_tripped.hardware.signal_processing.jamming_suppression_gain_db, 8.0f);
}

}  // namespace
}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar
