#include <gtest/gtest.h>

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "sbirs_sensor/runtime/SbirsRuntimeConfigResolver.h"

namespace {

TEST(SbirsRuntimeConfigResolverTest, MissionDomainPreservesExistingPowerState) {
  // COMMON-OQ-4 字段提升：电源状态仅由 has_sensor_enabled 叶子控制；
  // mission 域在类型层面已无电源字段，整块域全量拷贝不影响 sensor_enabled。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.sensor_enabled = false;
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kWideSearch;

  sbirs_sensor::config::SbirsMissionConfig mission_patch;
  mission_patch.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  const sbirs_sensor::config::SbirsRuntimeConfigPatch patch =
      sbirs_sensor::config::SbirsRuntimeConfigBuilder().WithMission(mission_patch).Build();

  const auto resolved = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FALSE(resolved.resolved_config.sensor_enabled)
      << "has_mission must not change power state";
  EXPECT_EQ(resolved.resolved_config.mission.work_mode,
            sbirs_sensor::config::SbirsWorkMode::kSearchAndStare);
}

TEST(SbirsRuntimeConfigResolverTest, SensorEnabledLeafRemainsSolePowerControl) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.sensor_enabled = true;

  const sbirs_sensor::config::SbirsRuntimeConfigPatch patch =
      sbirs_sensor::config::SbirsRuntimeConfigBuilder().WithSensorEnabled(false).Build();

  const auto resolved = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FALSE(resolved.resolved_config.sensor_enabled);
  EXPECT_TRUE(resolved.impact.clear_for_inactive)
      << "power-off transition must classify as inactive";
}

TEST(SbirsRuntimeConfigResolverTest, SameValueMissionPatchIsValidWithoutMigration) {
  const sbirs_sensor::config::SbirsSessionConfig config;
  const sbirs_sensor::config::SbirsRuntimeConfigPatch patch =
      sbirs_sensor::config::SbirsRuntimeConfigBuilder().WithMission(config.mission).Build();

  const auto resolved = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.impact.scan_sector_changed);
  EXPECT_FALSE(resolved.impact.reset_measurement_random_stream);
  EXPECT_FALSE(resolved.impact.reset_nis_gate_counts);
  EXPECT_FALSE(resolved.impact.reset_nfov_gate_failure_counts);
  EXPECT_FALSE(resolved.impact.restart_pointing_disturbance);
  EXPECT_FALSE(resolved.impact.release_estimated_tracks);
  EXPECT_FALSE(resolved.impact.release_incompatible_tracks);
  EXPECT_FALSE(resolved.impact.retag_truth_tracks);
  EXPECT_FALSE(resolved.impact.nfov_channel_count_changed);
  EXPECT_FALSE(resolved.impact.clear_for_inactive);
  EXPECT_FALSE(resolved.impact.clear_for_wide_search);
}

TEST(SbirsRuntimeConfigResolverTest, SeparatesTruthRetagFromEstimatedFamilyTransition) {
  sbirs_sensor::config::SbirsSessionConfig strict;
  strict.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kStrictTruthAssisted;
  sbirs_sensor::config::SbirsSessionConfig sensor_like = strict;
  sensor_like.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kSensorLikeTruthAssisted;

  const auto truth_patch = sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                               .WithPolicy(sensor_like.policy)
                               .Build();
  const auto truth =
      sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(strict, truth_patch);
  ASSERT_TRUE(truth.is_valid);
  EXPECT_TRUE(truth.impact.retag_truth_tracks);
  EXPECT_FALSE(truth.impact.release_incompatible_tracks);
  EXPECT_FALSE(truth.impact.release_estimated_tracks);

  sbirs_sensor::config::SbirsSessionConfig estimated = sensor_like;
  estimated.policy.tracking.tracking_mode =
      sbirs_sensor::config::SbirsTrackingMode::kEstimated;
  const auto estimated_patch = sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                                   .WithPolicy(estimated.policy)
                                   .Build();
  const auto family =
      sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(sensor_like, estimated_patch);
  ASSERT_TRUE(family.is_valid);
  EXPECT_FALSE(family.impact.retag_truth_tracks);
  EXPECT_TRUE(family.impact.release_incompatible_tracks);
  EXPECT_FALSE(family.impact.release_estimated_tracks);
}

TEST(SbirsRuntimeConfigResolverTest, ClassifiesIndependentStateMigrationGroups) {
  sbirs_sensor::config::SbirsSessionConfig current;
  sbirs_sensor::config::SbirsSessionConfig next = current;
  next.mission.scan_start_az_deg = 170.0f;
  next.mission.scan_span_deg = 40.0f;
  next.mission.scan_direction = sbirs_sensor::config::SbirsScanDirection::kDecreasingAzimuth;
  next.mission.narrow_field_fov_az_deg = 3.0f;
  next.policy.error_model.random_seed = 9U;
  next.policy.error_model.attitude_sigma_deg = 0.02f;
  next.policy.pointing_disturbance.random_seed = 10U;
  next.policy.tracking.process_noise_diff_coeff = 2.0f;
  next.policy.tracking.estimated_backend =
      sbirs_sensor::config::SbirsEstimatedTrackingBackend::kImm;
  next.policy.scheduler.max_concurrent_nfov_locks = 2;

  const auto patch = sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                         .WithMission(next.mission)
                         .WithPolicy(next.policy)
                         .Build();
  const auto resolved = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(current, patch);

  ASSERT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.impact.scan_sector_changed);
  EXPECT_TRUE(resolved.impact.reset_measurement_random_stream);
  EXPECT_TRUE(resolved.impact.reset_nis_gate_counts);
  EXPECT_TRUE(resolved.impact.reset_nfov_gate_failure_counts);
  EXPECT_TRUE(resolved.impact.restart_pointing_disturbance);
  EXPECT_TRUE(resolved.impact.release_estimated_tracks);
  EXPECT_TRUE(resolved.impact.nfov_channel_count_changed);
  EXPECT_EQ(resolved.impact.previous_nfov_channel_count, 1);
  EXPECT_EQ(resolved.impact.next_nfov_channel_count, 2);
}

TEST(SbirsRuntimeConfigResolverTest, EnvironmentAndScanRatePreserveAccumulatedState) {
  sbirs_sensor::config::SbirsSessionConfig current;
  sbirs_sensor::config::SbirsRuntimeConfigPatch patch;
  patch.has_environment = true;
  patch.environment = current.environment;
  patch.environment.visibility_km = 8.0f;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 25.0f;

  const auto resolved = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(current, patch);

  ASSERT_TRUE(resolved.is_valid);
  EXPECT_FALSE(resolved.impact.scan_sector_changed);
  EXPECT_FALSE(resolved.impact.reset_measurement_random_stream);
  EXPECT_FALSE(resolved.impact.reset_nis_gate_counts);
  EXPECT_FALSE(resolved.impact.reset_nfov_gate_failure_counts);
  EXPECT_FALSE(resolved.impact.restart_pointing_disturbance);
  EXPECT_FALSE(resolved.impact.release_estimated_tracks);
  EXPECT_FALSE(resolved.impact.nfov_channel_count_changed);
}

TEST(SbirsRuntimeConfigResolverTest, ElevationRasterChangeClassifiesAsScanSector) {
  // 阶段 4：任一 el 栅格字段变化须触发 scan_sector_changed（ApplyConfig 据此重锚行）。
  sbirs_sensor::config::SbirsSessionConfig current;
  sbirs_sensor::config::SbirsSessionConfig next = current;
  next.mission.scan_el_start_deg = -5.0f;
  next.mission.scan_el_span_deg = 30.0f;
  next.mission.scan_el_step_deg = 10.0f;

  const auto patch = sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                         .WithMission(next.mission)
                         .Build();
  const auto resolved = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(current, patch);
  ASSERT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.impact.scan_sector_changed);

  // 单独改 step 同样触发。
  sbirs_sensor::config::SbirsSessionConfig step_only = current;
  step_only.mission.scan_el_step_deg = 4.0f;
  const auto step_patch = sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                              .WithMission(step_only.mission)
                              .Build();
  const auto step_resolved =
      sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(current, step_patch);
  ASSERT_TRUE(step_resolved.is_valid);
  EXPECT_TRUE(step_resolved.impact.scan_sector_changed);

  // 未变栅格 → 不触发。
  const auto no_change = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(
      current, sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                   .WithMission(current.mission)
                   .Build());
  ASSERT_TRUE(no_change.is_valid);
  EXPECT_FALSE(no_change.impact.scan_sector_changed);
}

TEST(SbirsRuntimeConfigResolverTest, WorkModeTransitionsClassifyOnlyEntryCleanup) {
  const sbirs_sensor::config::SbirsSessionConfig current;
  const auto wide_patch = sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                              .WithWorkMode(sbirs_sensor::config::SbirsWorkMode::kWideSearch)
                              .Build();
  const auto wide = sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(current, wide_patch);
  ASSERT_TRUE(wide.is_valid);
  EXPECT_TRUE(wide.impact.clear_for_wide_search);
  EXPECT_FALSE(wide.impact.clear_for_inactive);

  const auto standby_patch = sbirs_sensor::config::SbirsRuntimeConfigBuilder()
                                 .WithWorkMode(sbirs_sensor::config::SbirsWorkMode::kStandby)
                                 .Build();
  const auto standby =
      sbirs_sensor::runtime::ResolveSbirsRuntimeConfigPatch(current, standby_patch);
  ASSERT_TRUE(standby.is_valid);
  EXPECT_TRUE(standby.impact.clear_for_inactive);
  EXPECT_FALSE(standby.impact.clear_for_wide_search);
}

}  // namespace
