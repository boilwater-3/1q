/**
 * @file esr_runtime_config_resolver_test.cpp
 * @brief ESR 运行期补丁解析器测试（高层语义补丁）。
 */

#include <gtest/gtest.h>

#include <limits>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"

namespace electronic_surveillance_radar {
namespace session {
namespace internal {
namespace {

namespace esr_config = ::electronic_surveillance_radar::config;

TEST(EsrRuntimeConfigResolverTest, ValidPatchUpdatesRuntimePipelineAndEnvironment) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.scan.scan_rate_hz = 1.0f;

  config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.66f;

  const config::EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithScanRateHz(4.0f)
                                          .WithWorkMode(esr_config::EsrWorkMode::kRwr)
                                          .WithAtmosphericPhysicsConfig(atmospheric_physics)
                                          .Build();

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(resolved.runtime_config_changed);
  EXPECT_TRUE(resolved.pipeline_config_changed);
  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan.scan_rate_hz, 4.0f);
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(
      resolved.next_config.environment.atmospheric_physics.relative_humidity, 0.66f);
}

TEST(EsrRuntimeConfigResolverTest, AtmosphericPhysicsOnlyDoesNotOverridePreset) {
  EsrInternalExecutionConfig current_config;
  current_config.environment.preset = config::EsrEnvironmentPreset::kDenseClutter;

  config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.pressure_hpa = 950.0f;
  atmospheric_physics.temperature_k = 295.0f;
  atmospheric_physics.relative_humidity = 0.81f;

  const config::EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithAtmosphericPhysicsConfig(atmospheric_physics)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_EQ(resolved.next_config.environment.preset,
            config::EsrEnvironmentPreset::kDenseClutter);
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.atmospheric_physics.pressure_hpa, 950.0f);
}

TEST(EsrRuntimeConfigResolverTest, MultiEnvironmentSubdomainsCanBeUpdatedInSinglePatch) {
  EsrInternalExecutionConfig current_config;
  current_config.environment.preset = config::EsrEnvironmentPreset::kStandard;

  config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.9f;

  const config::EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithAtmosphericPhysicsConfig(atmospheric_physics)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_EQ(resolved.next_config.environment.preset,
            config::EsrEnvironmentPreset::kStandard);
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.atmospheric_physics.relative_humidity,
                  0.9f);
}

TEST(EsrRuntimeConfigResolverTest, MissionDomainPatchUpdatesMissionAndResolvedScan) {
  EsrInternalExecutionConfig current_config;
  current_config.hardware.az_scan_range_deg = 80.0f;
  current_config.hardware.el_scan_range_deg = 20.0f;
  current_config.hardware.antenna_mount_az_deg = 3.0f;
  current_config.hardware.antenna_mount_el_deg = -2.0f;
  current_config.mission.power_on = false;
  current_config.mission.work_mode = config::EsrWorkMode::kEsm;
  current_config.mission.scan.scan_rate_hz = 1.0f;
  current_config.mission.scan.scan_start_position = config::EsrScanStartPosition::kLeftTop;
  current_config.mission.scan.scan_sequence = config::EsrScanSequence::kAzimuthFirst;
  current_config.detection.pulse_count = 8U;
  current_config.detection.threshold_scale = 1.0f;

  config::EsrMissionConfig mission;
  mission.power_on = true;
  mission.work_mode = config::EsrWorkMode::kHgesm;
  mission.scan.scan_rate_hz = 6.0f;
  mission.scan.scan_start_position = config::EsrScanStartPosition::kRightBottom;
  mission.scan.scan_sequence = config::EsrScanSequence::kElevationFirst;
  mission.scan.scan_center_az_deg = 20.0f;
  mission.scan.scan_center_el_deg = 4.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = mission;

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(resolved.runtime_config_changed);
  EXPECT_TRUE(resolved.pipeline_config_changed);
  EXPECT_TRUE(resolved.next_config.mission.power_on);
  EXPECT_EQ(resolved.next_config.mission.work_mode, config::EsrWorkMode::kHgesm);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan.scan_rate_hz, 6.0f);
  EXPECT_EQ(resolved.next_config.resolved_scan.scan_start_pos,
            static_cast<int>(config::EsrScanStartPosition::kRightBottom));
  EXPECT_EQ(resolved.next_config.resolved_scan.scan_sequence,
            static_cast<int>(config::EsrScanSequence::kElevationFirst));
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_az_deg, -23.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_az_deg, 57.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_el_deg, -4.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_el_deg, 16.0f);
  EXPECT_EQ(resolved.next_config.detection.pulse_count, 32U);
  EXPECT_FLOAT_EQ(resolved.next_config.detection.threshold_scale, 0.85f);
}

TEST(EsrRuntimeConfigResolverTest, MissionDomainPatchRejectsInvalidScanRateAtomically) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.power_on = true;
  current_config.mission.scan.scan_rate_hz = 2.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = current_config.mission;
  patch.mission.power_on = false;
  patch.mission.scan.scan_rate_hz = std::numeric_limits<float>::infinity();

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidScanRate);
  EXPECT_TRUE(resolved.next_config.mission.power_on);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan.scan_rate_hz, 2.0f);
}

TEST(EsrRuntimeConfigResolverTest, MissionDomainRejectsInvalidExplicitBoundsAtomically) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.power_on = true;
  current_config.mission.scan.use_explicit_scan_bounds = false;
  current_config.resolved_scan.scan_start_az_deg = -30.0f;
  current_config.resolved_scan.scan_end_az_deg = 30.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = current_config.mission;
  patch.mission.power_on = false;
  patch.mission.scan.use_explicit_scan_bounds = true;
  patch.mission.scan.scan_start_az_deg = 5.0f;
  patch.mission.scan.scan_end_az_deg = 5.0f;

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_FALSE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
  EXPECT_TRUE(resolved.next_config.mission.power_on);
  EXPECT_FALSE(resolved.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_az_deg, -30.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_az_deg, 30.0f);
}

TEST(EsrRuntimeConfigResolverTest, MissionDomainRejectsInvalidCenterAtomically) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.power_on = true;
  current_config.mission.scan.scan_center_az_deg = 2.0f;
  current_config.resolved_scan.scan_start_az_deg = -30.0f;
  current_config.resolved_scan.scan_end_az_deg = 30.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = current_config.mission;
  patch.mission.power_on = false;
  patch.mission.scan.use_explicit_scan_bounds = false;
  patch.mission.scan.scan_center_az_deg = std::numeric_limits<float>::quiet_NaN();

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_FALSE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidScanCenterAz);
  EXPECT_TRUE(resolved.next_config.mission.power_on);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan.scan_center_az_deg, 2.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_az_deg, -30.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_az_deg, 30.0f);
}

TEST(EsrRuntimeConfigResolverTest, LeafOverridesAreValidatedAfterInvalidMissionScanValues) {
  EsrInternalExecutionConfig current_config;
  current_config.hardware.az_scan_range_deg = 80.0f;
  current_config.hardware.el_scan_range_deg = 20.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = current_config.mission;
  patch.mission.scan.scan_rate_hz = std::numeric_limits<float>::infinity();
  patch.mission.scan.use_explicit_scan_bounds = true;
  patch.mission.scan.scan_start_az_deg = 10.0f;
  patch.mission.scan.scan_end_az_deg = -10.0f;
  patch.has_scan_rate_hz = true;
  patch.scan_rate_hz = 3.0f;
  patch.has_explicit_scan_bounds = true;
  patch.explicit_scan_bounds.enabled = false;

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan.scan_rate_hz, 3.0f);
  EXPECT_FALSE(resolved.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_az_deg, -40.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_az_deg, 40.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_el_deg, -10.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_el_deg, 10.0f);
}

TEST(EsrRuntimeConfigResolverTest, MissionAndPolicyPatchAppliesWorkModeAfterPolicy) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.work_mode = config::EsrWorkMode::kEsm;
  current_config.detection.pulse_count = 8U;
  current_config.detection.threshold_scale = 1.0f;

  config::EsrMissionConfig mission;
  mission.work_mode = config::EsrWorkMode::kRwr;
  mission.scan.scan_rate_hz = 2.0f;

  config::EsrPolicyConfig policy;
  policy.detection.pulse_count = 9U;
  policy.detection.threshold_scale = 1.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = mission;
  patch.has_policy = true;
  patch.policy = policy;

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.next_config.mission.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_EQ(resolved.next_config.detection.pulse_count, 4U);
  EXPECT_FLOAT_EQ(resolved.next_config.detection.threshold_scale, 1.25f);
}

TEST(EsrRuntimeConfigResolverTest, InvalidExplicitBoundsRejectWholePatch) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.scan.scan_rate_hz = 1.0f;

  config::EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder().WithScanRateHz(3.0f).Build();
  patch.has_explicit_scan_bounds = true;
  patch.explicit_scan_bounds.enabled = true;




  patch.explicit_scan_bounds.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  patch.explicit_scan_bounds.scan_end_az_deg = 10.0f;
  patch.explicit_scan_bounds.scan_start_el_deg = -10.0f;
  patch.explicit_scan_bounds.scan_end_el_deg = 10.0f;

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
  EXPECT_FALSE(resolved.runtime_config_changed);
  EXPECT_FALSE(resolved.pipeline_config_changed);
  EXPECT_FALSE(resolved.environment_model_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan.scan_rate_hz, 1.0f);
}

TEST(EsrRuntimeConfigResolverTest, EqualOrSwappedExplicitBoundsRejectWholePatch) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.scan.use_explicit_scan_bounds = false;
  current_config.resolved_scan.scan_start_az_deg = -30.0f;
  current_config.resolved_scan.scan_end_az_deg = 30.0f;

  config::EsrRuntimeConfigPatch swapped_az_patch;
  swapped_az_patch.has_explicit_scan_bounds = true;
  swapped_az_patch.explicit_scan_bounds.enabled = true;
  swapped_az_patch.explicit_scan_bounds.scan_start_az_deg = 10.0f;
  swapped_az_patch.explicit_scan_bounds.scan_end_az_deg = -10.0f;
  swapped_az_patch.explicit_scan_bounds.scan_start_el_deg = -5.0f;
  swapped_az_patch.explicit_scan_bounds.scan_end_el_deg = 5.0f;

  const EsrRuntimeConfigResolveResult swapped_az =
      ResolveEsrRuntimeConfigPatch(current_config, swapped_az_patch);
  EXPECT_FALSE(swapped_az.is_valid);
  EXPECT_EQ(swapped_az.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
  EXPECT_FALSE(swapped_az.pipeline_config_changed);
  EXPECT_FALSE(swapped_az.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(swapped_az.next_config.resolved_scan.scan_start_az_deg, -30.0f);
  EXPECT_FLOAT_EQ(swapped_az.next_config.resolved_scan.scan_end_az_deg, 30.0f);

  config::EsrRuntimeConfigPatch equal_el_patch = swapped_az_patch;
  equal_el_patch.explicit_scan_bounds.scan_start_az_deg = -10.0f;
  equal_el_patch.explicit_scan_bounds.scan_end_az_deg = 10.0f;
  equal_el_patch.explicit_scan_bounds.scan_start_el_deg = 5.0f;
  equal_el_patch.explicit_scan_bounds.scan_end_el_deg = 5.0f;

  const EsrRuntimeConfigResolveResult equal_el =
      ResolveEsrRuntimeConfigPatch(current_config, equal_el_patch);
  EXPECT_FALSE(equal_el.is_valid);
  EXPECT_EQ(equal_el.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
  EXPECT_FALSE(equal_el.pipeline_config_changed);
  EXPECT_FALSE(equal_el.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(equal_el.next_config.resolved_scan.scan_start_az_deg, -30.0f);
  EXPECT_FLOAT_EQ(equal_el.next_config.resolved_scan.scan_end_az_deg, 30.0f);
}

TEST(EsrRuntimeConfigResolverTest, DisableExplicitBoundsRebuildsCenterDrivenWindow) {
  EsrInternalExecutionConfig current_config;
  current_config.hardware.az_scan_range_deg = 80.0f;
  current_config.hardware.el_scan_range_deg = 20.0f;
  current_config.hardware.antenna_mount_az_deg = 3.0f;
  current_config.hardware.antenna_mount_el_deg = -2.0f;
  current_config.mission.scan.use_explicit_scan_bounds = true;
  current_config.mission.scan.scan_center_az_deg = 20.0f;
  current_config.mission.scan.scan_center_el_deg = 4.0f;
  current_config.mission.scan.scan_start_az_deg = -10.0f;
  current_config.mission.scan.scan_end_az_deg = 10.0f;
  current_config.mission.scan.scan_start_el_deg = -5.0f;
  current_config.mission.scan.scan_end_el_deg = 5.0f;
  current_config.resolved_scan.scan_start_az_deg = -10.0f;
  current_config.resolved_scan.scan_end_az_deg = 10.0f;
  current_config.resolved_scan.scan_start_el_deg = -5.0f;
  current_config.resolved_scan.scan_end_el_deg = 5.0f;

  const config::EsrRuntimeConfigPatch patch =
      esr_config::EsrRuntimeConfigBuilder().WithExplicitScanBoundsEnabled(false).Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(resolved.pipeline_config_changed);
  EXPECT_FALSE(resolved.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_az_deg, -23.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_az_deg, 57.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_el_deg, -4.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_el_deg, 16.0f);
}

TEST(EsrRuntimeConfigResolverTest, EnableExplicitBoundsUsesStaticMountConvention) {
  EsrInternalExecutionConfig current_config;
  current_config.hardware.antenna_mount_az_deg = 3.0f;
  current_config.hardware.antenna_mount_el_deg = -2.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_explicit_scan_bounds = true;
  patch.explicit_scan_bounds.enabled = true;
  patch.explicit_scan_bounds.scan_start_az_deg = -10.0f;
  patch.explicit_scan_bounds.scan_end_az_deg = 30.0f;
  patch.explicit_scan_bounds.scan_start_el_deg = -5.0f;
  patch.explicit_scan_bounds.scan_end_el_deg = 7.0f;

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.pipeline_config_changed);
  EXPECT_TRUE(resolved.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_az_deg, -13.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_az_deg, 27.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_el_deg, -3.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_el_deg, 9.0f);
}

TEST(EsrRuntimeConfigResolverTest, DisableExplicitBoundsIgnoresInactiveNonFinitePayload) {
  EsrInternalExecutionConfig current_config;
  current_config.hardware.az_scan_range_deg = 60.0f;
  current_config.hardware.el_scan_range_deg = 30.0f;
  current_config.mission.scan.use_explicit_scan_bounds = true;
  current_config.mission.scan.scan_center_az_deg = 5.0f;
  current_config.mission.scan.scan_center_el_deg = -3.0f;

  config::EsrRuntimeConfigPatch patch;
  patch.has_explicit_scan_bounds = true;
  patch.explicit_scan_bounds.enabled = false;
  patch.explicit_scan_bounds.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  patch.explicit_scan_bounds.scan_end_az_deg = std::numeric_limits<float>::infinity();
  patch.explicit_scan_bounds.scan_start_el_deg = -std::numeric_limits<float>::infinity();
  patch.explicit_scan_bounds.scan_end_el_deg = std::numeric_limits<float>::quiet_NaN();

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(resolved.pipeline_config_changed);
  EXPECT_FALSE(resolved.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_az_deg, -25.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_az_deg, 35.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_start_el_deg, -18.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.resolved_scan.scan_end_el_deg, 12.0f);
}

TEST(EsrRuntimeConfigResolverTest, DisableExplicitBoundsRejectsNonFiniteCenterAtomically) {
  EsrInternalExecutionConfig current_config;
  current_config.mission.scan.use_explicit_scan_bounds = true;
  current_config.mission.scan.scan_center_az_deg = std::numeric_limits<float>::quiet_NaN();
  current_config.mission.scan.scan_center_el_deg = 2.0f;
  current_config.resolved_scan.scan_start_az_deg = -10.0f;
  current_config.resolved_scan.scan_end_az_deg = 10.0f;

  const config::EsrRuntimeConfigPatch disable_patch =
      esr_config::EsrRuntimeConfigBuilder().WithExplicitScanBoundsEnabled(false).Build();
  const EsrRuntimeConfigResolveResult invalid_az =
      ResolveEsrRuntimeConfigPatch(current_config, disable_patch);

  EXPECT_FALSE(invalid_az.is_valid);
  EXPECT_EQ(invalid_az.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidScanCenterAz);
  EXPECT_FALSE(invalid_az.pipeline_config_changed);
  EXPECT_TRUE(invalid_az.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(invalid_az.next_config.resolved_scan.scan_start_az_deg, -10.0f);
  EXPECT_FLOAT_EQ(invalid_az.next_config.resolved_scan.scan_end_az_deg, 10.0f);

  current_config.mission.scan.scan_center_az_deg = 1.0f;
  current_config.mission.scan.scan_center_el_deg = std::numeric_limits<float>::infinity();
  const EsrRuntimeConfigResolveResult invalid_el =
      ResolveEsrRuntimeConfigPatch(current_config, disable_patch);

  EXPECT_FALSE(invalid_el.is_valid);
  EXPECT_EQ(invalid_el.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidScanCenterEl);
  EXPECT_FALSE(invalid_el.pipeline_config_changed);
  EXPECT_TRUE(invalid_el.next_config.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(invalid_el.next_config.resolved_scan.scan_start_az_deg, -10.0f);
  EXPECT_FLOAT_EQ(invalid_el.next_config.resolved_scan.scan_end_az_deg, 10.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electronic_surveillance_radar
