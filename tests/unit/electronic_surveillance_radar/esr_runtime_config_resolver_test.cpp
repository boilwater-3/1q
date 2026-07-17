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

TEST(EsrRuntimeConfigResolverTest, AtmosphericPhysicsOnlyDoesNotOverridePresetOrContext) {
  EsrInternalExecutionConfig current_config;
  current_config.environment.preset = config::EsrEnvironmentPreset::kDenseClutter;
  current_config.environment.atmospheric_context.has_k_factor = true;
  current_config.environment.atmospheric_context.k_factor = 1.45f;

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
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.atmospheric_context.k_factor, 1.45f);
}

TEST(EsrRuntimeConfigResolverTest, AtmosphericContextOnlyDoesNotOverridePresetOrPhysics) {
  EsrInternalExecutionConfig current_config;
  current_config.environment.preset = config::EsrEnvironmentPreset::kLowClutter;
  current_config.environment.atmospheric_physics.enable_physical_model = true;
  current_config.environment.atmospheric_physics.relative_humidity = 0.42f;

  config::EsrAtmosphericDerivedContext atmospheric_context;
  atmospheric_context.has_day_of_year = true;
  atmospheric_context.day_of_year = 245;
  atmospheric_context.solar_flux_f107 = 180.0f;

  const config::EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithAtmosphericContext(atmospheric_context)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_EQ(resolved.next_config.environment.preset,
            config::EsrEnvironmentPreset::kLowClutter);
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.atmospheric_physics.relative_humidity,
                  0.42f);
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_context.has_day_of_year);
  EXPECT_EQ(resolved.next_config.environment.atmospheric_context.day_of_year, 245);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.atmospheric_context.solar_flux_f107,
                  180.0f);
}

TEST(EsrRuntimeConfigResolverTest, MultiEnvironmentSubdomainsCanBeUpdatedInSinglePatch) {
  EsrInternalExecutionConfig current_config;
  current_config.environment.preset = config::EsrEnvironmentPreset::kStandard;

  config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.9f;
  config::EsrAtmosphericDerivedContext atmospheric_context;
  atmospheric_context.has_k_factor = true;
  atmospheric_context.k_factor = 1.37f;

  const config::EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithAtmosphericPhysicsConfig(atmospheric_physics)
                                          .WithAtmosphericContext(atmospheric_context)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_EQ(resolved.next_config.environment.preset,
            config::EsrEnvironmentPreset::kStandard);
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.atmospheric_physics.relative_humidity,
                  0.9f);
  EXPECT_TRUE(resolved.next_config.environment.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.atmospheric_context.k_factor, 1.37f);
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

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electronic_surveillance_radar
