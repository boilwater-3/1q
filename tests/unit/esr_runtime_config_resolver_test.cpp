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
