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
  ResolvedEsrSessionConfig current_config;
  current_config.runtime_config.scan_rate_hz = 1.0f;

  environment::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.66f;

  const EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
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
  EXPECT_FLOAT_EQ(resolved.next_config.runtime_config.scan_rate_hz, 4.0f);
  EXPECT_TRUE(resolved.next_config.environment_model_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(
      resolved.next_config.environment_model_config.atmospheric_physics.relative_humidity, 0.66f);
}

TEST(EsrRuntimeConfigResolverTest, PresetPatchIsRejected) {
  ResolvedEsrSessionConfig current_config;
  current_config.environment_model_config.preset = config::EsrEnvironmentPreset::kStandard;
  current_config.environment_model_config.atmospheric_physics.enable_physical_model = true;
  current_config.environment_model_config.atmospheric_physics.relative_humidity = 0.72f;
  current_config.environment_model_config.atmospheric_context.has_day_of_year = true;
  current_config.environment_model_config.atmospheric_context.day_of_year = 130;

  environment::EsrEnvironmentRuntimeConfigPatch environment_patch;
  environment_patch.has_preset = true;
  environment_patch.preset = esr_config::EsrEnvironmentPreset::kJammed;

  const EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithEnvironmentRuntimeConfig(environment_patch)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

    EXPECT_TRUE(resolved.has_requested_update);
    EXPECT_FALSE(resolved.is_valid);
    EXPECT_EQ(resolved.status,
        EsrRuntimeConfigApplyStatus::kRejectedUnsupportedEnvironmentPresetPatch);
    EXPECT_FALSE(resolved.environment_model_config_changed);
    EXPECT_EQ(resolved.next_config.environment_model_config.preset,
        config::EsrEnvironmentPreset::kStandard);
}

TEST(EsrRuntimeConfigResolverTest, AtmosphericPhysicsOnlyDoesNotOverridePresetOrContext) {
  ResolvedEsrSessionConfig current_config;
  current_config.environment_model_config.preset = config::EsrEnvironmentPreset::kDenseClutter;
  current_config.environment_model_config.atmospheric_context.has_k_factor = true;
  current_config.environment_model_config.atmospheric_context.k_factor = 1.45f;

  environment::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.pressure_hpa = 950.0f;
  atmospheric_physics.temperature_k = 295.0f;
  atmospheric_physics.relative_humidity = 0.81f;

  const EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithAtmosphericPhysicsConfig(atmospheric_physics)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_EQ(resolved.next_config.environment_model_config.preset,
            config::EsrEnvironmentPreset::kDenseClutter);
  EXPECT_TRUE(resolved.next_config.environment_model_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_model_config.atmospheric_physics.pressure_hpa, 950.0f);
  EXPECT_TRUE(resolved.next_config.environment_model_config.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_model_config.atmospheric_context.k_factor, 1.45f);
}

TEST(EsrRuntimeConfigResolverTest, AtmosphericContextOnlyDoesNotOverridePresetOrPhysics) {
  ResolvedEsrSessionConfig current_config;
  current_config.environment_model_config.preset = config::EsrEnvironmentPreset::kLowClutter;
  current_config.environment_model_config.atmospheric_physics.enable_physical_model = true;
  current_config.environment_model_config.atmospheric_physics.relative_humidity = 0.42f;

  environment::EsrAtmosphericDerivedContext atmospheric_context;
  atmospheric_context.has_day_of_year = true;
  atmospheric_context.day_of_year = 245;
  atmospheric_context.solar_flux_f107 = 180.0f;

  const EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithAtmosphericContext(atmospheric_context)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_EQ(resolved.next_config.environment_model_config.preset,
            config::EsrEnvironmentPreset::kLowClutter);
  EXPECT_TRUE(resolved.next_config.environment_model_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_model_config.atmospheric_physics.relative_humidity,
                  0.42f);
  EXPECT_TRUE(resolved.next_config.environment_model_config.atmospheric_context.has_day_of_year);
  EXPECT_EQ(resolved.next_config.environment_model_config.atmospheric_context.day_of_year, 245);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_model_config.atmospheric_context.solar_flux_f107,
                  180.0f);
}

TEST(EsrRuntimeConfigResolverTest, MultiEnvironmentSubdomainsCanBeUpdatedInSinglePatch) {
  ResolvedEsrSessionConfig current_config;
  current_config.environment_model_config.preset = config::EsrEnvironmentPreset::kStandard;

  environment::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.9f;
  environment::EsrAtmosphericDerivedContext atmospheric_context;
  atmospheric_context.has_k_factor = true;
  atmospheric_context.k_factor = 1.37f;

  const EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithAtmosphericPhysicsConfig(atmospheric_physics)
                                          .WithAtmosphericContext(atmospheric_context)
                                          .Build();
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_EQ(resolved.next_config.environment_model_config.preset,
            config::EsrEnvironmentPreset::kStandard);
  EXPECT_TRUE(resolved.next_config.environment_model_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_model_config.atmospheric_physics.relative_humidity,
                  0.9f);
  EXPECT_TRUE(resolved.next_config.environment_model_config.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_model_config.atmospheric_context.k_factor, 1.37f);
}

TEST(EsrRuntimeConfigResolverTest, InvalidExplicitBoundsRejectWholePatch) {
  ResolvedEsrSessionConfig current_config;
  current_config.runtime_config.scan_rate_hz = 1.0f;

  EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder().WithScanRateHz(3.0f).Build();
  patch.has_use_explicit_scan_bounds = true;
  patch.use_explicit_scan_bounds = true;
  patch.has_scan_start_az_deg = true;
  patch.has_scan_end_az_deg = true;
  patch.has_scan_start_el_deg = true;
  patch.has_scan_end_el_deg = true;
  patch.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  patch.scan_end_az_deg = 10.0f;
  patch.scan_start_el_deg = -10.0f;
  patch.scan_end_el_deg = 10.0f;

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_EQ(resolved.status, EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
  EXPECT_FALSE(resolved.runtime_config_changed);
  EXPECT_FALSE(resolved.pipeline_config_changed);
  EXPECT_FALSE(resolved.environment_model_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_config.runtime_config.scan_rate_hz, 1.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electronic_surveillance_radar
