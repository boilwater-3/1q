/**
 * @file eos_runtime_config_resolver_test.cpp
 * @brief 验证 EOS 运行期补丁解析器的原子更新语义。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/config/EosPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace internal {
namespace {

namespace eos_config = ::electro_optical_sensor::config;

config::execution::EosInternalExecutionConfig MakeValidCurrentConfig() {
  config::execution::EosInternalExecutionConfig config;
  config.scan.scan_rate_deg_per_sec = 20.0f;
  config.scan.frame_rate_hz = 30.0f;
  config.detection.minimum_snr_db = 6.0f;
  return config;
}

TEST(EosRuntimeConfigResolverTest, ValidPatchBuildsRuntimeUpdateAndScanResetFlag) {
  config::execution::EosInternalExecutionConfig current_config;
  current_config.scan.scan_rate_deg_per_sec = 20.0f;
  current_config.detection.minimum_snr_db = 6.0f;

  config::EosEnvironmentScenarioConfig env_config;
  env_config.preset = eos_config::EosEnvironmentPreset::kDusty;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(60.0f)
          .WithMinimumSnrDb(60.0f)
          .WithDetectionSensitivityW(2.0e-12f)
          .WithVisibleReferenceIrradianceWM2(1000.0f)
          .WithEnvironmentScenarioConfig(env_config)
          .Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.reset_scan_phase);
  // scan field updated
  EXPECT_FLOAT_EQ(resolved.next_config.scan.scan_rate_deg_per_sec, 60.0f);
  // detection values set directly
  EXPECT_FLOAT_EQ(resolved.next_config.detection.minimum_snr_db, 60.0f);
  // environment preset applied
  EXPECT_FLOAT_EQ(
      resolved.next_config.environment.aerosol_density_factor, 2.0f);
  EXPECT_FLOAT_EQ(
      resolved.next_config.environment.turbulence_factor, 1.2f);
}

TEST(EosRuntimeConfigResolverTest, InvalidFieldRejectsWholePatch) {
  config::execution::EosInternalExecutionConfig current_config;
  current_config.scan.scan_rate_deg_per_sec = 20.0f;
  current_config.detection.minimum_snr_db = 6.0f;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(60.0f)
          .WithMinimumSnrDb(6.0f)
          .WithFrameRateHz(0.0f)
          .Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.reset_scan_phase);
  // values unchanged on reject
  EXPECT_FLOAT_EQ(resolved.next_config.scan.scan_rate_deg_per_sec, 20.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.detection.minimum_snr_db, 6.0f);
}

// =============================================================================
// Mission 域校验（IsValidMission 的全部分支）
// =============================================================================

TEST(EosRuntimeConfigResolverTest, ValidMissionPatchAppliesAndResetsScanPhase) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosMissionConfig mission;
  mission.work_mode = eos_config::EosWorkMode::kFused;
  mission.horizontal_fov_deg = 6.0f;
  mission.vertical_fov_deg = 4.0f;
  mission.scan_rate_deg_per_sec = 60.0f;
  mission.frame_rate_hz = 30.0f;
  mission.scan_start_az_deg = -60.0f;
  mission.scan_end_az_deg = 60.0f;
  mission.scan_center_el_deg = 0.0f;
  mission.boresight_depression_deg = 45.0f;
  mission.power_on = false;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithMission(mission).Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.reset_scan_phase);
  EXPECT_FLOAT_EQ(resolved.next_config.scan.scan_rate_deg_per_sec, 60.0f);
  EXPECT_EQ(resolved.next_config.scan.work_mode, eos_config::EosWorkMode::kFused);
  EXPECT_FALSE(resolved.next_config.sensor_enabled);
}

TEST(EosRuntimeConfigResolverTest, SensorEnabledLeafOverridesMissionPowerState) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosMissionConfig mission;
  mission.power_on = false;
  eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithMission(mission).Build();
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = true;

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  ASSERT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.next_config.sensor_enabled);
}

TEST(EosRuntimeConfigResolverTest, MissionWithZeroScanRateIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosMissionConfig mission;
  mission.scan_rate_deg_per_sec = 0.0f;
  mission.frame_rate_hz = 30.0f;
  mission.horizontal_fov_deg = 6.0f;
  mission.vertical_fov_deg = 4.0f;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithMission(mission).Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);
  EXPECT_FALSE(resolved.is_valid);
}

TEST(EosRuntimeConfigResolverTest, MissionWithZeroFrameRateIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosMissionConfig mission;
  mission.scan_rate_deg_per_sec = 60.0f;
  mission.frame_rate_hz = 0.0f;
  mission.horizontal_fov_deg = 6.0f;
  mission.vertical_fov_deg = 4.0f;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithMission(mission).Build();

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(current_config, patch).is_valid);
}

TEST(EosRuntimeConfigResolverTest, MissionWithZeroFovIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosMissionConfig mission;
  mission.scan_rate_deg_per_sec = 60.0f;
  mission.frame_rate_hz = 30.0f;
  mission.horizontal_fov_deg = 0.0f;
  mission.vertical_fov_deg = 4.0f;

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithMission(mission).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, MissionWithNanFieldIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosMissionConfig mission;
  mission.scan_rate_deg_per_sec = 60.0f;
  mission.frame_rate_hz = 30.0f;
  mission.horizontal_fov_deg = 6.0f;
  mission.vertical_fov_deg = 4.0f;
  mission.scan_center_el_deg = std::numeric_limits<float>::quiet_NaN();

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithMission(mission).Build())
                   .is_valid);
}

// =============================================================================
// Policy 域校验（IsValidDetectionPolicy + IsValidStrayLightPolicy）
// =============================================================================

TEST(EosRuntimeConfigResolverTest, ValidPolicyPatchApplies) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.detection.minimum_snr_db = 8.0f;
  policy.detection.detection_sensitivity_w = 1.0e-12f;
  policy.detection.visible_reference_irradiance_w_m2 = 800.0f;
  policy.stray_light.enable_straylight_filter = true;
  policy.stray_light.hood_inner_half_angle_deg = 12.0f;
  policy.stray_light.hood_outer_half_angle_deg = 75.0f;
  policy.stray_light.hood_min_suppression_ratio = 0.2f;
  policy.stray_light.hood_max_suppression_ratio = 0.85f;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FLOAT_EQ(resolved.next_config.detection.minimum_snr_db, 8.0f);
}

TEST(EosRuntimeConfigResolverTest, PolicyWithZeroDetectionSensitivityIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.detection.detection_sensitivity_w = 0.0f;

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, PolicyWithZeroIrradianceIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.detection.visible_reference_irradiance_w_m2 = 0.0f;

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, PolicyWithZeroHoodInnerAngleIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.stray_light.hood_inner_half_angle_deg = 0.0f;

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, PolicyWithOuterLessThanInnerHoodIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.stray_light.hood_inner_half_angle_deg = 50.0f;
  policy.stray_light.hood_outer_half_angle_deg = 30.0f;  // < inner

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, PolicyWithSuppressionRatioOutOfRangeIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.stray_light.hood_min_suppression_ratio = -0.1f;

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, PolicyWithMaxLessThanMinSuppressionIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.stray_light.hood_min_suppression_ratio = 0.8f;
  policy.stray_light.hood_max_suppression_ratio = 0.2f;  // < min

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, PolicyWithNanHoodAngleIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosPolicyConfig policy;
  policy.stray_light.hood_outer_half_angle_deg = std::numeric_limits<float>::quiet_NaN();

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithPolicy(policy).Build())
                   .is_valid);
}

// =============================================================================
// 环境补丁校验（IsValidEnvironmentPatch 边界）
// =============================================================================

TEST(EosRuntimeConfigResolverTest, EnvironmentWithoutScenarioConfigIsValid) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosEnvironmentRuntimeConfigPatch env_patch;
  env_patch.has_scenario_config = false;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithEnvironment(env_patch).Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);
  EXPECT_TRUE(resolved.is_valid);
}

TEST(EosRuntimeConfigResolverTest, EnvironmentWithInvalidAtmosphericPressureIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosEnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  scenario.atmospheric_physics.pressure_hpa = 0.0f;

  eos_config::EosEnvironmentRuntimeConfigPatch env_patch;
  env_patch.has_scenario_config = true;
  env_patch.scenario_config = scenario;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithEnvironment(env_patch).Build();

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(current_config, patch).is_valid);
}

TEST(EosRuntimeConfigResolverTest, EnvironmentWithInvalidAtmosphericHumidityIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  eos_config::EosEnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  scenario.atmospheric_physics.relative_humidity = 1.1f;

  eos_config::EosEnvironmentRuntimeConfigPatch env_patch;
  env_patch.has_scenario_config = true;
  env_patch.scenario_config = scenario;

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(
                   current_config,
                   eos_config::EosRuntimeConfigBuilder().WithEnvironment(env_patch).Build())
                   .is_valid);
}

TEST(EosRuntimeConfigResolverTest, EnvironmentWithInvalidPresetIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();
  eos_config::EosEnvironmentScenarioConfig scenario;
  scenario.preset = static_cast<eos_config::EosEnvironmentPreset>(99);
  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithEnvironmentScenarioConfig(scenario).Build();
  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(current_config, patch).is_valid);
}
TEST(EosRuntimeConfigResolverTest, EnvironmentPresetAppliesAtRuntime) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();
  eos_config::EosEnvironmentScenarioConfig scenario;
  scenario.preset = eos_config::EosEnvironmentPreset::kMaritime;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithEnvironmentScenarioConfig(scenario).Build();
  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.aerosol_density_factor, 1.5f);
  EXPECT_FLOAT_EQ(resolved.next_config.environment.turbulence_factor, 1.4f);
}

// =============================================================================
// 简单补丁路径（work_mode / sensor_enabled / frame_rate / scan_rate）
// =============================================================================

TEST(EosRuntimeConfigResolverTest, WorkModePatchApplies) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithWorkMode(eos_config::EosWorkMode::kInfraredOnly)
          .Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_EQ(resolved.next_config.scan.work_mode, eos_config::EosWorkMode::kInfraredOnly);
}

TEST(EosRuntimeConfigResolverTest, SensorEnabledPatchApplies) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();
  current_config.sensor_enabled = true;

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithSensorEnabled(false).Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FALSE(resolved.next_config.sensor_enabled);
}

TEST(EosRuntimeConfigResolverTest, ValidFrameRatePatchApplies) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(60.0f).Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FLOAT_EQ(resolved.next_config.scan.frame_rate_hz, 60.0f);
}

TEST(EosRuntimeConfigResolverTest, InvalidFrameRateZeroIsRejected) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build();

  EXPECT_FALSE(ResolveEosRuntimeConfigPatch(current_config, patch).is_valid);
}

TEST(EosRuntimeConfigResolverTest, EmptyPatchProducesNoUpdate) {
  config::execution::EosInternalExecutionConfig current_config = MakeValidCurrentConfig();

  const eos_config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);
  EXPECT_FALSE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
