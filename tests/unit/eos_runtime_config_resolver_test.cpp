/**
 * @file eos_runtime_config_resolver_test.cpp
 * @brief 验证 EOS 运行期补丁解析器的原子更新语义。
 */

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace internal {
namespace {

namespace eos_config = ::electro_optical_sensor::config;

TEST(EosRuntimeConfigResolverTest, ValidPatchBuildsRuntimeUpdateAndScanResetFlag) {
  config::execution::EosInternalExecutionConfig current_config;
  current_config.scan.scan_rate_deg_per_sec = 20.0f;
  current_config.detection.minimum_snr_db = 6.0f;

  config::EosEnvironmentScenarioConfig env_config;
  env_config.has_custom_overrides = true;
  env_config.custom_overrides.radiative_transfer_model =
      ::electro_optical_sensor::foundation::radiative_transfer::
          RadiativeTransferModel::kAdaptivePathRadiance;
  env_config.custom_overrides.aerosol_density_factor = 2.0f;
  env_config.custom_overrides.turbulence_factor = 1.2f;

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
  // environment custom overrides applied
  EXPECT_EQ(
      resolved.next_config.environment.radiative_transfer_model,
      ::electro_optical_sensor::foundation::radiative_transfer::
          RadiativeTransferModel::kAdaptivePathRadiance);
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

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
