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
namespace eos_session = ::electro_optical_sensor::session;

TEST(EosRuntimeConfigResolverTest, ValidPatchBuildsRuntimeUpdateAndScanResetFlag) {
  eos_session::EosSessionConfig current_config;
  current_config.mission.scan_rate_deg_per_sec = 20.0f;
  current_config.policy.detection.profile = eos_config::EosDetectionProfile::kBalanced;
  current_config.environment.scenario_config.preset = eos_config::EosEnvironmentPreset::kStandard;

  const eos_session::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(60.0f)
          .WithDetectionProfile(eos_config::EosDetectionProfile::kConservative)
          .WithEnvironmentDetails(
              ::electro_optical_sensor::foundation::radiative_transfer::
                  RadiativeTransferModel::kAdaptivePathRadiance,
              2.0f,
              1.2f)
          .Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.reset_scan_phase);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan_rate_deg_per_sec, 60.0f);
  EXPECT_EQ(resolved.next_config.policy.detection.profile, eos_config::EosDetectionProfile::kConservative);
  EXPECT_TRUE(resolved.next_config.environment.scenario_config.has_custom_overrides);
  EXPECT_EQ(
      resolved.next_config.environment.scenario_config.custom_overrides.radiative_transfer_model,
      ::electro_optical_sensor::foundation::radiative_transfer::
          RadiativeTransferModel::kAdaptivePathRadiance);
  EXPECT_FLOAT_EQ(
      resolved.next_config.environment.scenario_config.custom_overrides.aerosol_density_factor,
      2.0f);
  EXPECT_FLOAT_EQ(
      resolved.next_config.environment.scenario_config.custom_overrides.turbulence_factor,
      1.2f);
}

TEST(EosRuntimeConfigResolverTest, InvalidFieldRejectsWholePatch) {
  eos_session::EosSessionConfig current_config;
  current_config.mission.scan_rate_deg_per_sec = 20.0f;
  current_config.policy.detection.profile = eos_config::EosDetectionProfile::kBalanced;

  const eos_session::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(60.0f)
          .WithDetectionProfile(eos_config::EosDetectionProfile::kConservative)
          .WithFrameRateHz(0.0f)
          .Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.reset_scan_phase);
  EXPECT_FLOAT_EQ(resolved.next_config.mission.scan_rate_deg_per_sec, 20.0f);
  EXPECT_EQ(resolved.next_config.policy.detection.profile, eos_config::EosDetectionProfile::kBalanced);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
