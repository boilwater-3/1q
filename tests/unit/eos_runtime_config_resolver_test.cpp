/**
 * @file eos_runtime_config_resolver_test.cpp
 * @brief 验证 EOS 运行期补丁解析器的原子更新语义。
 */

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "electro_optical_sensor/core/session/EosRuntimeConfigResolver.h"

namespace electro_optical_sensor {
namespace core {
namespace session {
namespace internal {
namespace {

namespace eos_config = ::electro_optical_sensor::config;

TEST(EosRuntimeConfigResolverTest, ValidPatchBuildsRuntimeUpdateAndScanResetFlag) {
  EosSessionConfig current_config;
  current_config.scan_rate_deg_per_sec = 20.0f;
  current_config.minimum_snr_db = 6.0f;
  current_config.environment_default_config.aerosol_density_factor = 1.0f;

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(60.0f)
          .WithMinimumSnrDb(12.0f)
          .WithAerosolDensityFactor(1.5f)
          .Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.reset_scan_phase);
  EXPECT_FLOAT_EQ(resolved.next_config.scan_rate_deg_per_sec, 60.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.minimum_snr_db, 12.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_default_config.aerosol_density_factor, 1.5f);
}

TEST(EosRuntimeConfigResolverTest, InvalidFieldRejectsWholePatch) {
  EosSessionConfig current_config;
  current_config.scan_rate_deg_per_sec = 20.0f;
  current_config.minimum_snr_db = 6.0f;

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(60.0f)
          .WithMinimumSnrDb(12.0f)
          .WithFrameRateHz(0.0f)
          .Build();

  const EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.reset_scan_phase);
  EXPECT_FLOAT_EQ(resolved.next_config.scan_rate_deg_per_sec, 20.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.minimum_snr_db, 6.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor
