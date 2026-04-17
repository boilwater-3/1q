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

  const EsrRuntimeConfigPatch patch = esr_config::EsrRuntimeConfigBuilder()
                                          .WithScanRateHz(4.0f)
                                          .WithWorkMode(esr_config::EsrWorkMode::kRwr)
                                          .WithEnvironmentPreset(esr_config::EsrEnvironmentPreset::kDenseClutter)
                                          .Build();

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.runtime_config_changed);
  EXPECT_TRUE(resolved.pipeline_config_changed);
  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_config.runtime_config.scan_rate_hz, 4.0f);
  EXPECT_EQ(resolved.next_config.environment_model_config.preset,
            config::EsrEnvironmentPreset::kDenseClutter);
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
  EXPECT_FALSE(resolved.runtime_config_changed);
  EXPECT_FALSE(resolved.pipeline_config_changed);
  EXPECT_FALSE(resolved.environment_model_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_config.runtime_config.scan_rate_hz, 1.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electronic_surveillance_radar
