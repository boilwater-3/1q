/**
 * @file esr_runtime_config_resolver_test.cpp
 * @brief 验证 ESR 运行期补丁解析器的原子更新语义。
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

TEST(EsrRuntimeConfigResolverTest, ValidPatchBuildsRuntimePipelineAndEnvironmentUpdates) {
  ResolvedEsrSessionConfig current_config;
  current_config.runtime_config.scan_rate_hz = 1.0f;
  current_config.pipeline_config.detection.min_detect_snr_db = 6.0f;
  current_config.environment_model_config.jamming_detection_threshold_w = 1.0e-9f;

  const EsrRuntimeConfigPatch patch =
      esr_config::EsrRuntimeConfigBuilder()
          .WithScanRateHz(4.0f)
          .WithDetectionMinSnrDb(12.0f)
          .WithJammingDetectionThresholdW(2.0e-9f)
          .Build();

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.runtime_config_changed);
  EXPECT_TRUE(resolved.pipeline_config_changed);
  EXPECT_TRUE(resolved.environment_model_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_config.runtime_config.scan_rate_hz, 4.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.pipeline_config.detection.min_detect_snr_db, 12.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.environment_model_config.jamming_detection_threshold_w,
                  2.0e-9f);
}

TEST(EsrRuntimeConfigResolverTest, InvalidFieldRejectsWholePatch) {
  ResolvedEsrSessionConfig current_config;
  current_config.runtime_config.scan_rate_hz = 1.0f;
  current_config.pipeline_config.detection.min_detect_snr_db = 6.0f;

  EsrRuntimeConfigPatch patch =
      esr_config::EsrRuntimeConfigBuilder().WithScanRateHz(3.0f).WithDetectionMinSnrDb(9.0f).Build();
  patch.has_fixed_receiver_window_hz = true;
  patch.receiver_lower_hz = 3.0e9;
  patch.receiver_upper_hz = std::numeric_limits<double>::quiet_NaN();

  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.runtime_config_changed);
  EXPECT_FALSE(resolved.pipeline_config_changed);
  EXPECT_FALSE(resolved.environment_model_config_changed);
  EXPECT_FLOAT_EQ(resolved.next_config.runtime_config.scan_rate_hz, 1.0f);
  EXPECT_FLOAT_EQ(resolved.next_config.pipeline_config.detection.min_detect_snr_db, 6.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session

}  // namespace electronic_surveillance_radar
