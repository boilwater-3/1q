/**
 * @file sar_runtime_config_resolver_test.cpp
 * @brief SAR 运行期补丁解析器测试。
 *
 * 对齐 ESR/EOS resolver 测试约定：覆盖有效补丁写入、空补丁、无效字段拒绝
 * （NaN snr、L1 RDA 缺 raw echo）、拒绝时 next_config 原样回传。
 */

#include <gtest/gtest.h>

#include <limits>

#include "1q/sar/config/SarRuntimeConfigBuilder.h"
#include "sar/session/SarRuntimeConfigResolver.h"

namespace sar {
namespace session {
namespace internal {
namespace {

namespace sar_config = ::sar::config;

TEST(SarRuntimeConfigResolverTest, EmptyPatchReportsNoRequestedUpdateAndIsUnchanged) {
  const config::SarSessionConfig current_config;
  const config::SarRuntimeConfigPatch patch{};

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(current_config, patch);

  EXPECT_FALSE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FALSE(resolved.policy_changed);
  EXPECT_EQ(resolved.next_config.policy.retain_focused_image,
            current_config.policy.retain_focused_image);
}

TEST(SarRuntimeConfigResolverTest, ValidPatchWritesPolicyFields) {
  const config::SarRuntimeConfigPatch patch = sar_config::SarRuntimeConfigBuilder()
                                                  .WithEnableL1RdaImaging(true)
                                                  .WithMinimumSnrDb(-5.0)
                                                  .WithRetainFocusedImage(false)
                                                  .Build();

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(config::SarSessionConfig{}, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.policy_changed);
  EXPECT_TRUE(resolved.next_config.policy.enable_l1_rda_imaging);
  EXPECT_DOUBLE_EQ(resolved.next_config.policy.minimum_snr_db, -5.0);
  EXPECT_FALSE(resolved.next_config.policy.retain_focused_image);
}

TEST(SarRuntimeConfigResolverTest, PatchEnablingRdaAndRawEchoTogetherIsAccepted) {
  // 同一补丁内同时打开 raw echo + L1 RDA，resolve 后两者并存，应放行。
  const config::SarRuntimeConfigPatch patch = sar_config::SarRuntimeConfigBuilder()
                                                  .WithEnableRawEchoGeneration(true)
                                                  .WithEnableL1RdaImaging(true)
                                                  .Build();

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(config::SarSessionConfig{}, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.next_config.policy.enable_raw_echo_generation);
  EXPECT_TRUE(resolved.next_config.policy.enable_l1_rda_imaging);
}

TEST(SarRuntimeConfigResolverTest, NonFiniteMinimumSnrDbRejectsWholePatch) {
  const config::SarRuntimeConfigPatch patch =
      sar_config::SarRuntimeConfigBuilder().WithMinimumSnrDb(std::numeric_limits<double>::quiet_NaN()).Build();

  config::SarSessionConfig current_config;
  current_config.policy.minimum_snr_db = -10.0;
  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  // 拒绝时 next_config 原样回传 current_config，未被污染。
  EXPECT_DOUBLE_EQ(resolved.next_config.policy.minimum_snr_db, -10.0);
}

TEST(SarRuntimeConfigResolverTest, L1RdaWithoutRawEchoRejectsWholePatch) {
  // current_config 已关闭 raw echo；补丁单独打开 L1 RDA，违反依赖不变式，应拒绝。
  config::SarSessionConfig current_config;
  current_config.policy.enable_raw_echo_generation = false;

  const config::SarRuntimeConfigPatch patch =
      sar_config::SarRuntimeConfigBuilder().WithEnableL1RdaImaging(true).Build();

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.next_config.policy.enable_l1_rda_imaging);
}

TEST(SarRuntimeConfigResolverTest, L1RdaBlockedEvenIfCurrentConfigEnabledRawEcho) {
  // 补丁显式关闭 raw echo 同时打开 L1 RDA：resolve 基于 next_config 判定，应拒绝。
  const config::SarRuntimeConfigPatch patch = sar_config::SarRuntimeConfigBuilder()
                                                  .WithEnableRawEchoGeneration(false)
                                                  .WithEnableL1RdaImaging(true)
                                                  .Build();

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(config::SarSessionConfig{}, patch);

  EXPECT_FALSE(resolved.is_valid);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace sar
