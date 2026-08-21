/**
 * @file sar_runtime_config_resolver_test.cpp
 * @brief SAR 运行期补丁解析器测试。
 *
 * 对齐 ESR/EOS resolver 测试约定：覆盖有效补丁写入、空补丁、无效字段拒绝
 * （NaN snr、L1 RDA 缺 raw echo）、拒绝时 next_config 原样回传。
 */

#include <gtest/gtest.h>

#include <limits>

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "sar/session/SarRuntimeConfigResolver.h"

namespace sar {
namespace session {
namespace internal {
namespace {

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
  config::SarRuntimeConfigPatch patch;
  patch.has_enable_l1_rda_imaging = true;
  patch.enable_l1_rda_imaging = true;
  patch.has_minimum_snr_db = true;
  patch.minimum_snr_db = -5.0;
  patch.has_retain_focused_image = true;
  patch.retain_focused_image = false;

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
  config::SarRuntimeConfigPatch patch;
  patch.has_enable_raw_echo_generation = true;
  patch.enable_raw_echo_generation = true;
  patch.has_enable_l1_rda_imaging = true;
  patch.enable_l1_rda_imaging = true;

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(config::SarSessionConfig{}, patch);

  EXPECT_TRUE(resolved.is_valid);
  EXPECT_TRUE(resolved.next_config.policy.enable_raw_echo_generation);
  EXPECT_TRUE(resolved.next_config.policy.enable_l1_rda_imaging);
}

TEST(SarRuntimeConfigResolverTest, NonFiniteMinimumSnrDbRejectsWholePatch) {
  config::SarRuntimeConfigPatch patch;
  patch.has_minimum_snr_db = true;
  patch.minimum_snr_db = std::numeric_limits<double>::quiet_NaN();

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

  config::SarRuntimeConfigPatch patch;
  patch.has_enable_l1_rda_imaging = true;
  patch.enable_l1_rda_imaging = true;

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.next_config.policy.enable_l1_rda_imaging);
}

TEST(SarRuntimeConfigResolverTest, L1RdaBlockedEvenIfCurrentConfigEnabledRawEcho) {
  // 补丁显式关闭 raw echo 同时打开 L1 RDA：resolve 基于 next_config 判定，应拒绝。
  config::SarRuntimeConfigPatch patch;
  patch.has_enable_raw_echo_generation = true;
  patch.enable_raw_echo_generation = false;
  patch.has_enable_l1_rda_imaging = true;
  patch.enable_l1_rda_imaging = true;

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(config::SarSessionConfig{}, patch);

  EXPECT_FALSE(resolved.is_valid);
}

TEST(SarRuntimeConfigResolverTest, RetainRawHistoryWithoutRawEchoRejectsWholePatch) {
  config::SarRuntimeConfigPatch patch;
  patch.has_enable_raw_echo_generation = true;
  patch.enable_raw_echo_generation = false;
  patch.has_retain_raw_phase_history = true;
  patch.retain_raw_phase_history = true;
  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(config::SarSessionConfig{}, patch);
  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_FALSE(resolved.is_valid);
  EXPECT_FALSE(resolved.next_config.policy.retain_raw_phase_history);
  EXPECT_TRUE(resolved.next_config.policy.enable_raw_echo_generation);
}

TEST(SarRuntimeConfigResolverTest, SensorEnabledLeafUpdatesConfig) {
  // 电源叶子（COMMON-OQ-4 字段提升）：false 值透传到 next_config。
  config::SarRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(config::SarSessionConfig{}, patch);

  EXPECT_TRUE(resolved.has_requested_update);
  EXPECT_TRUE(resolved.is_valid);
  EXPECT_FALSE(resolved.next_config.sensor_enabled);
}

TEST(SarRuntimeConfigResolverTest, ProcessingPatchPreservesExistingPowerState) {
  // 电源仅由叶子 has_sensor_enabled 控制：非电源补丁不改变电源状态。
  config::SarSessionConfig current_config;
  current_config.sensor_enabled = false;

  config::SarRuntimeConfigPatch patch;
  patch.has_minimum_snr_db = true;
  patch.minimum_snr_db = -5.0;

  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(current_config, patch);

  EXPECT_TRUE(resolved.is_valid);
  // 关机状态下处理补丁仍可应用，但电源保持 false（叶子唯一控制）。
  EXPECT_FALSE(resolved.next_config.sensor_enabled);
  EXPECT_DOUBLE_EQ(resolved.next_config.policy.minimum_snr_db, -5.0);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace sar
