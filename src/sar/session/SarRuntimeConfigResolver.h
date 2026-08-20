/**
 * @file SarRuntimeConfigResolver.h
 * @brief SAR 运行期补丁解析与可前置不变式校验。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_RESOLVER_H_
#define ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_RESOLVER_H_

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/config/SarRuntimeConfigPatch.h"

namespace sar {
namespace session {

/**
 * @brief SarRuntimeConfigResolveResult 描述 SAR 运行期补丁解析结果。
 *
 * 与 ESR/EOS resolver 同型：boolean-only reject（无 status enum），对齐
 * AR/ESR/EOS 的"写之前校验"语义。原先 SAR 的 TryApplyRuntimeConfig 直接
 * ApplyPatchToConfig 盲写 has_* 标志位且恒返回 true，校验整体推迟到 Step
 * 内的 ValidateRuntimeConfigForStep；引入本 resolver 把可静态判定的不变式
 * 前置到 apply 时刻。
 */
struct SarRuntimeConfigResolveResult {
  config::SarSessionConfig next_config{};
  bool has_requested_update{false};
  bool is_valid{true};
  bool policy_changed{false};
};

/**
 * @brief 解析 SAR 运行期补丁并返回带校验的更新计划。
 * @param[in] current_config 当前会话配置。
 * @param[in] patch 运行期补丁。
 * @return 解析结果；is_valid=false 时 next_config 原样回传 current_config。
 *
 * 校验规则（对齐 step-time ValidateRuntimeConfigForStep 的可前置子集）：
 * - minimum_snr_db 必须为有限值；
 * - L1 RDA 成像依赖 raw echo generation（enable_l1_rda_imaging=true 时
 *   enable_raw_echo_generation 必须为 true），原为 step-time gate
 *   （SarRuntimeConfigValidation.cpp:54-58），此处前置。
 *
 * 对有效补丁行为零变化：仍写 policy 字段、返回 is_valid=true。
 */
SarRuntimeConfigResolveResult ResolveSarRuntimeConfigPatch(
    const config::SarSessionConfig& current_config,
    const config::SarRuntimeConfigPatch& patch);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_RESOLVER_H_
