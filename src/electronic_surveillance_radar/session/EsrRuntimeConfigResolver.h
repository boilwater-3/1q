#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_RUNTIME_CONFIG_RESOLVER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_RUNTIME_CONFIG_RESOLVER_H_

#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "electronic_surveillance_radar/session/EsrSessionConfigResolver.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrRuntimeConfigResolveResult 描述 ESR 运行期补丁解析结果。
 */
struct EsrRuntimeConfigResolveResult {
  ResolvedEsrSessionConfig next_config{};
  EsrRuntimeConfigApplyStatus status{EsrRuntimeConfigApplyStatus::kNoRequestedUpdate};
  bool has_requested_update{false};
  bool is_valid{true};
  bool runtime_config_changed{false};
  bool pipeline_config_changed{false};
  bool environment_model_config_changed{false};
};

/**
 * @brief 解析 ESR 运行期补丁并返回统一更新计划。
 * @param[in] current_config 当前解析后会话配置。
 * @param[in] patch 运行期补丁。
 * @return 解析结果。
 */
EsrRuntimeConfigResolveResult ResolveEsrRuntimeConfigPatch(
    const ResolvedEsrSessionConfig& current_config, const config::EsrRuntimeConfigPatch& patch);

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_RUNTIME_CONFIG_RESOLVER_H_
