/**
 * @file EsrSessionConfigResolver.h
 * @brief 定义 ESR 会话分层参数到运行态配置的私有解析器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_CORE_SESSION_ESR_SESSION_CONFIG_RESOLVER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_CORE_SESSION_ESR_SESSION_CONFIG_RESOLVER_H_

#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {
namespace internal {

/**
 * @brief ResolvedEsrSessionConfig 描述会话装配前的统一解析结果。
 */
struct ResolvedEsrSessionConfig {
  pipeline::InterceptPipelineConfig pipeline_config{}; /**< 解析后的流水线配置 */
  environment::EsrEnvironmentModelConfig
      environment_model_config{};                    /**< 解析后的环境模型配置 */
  pipeline::InterceptRuntimeConfig runtime_config{}; /**< 解析后的运行态配置 */
};

/**
 * @brief 解析会话配置并生成运行态唯一真值。
 * @param[in] config 输入会话配置。
 * @return 解析后的会话配置结果。
 */
ResolvedEsrSessionConfig ResolveEsrSessionConfig(const EsrSessionConfig& config);

}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_CORE_SESSION_ESR_SESSION_CONFIG_RESOLVER_H_
