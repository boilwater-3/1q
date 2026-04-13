/**
 * @file EsrSessionConfig.h
 * @brief 定义 ESR 会话初始化配置结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrLayeredConfig.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrSessionConfig 描述电子侦察会话默认装配配置。
 */
struct ONEQ_API EsrSessionConfig {
  bool enable_layered_config{false};                    /**< 是否启用分层参数覆盖 */
  EsrLayeredConfig layered_config{};                    /**< 分层参数入口 */
  extension::InterceptPipelineConfig pipeline_config{}; /**< 流水线配置 */
  environment::EsrEnvironmentDefaultConfig environment_default_config{}; /**< 默认环境配置 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_
