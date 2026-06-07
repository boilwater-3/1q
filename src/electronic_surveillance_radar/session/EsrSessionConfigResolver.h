/**
 * @file EsrSessionConfigResolver.h
 * @brief 定义 ESR 会话分层参数到运行态配置的私有解析器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_CONFIG_RESOLVER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_CONFIG_RESOLVER_H_

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief 将会话配置映射为内部执行态配置。
 * @param[in] config 输入会话配置。
 * @return 内部执行态配置。
 */
EsrInternalExecutionConfig MapSessionToInternal(const config::EsrSessionConfig& config);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_CONFIG_RESOLVER_H_
