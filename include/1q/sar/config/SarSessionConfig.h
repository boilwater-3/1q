/**
 * @file SarSessionConfig.h
 * @brief 定义 SAR 会话初始化配置结构。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_H_
#define ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/sar/config/SarEnvironmentConfig.h"
#include "1q/sar/config/SarHardwareConfig.h"
#include "1q/sar/config/SarMissionConfig.h"
#include "1q/sar/config/SarPolicyConfig.h"

namespace sar {
namespace config {

/**
 * @brief SAR 会话初始化高层输入。
 */
struct ONEQ_API SarSessionConfig {
  SarHardwareConfig hardware{};    /**< 传感器硬件与波形配置 */
  SarMissionConfig mission{};      /**< 平台与场景任务配置 */
  SarPolicyConfig policy{};        /**< 算法与运行策略配置 */
  SarEnvironmentConfig environment{}; /**< 环境与传播配置；字段生效范围见 SarEnvironmentConfig */
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_H_
