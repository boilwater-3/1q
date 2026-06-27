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
  SarHardwareConfig hardware{};
  SarMissionConfig mission{};
  SarPolicyConfig policy{};
  /// 保留域：environment 当前不进入计算链路，仅用于 replay/config 保真。
  /// 详见 SarEnvironmentConfig 的保留域说明。
  SarEnvironmentConfig environment{};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_H_
