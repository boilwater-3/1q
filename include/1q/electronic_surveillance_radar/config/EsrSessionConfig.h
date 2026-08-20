/**
 * @file EsrSessionConfig.h
 * @brief 定义 ESR 会话初始化配置结构。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrHardwareConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrMissionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrPolicyConfig.h"

namespace electronic_surveillance_radar {
namespace config {


/**
 * @brief config::EsrSessionConfig 描述电子侦察会话初始化高层输入。
 */
struct ONEQ_API EsrSessionConfig {
  EsrHardwareConfig hardware{};       /**< 装备固有能力输入 */
  EsrMissionConfig mission{};         /**< 任务域语义输入 */
  EsrPolicyConfig policy{};           /**< 策略域语义输入 */
  EsrEnvironmentConfig environment{}; /**< 环境域语义输入 */
  bool sensor_enabled{true};          /**< 传感器初始电源状态（COMMON-OQ-4 字段提升） */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_
