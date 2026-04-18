/**
 * @file EsrSessionConfig.h
 * @brief 定义 ESR 会话初始化配置结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrHardwareConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrMissionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrPolicyConfig.h"

namespace electronic_surveillance_radar {
namespace session {

using EsrWorkMode = config::EsrWorkMode;
using EsrScanStartPosition = config::EsrScanStartPosition;
using EsrScanSequence = config::EsrScanSequence;
using EsrHardwareConfig = config::EsrHardwareConfig;
using EsrMissionConfig = config::EsrMissionConfig;
using EsrPolicyConfig = config::EsrPolicyConfig;
using EsrEnvironmentConfig = config::EsrEnvironmentConfig;

/**
 * @brief EsrSessionConfig 描述电子侦察会话初始化高层输入。
 */
struct ONEQ_API EsrSessionConfig {
  config::EsrHardwareConfig hardware{};       /**< 装备固有能力输入 */
  config::EsrMissionConfig mission{};         /**< 任务域语义输入 */
  config::EsrPolicyConfig policy{};           /**< 策略域语义输入 */
  config::EsrEnvironmentConfig environment{}; /**< 环境域语义输入 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_H_
