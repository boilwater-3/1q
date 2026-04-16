/**
 * @file EosSessionConfig.h
 * @brief 定义 EOS 会话初始化配置结构。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosDetectionPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosEnvironmentPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosOpticalConfig.h"
#include "1q/electro_optical_sensor/config/EosPointingConfig.h"
#include "1q/electro_optical_sensor/config/EosScanPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosStrayLightPolicyConfig.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosSessionConfig 描述 EOS 会话初始化高层输入。
 */
struct ONEQ_API EosSessionConfig {
  config::EosOpticalHardwareConfig optical{};
  config::EosScanPolicyConfig scan{};
  config::EosPointingConfig pointing{};
  config::EosDetectionPolicyConfig detection{};
  config::EosStrayLightPolicyConfig stray_light{};
  config::EosEnvironmentPolicyConfig environment{};
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
