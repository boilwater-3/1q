/**
 * @file SbirsSessionConfigBuilder.h
 * @brief 定义 SBIRS-inspired 会话配置 builder。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_BUILDER_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace sbirs_sensor {
namespace config {

class ONEQ_API SbirsSessionConfigBuilder {
 public:
  SbirsSessionConfigBuilder& WithHardware(const SbirsHardwareConfig& hardware);
  SbirsSessionConfigBuilder& WithMission(const SbirsMissionConfig& mission);
  SbirsSessionConfigBuilder& WithPolicy(const SbirsPolicyConfig& policy);
  SbirsSessionConfigBuilder& WithEnvironment(const SbirsEnvironmentConfig& environment);
  SbirsSessionConfig Build() const;

 private:
  SbirsSessionConfig config_{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_BUILDER_H_
