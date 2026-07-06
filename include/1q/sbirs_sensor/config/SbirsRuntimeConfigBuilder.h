/**
 * @file SbirsRuntimeConfigBuilder.h
 * @brief 定义 SBIRS-inspired runtime patch builder。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_BUILDER_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"

namespace sbirs_sensor {
namespace config {

class ONEQ_API SbirsRuntimeConfigBuilder {
 public:
  SbirsRuntimeConfigBuilder& WithMission(const SbirsMissionConfig& mission);
  SbirsRuntimeConfigBuilder& WithPolicy(const SbirsPolicyConfig& policy);
  SbirsRuntimeConfigBuilder& WithEnvironment(const SbirsEnvironmentConfig& environment);
  SbirsRuntimeConfigBuilder& WithWorkMode(SbirsWorkMode work_mode);
  SbirsRuntimeConfigBuilder& WithScanRateDegPerSec(float scan_rate_deg_per_sec);
  SbirsRuntimeConfigBuilder& WithSensorEnabled(bool sensor_enabled);
  SbirsRuntimeConfigPatch Build() const;

 private:
  SbirsRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_BUILDER_H_
