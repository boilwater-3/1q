#include "1q/sbirs_sensor/config/SbirsSessionConfigBuilder.h"

namespace sbirs_sensor {
namespace config {

SbirsSessionConfigBuilder& SbirsSessionConfigBuilder::WithHardware(
    const SbirsHardwareConfig& hardware) {
  config_.hardware = hardware;
  return *this;
}

SbirsSessionConfigBuilder& SbirsSessionConfigBuilder::WithMission(
    const SbirsMissionConfig& mission) {
  config_.mission = mission;
  return *this;
}

SbirsSessionConfigBuilder& SbirsSessionConfigBuilder::WithPolicy(const SbirsPolicyConfig& policy) {
  config_.policy = policy;
  return *this;
}

SbirsSessionConfigBuilder& SbirsSessionConfigBuilder::WithEnvironment(
    const SbirsEnvironmentConfig& environment) {
  config_.environment = environment;
  return *this;
}

SbirsSessionConfig SbirsSessionConfigBuilder::Build() const { return config_; }

}  // namespace config
}  // namespace sbirs_sensor
