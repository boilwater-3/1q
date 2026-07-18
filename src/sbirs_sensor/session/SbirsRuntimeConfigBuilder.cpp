#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"

namespace sbirs_sensor {
namespace config {

SbirsRuntimeConfigBuilder& SbirsRuntimeConfigBuilder::WithMission(
    const SbirsMissionConfig& mission) {
  patch_.has_mission = true;
  patch_.mission = mission;
  return *this;
}

SbirsRuntimeConfigBuilder& SbirsRuntimeConfigBuilder::WithPolicy(const SbirsPolicyConfig& policy) {
  patch_.has_policy = true;
  patch_.policy = policy;
  return *this;
}

SbirsRuntimeConfigBuilder& SbirsRuntimeConfigBuilder::WithEnvironment(
    const SbirsEnvironmentConfig& environment) {
  patch_.has_environment = true;
  patch_.environment = environment;
  return *this;
}

SbirsRuntimeConfigBuilder& SbirsRuntimeConfigBuilder::WithWorkMode(SbirsWorkMode work_mode) {
  patch_.has_work_mode = true;
  patch_.work_mode = work_mode;
  return *this;
}

SbirsRuntimeConfigBuilder& SbirsRuntimeConfigBuilder::WithScanRateDegPerSec(
    float scan_rate_deg_per_sec) {
  patch_.has_scan_rate_deg_per_sec = true;
  patch_.scan_rate_deg_per_sec = scan_rate_deg_per_sec;
  return *this;
}

SbirsRuntimeConfigBuilder& SbirsRuntimeConfigBuilder::WithPowerOn(bool power_on) {
  patch_.has_power_on = true;
  patch_.power_on = power_on;
  return *this;
}

SbirsRuntimeConfigPatch SbirsRuntimeConfigBuilder::Build() const { return patch_; }

}  // namespace config
}  // namespace sbirs_sensor
