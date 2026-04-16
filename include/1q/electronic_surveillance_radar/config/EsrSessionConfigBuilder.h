/**
 * @file EsrSessionConfigBuilder.h
 * @brief ESR 对外配置入口：初始化配置类型与 SessionConfig Builder。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrSessionConfigBuilder 提供初始化配置链式构造。
 */
class ONEQ_API EsrSessionConfigBuilder {
 public:
  explicit EsrSessionConfigBuilder(
      const session::EsrSessionConfig& config = {}) : config_(config) {}

  EsrSessionConfigBuilder& WithSessionConfig(const session::EsrSessionConfig& config) {
    config_ = config;
    return *this;
  }
  EsrSessionConfigBuilder& WithWorkMode(config::EsrWorkMode mode) {
    config_.mission.work_mode = mode;
    return *this;
  }
  EsrSessionConfigBuilder& WithPowerOn(bool power_on) {
    config_.mission.power_on = power_on;
    return *this;
  }
  EsrSessionConfigBuilder& WithScanRateHz(float value) {
    config_.mission.scan.scan_rate_hz = value;
    return *this;
  }
  EsrSessionConfigBuilder& WithDetectionProfile(config::EsrDetectionProfile profile) {
    config_.detection.profile = profile;
    return *this;
  }
  EsrSessionConfigBuilder& WithEnvironmentPreset(config::EsrEnvironmentPreset preset) {
    config_.environment.preset = preset;
    return *this;
  }
  EsrSessionConfigBuilder& WithHardwareConfig(const config::EsrHardwareConfig& hardware) {
    config_.hardware = hardware;
    return *this;
  }
  EsrSessionConfigBuilder& WithMissionControlConfig(
      const config::EsrMissionControlConfig& mission) {
    config_.mission = mission;
    return *this;
  }
  EsrSessionConfigBuilder& WithDetectionPolicyConfig(
      const config::EsrDetectionPolicyConfig& detection) {
    config_.detection = detection;
    return *this;
  }
  EsrSessionConfigBuilder& WithEnvironmentPolicyConfig(
      const config::EsrEnvironmentPolicyConfig& environment) {
    config_.environment = environment;
    return *this;
  }
  session::EsrSessionConfig Build() const { return config_; }

 private:
  session::EsrSessionConfig config_{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
