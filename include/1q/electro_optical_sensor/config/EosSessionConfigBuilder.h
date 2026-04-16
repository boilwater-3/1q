/**
 * @file EosSessionConfigBuilder.h
 * @brief EOS 对外配置入口：初始化配置类型与 SessionConfig Builder。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosSessionConfigBuilder 提供初始化配置的链式构造。
 */
class ONEQ_API EosSessionConfigBuilder {
 public:
  explicit EosSessionConfigBuilder(
      const session::EosSessionConfig& config = {}) noexcept : config_(config) {}

  EosSessionConfigBuilder& WithSessionConfig(
      const session::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }

  EosSessionConfigBuilder& WithOpticalHardware(
      const EosOpticalHardwareConfig& optical) noexcept {
    config_.optical = optical;
    return *this;
  }

  EosSessionConfigBuilder& WithScanPolicy(const EosScanPolicyConfig& scan) noexcept {
    config_.scan = scan;
    return *this;
  }

  EosSessionConfigBuilder& WithPointing(const EosPointingConfig& pointing) noexcept {
    config_.pointing = pointing;
    return *this;
  }

  EosSessionConfigBuilder& WithDetectionProfile(EosDetectionProfile profile) noexcept {
    config_.detection.profile = profile;
    return *this;
  }

  EosSessionConfigBuilder& WithStrayLightProfile(EosStrayLightProfile profile) noexcept {
    config_.stray_light.profile = profile;
    return *this;
  }

  EosSessionConfigBuilder& WithEnvironmentPolicy(
      const EosEnvironmentPolicyConfig& environment) noexcept {
    config_.environment = environment;
    return *this;
  }

  EosSessionConfigBuilder& WithWorkMode(session::EosWorkMode mode) noexcept {
    config_.scan.work_mode = mode;
    return *this;
  }

  EosSessionConfigBuilder& WithScanRateDegPerSec(float value) noexcept {
    config_.scan.scan_rate_deg_per_sec = value;
    return *this;
  }

  EosSessionConfigBuilder& WithFrameRateHz(float value) noexcept {
    config_.scan.frame_rate_hz = value;
    return *this;
  }

  EosSessionConfigBuilder& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    config_.environment.model_type = model_type;
    return *this;
  }

  EosSessionConfigBuilder& WithEnvironmentPreset(EosEnvironmentPreset preset) noexcept {
    config_.environment.preset = preset;
    return *this;
  }

  session::EosSessionConfig Build() const noexcept { return config_; }

 private:
  session::EosSessionConfig config_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
