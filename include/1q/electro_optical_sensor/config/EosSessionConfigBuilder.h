/**
 * @file EosSessionConfigBuilder.h
 * @brief EOS 语义式会话配置构造器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosSessionConfigBuilder 提供语义化会话配置构造入口。
 * @note 该构造器用于 work mode/profile/preset 等高层语义输入。
 *       如需直接编辑 hardware/mission/policy/environment 四域详细参数，
 *       请使用 EosDetailedSessionConfigBuilder。
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

  EosSessionConfigBuilder& WithWorkMode(session::EosWorkMode mode) noexcept {
    config_.mission.work_mode = mode;
    return *this;
  }

  EosSessionConfigBuilder& WithDetectionProfile(EosDetectionProfile profile) noexcept {
    config_.policy.detection.profile = profile;
    config_.policy.detection.use_profile_defaults = true;
    return *this;
  }

  EosSessionConfigBuilder& WithStrayLightProfile(EosStrayLightProfile profile) noexcept {
    config_.policy.stray_light.profile = profile;
    config_.policy.stray_light.use_profile_defaults = true;
    return *this;
  }

  EosSessionConfigBuilder& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    config_.environment.scenario_config.model_type = model_type;
    return *this;
  }

  EosSessionConfigBuilder& WithEnvironmentPreset(EosEnvironmentPreset preset) noexcept {
    config_.environment.scenario_config.preset = preset;
    return *this;
  }

  session::EosSessionConfig Build() const noexcept { return config_; }

 private:
  session::EosSessionConfig config_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
