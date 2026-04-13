/**
 * @file EsrEnvironmentConfigBuilder.h
 * @brief 提供 ESR 环境默认配置链式构造器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_BUILDER_H_

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrEnvironmentDefaultConfig 链式构造器。
 */
class ONEQ_API EsrEnvironmentConfigBuilder {
 public:
  explicit EsrEnvironmentConfigBuilder(const EsrEnvironmentDefaultConfig& config = {})
      : config_(config) {}

  EsrEnvironmentConfigBuilder& WithModelConfig(const EsrEnvironmentModelConfig& model_config) {
    config_.model_config = model_config;
    return *this;
  }

  EsrEnvironmentConfigBuilder& WithDefaultClutterNoiseW(float value) {
    config_.model_config.default_clutter_noise_w = value;
    return *this;
  }

  EsrEnvironmentConfigBuilder& WithJammingDetectionThresholdW(float value) {
    config_.model_config.jamming_detection_threshold_w = value;
    return *this;
  }

  EsrEnvironmentConfigBuilder& WithAtmosphericPhysicsConfig(
      const EsrAtmosphericPhysicsConfig& config) {
    config_.model_config.atmospheric_physics = config;
    return *this;
  }

  EsrEnvironmentDefaultConfig Build() const { return config_; }

 private:
  EsrEnvironmentDefaultConfig config_{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_CONFIG_BUILDER_H_
