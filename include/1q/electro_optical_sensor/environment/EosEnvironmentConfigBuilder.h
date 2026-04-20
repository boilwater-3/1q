/**
 * @file EosEnvironmentConfigBuilder.h
 * @brief 提供 EOS 环境默认配置链式构造器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief EosEnvironmentDefaultConfig 链式构造器。
 */
class ONEQ_API EosEnvironmentConfigBuilder {
 public:
  explicit EosEnvironmentConfigBuilder(const EosEnvironmentDefaultConfig& config = {})
      : config_(config) {}

  EosEnvironmentConfigBuilder& WithScenarioConfig(
      const EosEnvironmentScenarioConfig& scenario_config) {
    config_.scenario_config = scenario_config;
    return *this;
  }

  EosEnvironmentConfigBuilder& WithModelType(EosEnvironmentModelType model_type) {
    config_.scenario_config.model_type = model_type;
    return *this;
  }

  EosEnvironmentConfigBuilder& WithPreset(EosEnvironmentPreset preset) {
    config_.scenario_config.preset = preset;
    return *this;
  }

  EosEnvironmentConfigBuilder& WithCustomOverrides(
      const EosEnvironmentCustomOverrides& custom_overrides) {
    config_.scenario_config.has_custom_overrides = true;
    config_.scenario_config.custom_overrides = custom_overrides;
    return *this;
  }

  EosEnvironmentConfigBuilder& WithoutCustomOverrides() {
    config_.scenario_config.has_custom_overrides = false;
    config_.scenario_config.custom_overrides = EosEnvironmentCustomOverrides{};
    return *this;
  }

  EosEnvironmentConfigBuilder& WithCustomDetails(
      foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model,
      float aerosol_density_factor,
      float turbulence_factor,
      bool enable_optical_countermeasure_extension = false) {
    config_.scenario_config.has_custom_overrides = true;
    config_.scenario_config.custom_overrides.radiative_transfer_model =
        radiative_transfer_model;
    config_.scenario_config.custom_overrides.aerosol_density_factor =
        aerosol_density_factor;
    config_.scenario_config.custom_overrides.turbulence_factor =
        turbulence_factor;
    config_.scenario_config.custom_overrides.enable_optical_countermeasure_extension =
        enable_optical_countermeasure_extension;
    return *this;
  }

  EosEnvironmentDefaultConfig Build() const { return config_; }

 private:
  EosEnvironmentDefaultConfig config_{};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_BUILDER_H_
