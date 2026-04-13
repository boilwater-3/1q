/**
 * @file EnvironmentModelConfigBuilder.h
 * @brief 提供 EnvironmentModelConfig 链式构造器。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_MODEL_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_MODEL_CONFIG_BUILDER_H_

#include "1q/airborne_radar/environment/EnvironmentConfig.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief EnvironmentModelConfig 链式构造器。
 */
class EnvironmentModelConfigBuilder {
 public:
  explicit EnvironmentModelConfigBuilder(const EnvironmentModelConfig& config = {})
      : config_(config) {}

  EnvironmentModelConfigBuilder& WithBasePropagationLossDb(float value) {
    config_.base_propagation_loss_db = value;
    return *this;
  }

  EnvironmentModelConfigBuilder& WithAtmosphericAttenuationDb(float value) {
    config_.atmospheric_attenuation_db = value;
    return *this;
  }

  EnvironmentModelConfigBuilder& WithTerrainReflectionDb(float value) {
    config_.terrain_reflection_db = value;
    return *this;
  }

  EnvironmentModelConfigBuilder& WithClutterPowerDb(float value) {
    config_.clutter_power_db = value;
    return *this;
  }

  EnvironmentModelConfigBuilder& WithAtmosphericPhysicsConfig(
      const AtmosphericPhysicsConfig& config) {
    config_.atmospheric_physics = config;
    return *this;
  }

  EnvironmentModelConfigBuilder& WithVegetationScatterPhysicsConfig(
      const VegetationScatterPhysicsConfig& config) {
    config_.vegetation_scatter_physics = config;
    return *this;
  }

  EnvironmentModelConfigBuilder& WithJammerSources(const JammerSourceFactList& sources) {
    config_.jammer_sources = sources;
    return *this;
  }

  EnvironmentModelConfig Build() const { return config_; }

 private:
  EnvironmentModelConfig config_{};
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_MODEL_CONFIG_BUILDER_H_
