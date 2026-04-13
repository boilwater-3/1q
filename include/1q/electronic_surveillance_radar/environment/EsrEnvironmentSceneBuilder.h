/**
 * @file EsrEnvironmentSceneBuilder.h
 * @brief ESR 环境场景链式构造器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_SCENE_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_SCENE_BUILDER_H_

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrEnvironmentSceneState 链式构造器。
 */
class ONEQ_API EsrEnvironmentSceneBuilder {
 public:
  explicit EsrEnvironmentSceneBuilder(const EsrEnvironmentSceneState& scene_state = {})
      : scene_state_(scene_state) {}

  EsrEnvironmentSceneBuilder& WithBasePropagationLossDb(float value) {
    scene_state_.base_propagation_loss_db = value;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithAtmosphericAttenuationDb(float value) {
    scene_state_.atmospheric_attenuation_db = value;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithTerrainReflectionDb(float value) {
    scene_state_.terrain_reflection_db = value;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithClutterNoiseW(float value) {
    scene_state_.clutter_noise_w = value;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithSpectrumOccupancyRatio(float value) {
    scene_state_.spectrum_occupancy_ratio = value;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithAtmosphericPhysicsConfig(
      const EsrAtmosphericPhysicsConfig& config) {
    scene_state_.atmospheric_physics = config;
    return *this;
  }

  EsrEnvironmentSceneBuilder& AddJammerSource(const EsrJammerSource& source) {
    scene_state_.jammer_sources.push_back(source);
    return *this;
  }

  EsrEnvironmentSceneState Build() const { return scene_state_; }

 private:
  EsrEnvironmentSceneState scene_state_{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_SCENE_BUILDER_H_
