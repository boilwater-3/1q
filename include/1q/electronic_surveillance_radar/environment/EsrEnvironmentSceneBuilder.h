/**
 * @file EsrEnvironmentSceneBuilder.h
 * @brief ESR 环境场景链式构造器。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_SCENE_BUILDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_SCENE_BUILDER_H_

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrEnvironmentObservation 链式构造器。
 */
class ONEQ_API EsrEnvironmentSceneBuilder {
 public:
  explicit EsrEnvironmentSceneBuilder(const EsrEnvironmentObservation& observation = {})
      : observation_(observation) {}

  EsrEnvironmentSceneBuilder& WithPropagationProfile(EsrPropagationEnvironmentProfile profile) {
    observation_.propagation_profile = profile;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithClutterDensity(EsrClutterDensityLevel level) {
    observation_.clutter_density = level;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithAtmosphericObservation(
      const EsrAtmosphericObservation& observation) {
    observation_.atmospheric_observation = observation;
    return *this;
  }

  EsrEnvironmentSceneBuilder& WithSpectrumOccupancyRatio(float value) {
    observation_.spectrum_occupancy_ratio = value;
    return *this;
  }

  EsrEnvironmentSceneBuilder& AddJammerSource(const EsrJammerSource& source) {
    observation_.jammer_sources.push_back(source);
    return *this;
  }

  EsrEnvironmentObservation Build() const { return observation_; }

 private:
  EsrEnvironmentObservation observation_{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_SCENE_BUILDER_H_
