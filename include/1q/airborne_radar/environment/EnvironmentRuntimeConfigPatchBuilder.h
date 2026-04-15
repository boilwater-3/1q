/**
 * @file EnvironmentRuntimeConfigPatchBuilder.h
 * @brief 提供 EnvironmentRuntimeConfigPatch 链式构造器。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_

#include "1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief EnvironmentRuntimeConfigPatch 链式构造器。
 */
class EnvironmentRuntimeConfigPatchBuilder {
 public:
  explicit EnvironmentRuntimeConfigPatchBuilder(const EnvironmentRuntimeConfigPatch& patch = {})
      : patch_(patch) {}

  EnvironmentRuntimeConfigPatchBuilder& WithScenarioConfig(
      const EnvironmentScenarioConfig& scenario_config) {
    patch_.has_scenario_config = true;
    patch_.scenario_config = scenario_config;
    return *this;
  }

  EnvironmentRuntimeConfigPatchBuilder& WithJammingDetectionThresholdDb(float value) {
    patch_.has_jamming_detection_threshold_db = true;
    patch_.jamming_detection_threshold_db = value;
    return *this;
  }

  EnvironmentRuntimeConfigPatch Build() const { return patch_; }

 private:
  EnvironmentRuntimeConfigPatch patch_{};
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
