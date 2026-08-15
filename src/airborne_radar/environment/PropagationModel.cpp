/**
 * @file PropagationModel.cpp
 * @brief AR 环境层传播/杂波模型薄适配层（common 单源）。
 */

#include "airborne_radar/environment/PropagationModel.h"

#include "common/radar/VegetationClutterModel.h"

namespace airborne_radar {
namespace environment {

namespace {

oneq::common::radar::VegetationCoverProfile ToCommonProfile(
    config::VegetationCoverProfile profile) {
  switch (profile) {
    case config::VegetationCoverProfile::kOpenGrassland:
      return oneq::common::radar::VegetationCoverProfile::kOpenGrassland;
    case config::VegetationCoverProfile::kSparseWoodland:
      return oneq::common::radar::VegetationCoverProfile::kSparseWoodland;
    case config::VegetationCoverProfile::kDeciduousForest:
      return oneq::common::radar::VegetationCoverProfile::kDeciduousForest;
    case config::VegetationCoverProfile::kConiferousForest:
      return oneq::common::radar::VegetationCoverProfile::kConiferousForest;
    case config::VegetationCoverProfile::kTropicalDense:
      return oneq::common::radar::VegetationCoverProfile::kTropicalDense;
    case config::VegetationCoverProfile::kDisabled:
    default:
      return oneq::common::radar::VegetationCoverProfile::kDisabled;
  }
}

oneq::common::radar::VegetationScatterPhysicsConfig ToCommonConfig(
    const config::VegetationScatterPhysicsConfig& config) {
  oneq::common::radar::VegetationScatterPhysicsConfig common_config;
  common_config.cover_profile = ToCommonProfile(config.cover_profile);
  common_config.enable_physical_model = config.enable_physical_model;
  return common_config;
}

}  // namespace

PropagationResult PropagationModel::Evaluate(const EnvironmentSceneState& scene_state) const {
  const oneq::common::radar::PropagationClutterResult common_result =
      oneq::common::radar::EvaluatePropagationClutter(
          ToCommonConfig(scene_state.vegetation_scatter_physics));
  PropagationResult result;
  result.propagation_loss_db = common_result.propagation_loss_db;
  result.clutter_power_db = common_result.clutter_power_db;
  return result;
}

}  // namespace environment
}  // namespace airborne_radar
