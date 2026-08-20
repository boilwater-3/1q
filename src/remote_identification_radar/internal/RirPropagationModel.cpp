/**
 * @file RirPropagationModel.cpp
 * @brief RIR 环境层传播/杂波模型薄适配层（common 单源）。
 */

#include "remote_identification_radar/internal/RirPropagationModel.h"

#include "common/radar/VegetationClutterModel.h"

namespace remote_identification_radar {
namespace internal {

namespace {

oneq::common::radar::VegetationCoverProfile ToCommonProfile(
    config::RirVegetationCoverProfile profile) {
  switch (profile) {
    case config::RirVegetationCoverProfile::kOpenGrassland:
      return oneq::common::radar::VegetationCoverProfile::kOpenGrassland;
    case config::RirVegetationCoverProfile::kSparseWoodland:
      return oneq::common::radar::VegetationCoverProfile::kSparseWoodland;
    case config::RirVegetationCoverProfile::kDeciduousForest:
      return oneq::common::radar::VegetationCoverProfile::kDeciduousForest;
    case config::RirVegetationCoverProfile::kConiferousForest:
      return oneq::common::radar::VegetationCoverProfile::kConiferousForest;
    case config::RirVegetationCoverProfile::kTropicalDense:
      return oneq::common::radar::VegetationCoverProfile::kTropicalDense;
    case config::RirVegetationCoverProfile::kDisabled:
    default:
      return oneq::common::radar::VegetationCoverProfile::kDisabled;
  }
}

oneq::common::radar::VegetationScatterPhysicsConfig ToCommonConfig(
    const config::RirVegetationScatterPhysicsConfig& config) {
  oneq::common::radar::VegetationScatterPhysicsConfig common_config;
  common_config.cover_profile = ToCommonProfile(config.cover_profile);
  common_config.enable_physical_model = config.enable_physical_model;
  return common_config;
}

}  // namespace

RirPropagationResult RirPropagationModel::Evaluate(
    const RirEnvironmentSceneState& scene_state) const {
  const oneq::common::radar::PropagationClutterResult common_result =
      oneq::common::radar::EvaluatePropagationClutter(
          ToCommonConfig(scene_state.vegetation_scatter_physics));
  RirPropagationResult result;
  result.propagation_loss_db = common_result.propagation_loss_db;
  result.clutter_power_db = common_result.clutter_power_db;
  return result;
}

}  // namespace internal
}  // namespace remote_identification_radar
