// Copyright 2026. All Rights Reserved.
//
// @file PropagationModel.cpp
// @brief 实现环境层最小传播与杂波组合模型。

#include "airborne_radar/environment/simulation/PropagationModel.h"

#include <algorithm>

namespace airborne_radar {
namespace environment {
namespace simulation {

PropagationResult PropagationModel::Evaluate(
    const EnvironmentSceneState& scene_state) const {
  PropagationResult result;
  result.propagation_loss_db =
      std::max(0.0f, scene_state.base_propagation_loss_db +
                         scene_state.atmospheric_attenuation_db +
                         scene_state.terrain_reflection_db);
  result.clutter_power_db = std::max(0.0f, scene_state.clutter_power_db);
  return result;
}

} // namespace simulation
} // namespace environment
} // namespace airborne_radar
