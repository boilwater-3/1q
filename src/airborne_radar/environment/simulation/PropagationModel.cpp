#include "airborne_radar/environment/simulation/PropagationModel.h"

namespace airborne_radar {
namespace environment {
namespace simulation {

PropagationResult PropagationModel::Evaluate(const EnvironmentSceneState& scene_state) const {
  PropagationResult result;
  // terrain_reflection_db 可为负值（多径增益），不对总传播损耗做下限钳位。
  result.propagation_loss_db = scene_state.base_propagation_loss_db +
                               scene_state.atmospheric_attenuation_db +
                               scene_state.terrain_reflection_db;
  result.clutter_power_db = scene_state.clutter_power_db;
  return result;
}

}  // namespace simulation
}  // namespace environment
}  // namespace airborne_radar
