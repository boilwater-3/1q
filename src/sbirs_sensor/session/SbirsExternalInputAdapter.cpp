#include "1q/sbirs_sensor/session/SbirsExternalInputAdapter.h"

namespace sbirs_sensor {
namespace session {

SbirsCycleInput MakeSbirsCycleInput(std::uint32_t cycle_index, float dt_sec,
                                    const SbirsVector3M& satellite_position_ecef_m,
                                    const SbirsSceneTargetList& scene) {
  SbirsCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.satellite_position_ecef_m = satellite_position_ecef_m;
  input.scene = scene;
  return input;
}

}  // namespace session
}  // namespace sbirs_sensor
