#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"

namespace sbirs_sensor {
namespace session {

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithCycleIndex(std::uint32_t cycle_index) {
  input_.cycle_index = cycle_index;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithDeltaTimeSec(float dt_sec) {
  input_.dt_sec = dt_sec;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::WithSatellitePosition(
    const SbirsVector3M& position_ecef_m) {
  input_.has_satellite_position = true;
  input_.satellite_position_ecef_m = position_ecef_m;
  return *this;
}

SbirsCycleInputBuilder& SbirsCycleInputBuilder::AddTarget(const SbirsSceneTarget& target) {
  input_.scene.push_back(target);
  return *this;
}

SbirsCycleInput SbirsCycleInputBuilder::Build() const { return input_; }

}  // namespace session
}  // namespace sbirs_sensor
