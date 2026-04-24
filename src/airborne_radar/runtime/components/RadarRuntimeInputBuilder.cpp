#include "airborne_radar/runtime/components/RadarRuntimeInputBuilder.h"

namespace airborne_radar {
namespace runtime {
namespace components {

AirborneRuntimeInput RadarRuntimeInputBuilder::Build(
    const extension::IRadarContext& radar_context) const {
  AirborneRuntimeInput input;
  input.scene_targets = &radar_context.GetSceneTargets();
  input.platform_attitude = radar_context.GetPlatformAttitude();
  input.cycle_dt_sec = radar_context.GetCycleDeltaTimeSec();
  return input;
}

}  // namespace components
}  // namespace runtime
}  // namespace airborne_radar
