#ifndef AIRBORNE_RADAR_RUNTIME_COMPONENTS_AIRBORNE_RUNTIME_INPUT_H_
#define AIRBORNE_RADAR_RUNTIME_COMPONENTS_AIRBORNE_RUNTIME_INPUT_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"

namespace airborne_radar {
namespace runtime {
namespace components {

struct AirborneRuntimeInput {
  const session::RadarSceneTargetList* scene_targets{nullptr};
  model::PlatformAttitudeDeg platform_attitude{};
  float cycle_dt_sec{1.0f};
};

}  // namespace components
}  // namespace runtime
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RUNTIME_COMPONENTS_AIRBORNE_RUNTIME_INPUT_H_
