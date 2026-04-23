#ifndef AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_RUNTIME_INPUT_BUILDER_H_
#define AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_RUNTIME_INPUT_BUILDER_H_

#include "1q/airborne_radar/extension/IRadarContext.h"
#include "airborne_radar/runtime/components/AirborneRuntimeInput.h"

namespace airborne_radar {
namespace runtime {
namespace components {

class RadarRuntimeInputBuilder {
 public:
  AirborneRuntimeInput Build(const extension::IRadarContext& radar_context) const;
};

}  // namespace components
}  // namespace runtime
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_RUNTIME_INPUT_BUILDER_H_
