#ifndef AIRBORNE_RADAR_SESSION_RADAR_SCENE_TARGET_CONVERSION_INTERNAL_H_
#define AIRBORNE_RADAR_SESSION_RADAR_SCENE_TARGET_CONVERSION_INTERNAL_H_

#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"

namespace airborne_radar {
namespace session {

model::TargetFeature ToModelTargetFeature(const RadarSceneTarget& input);
model::TargetFeatureList ToModelTargetFeatureList(const RadarSceneTargetList& input);
RadarSceneTarget ToSceneTarget(const model::TargetFeature& input);
RadarSceneTargetList ToSceneTargetList(const model::TargetFeatureList& input);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_SCENE_TARGET_CONVERSION_INTERNAL_H_
