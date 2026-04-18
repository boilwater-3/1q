/**
 * @file BeamPointingConfig.h
 * @brief 定义 expert 波束指向配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_POINTING_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_POINTING_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {
namespace expert {
namespace beam {

/**
 * @brief expert 波束指向基线配置。
 */
struct BeamPointingConfig {
  model::AzimuthElevationDeg default_scan_center_deg{}; /**< 默认扫描中心。 */
  model::CommandedBeamwidthDeg nominal_beamwidth_deg{}; /**< 名义指令态波束宽度。 */
};

}  // namespace beam
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_POINTING_CONFIG_H_
