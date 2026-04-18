/**
 * @file BeamControlConfig.h
 * @brief 定义 semantic 波束控制配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_BEAM_CONTROL_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_BEAM_CONTROL_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief semantic 波束控制配置。
 */
struct BeamControlConfig {
  model::RadarOrientationConfig radar_orientation{}; /**< 公开方向与扫描控制配置。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_BEAM_CONTROL_CONFIG_H_
