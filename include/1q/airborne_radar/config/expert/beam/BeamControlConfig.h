/**
 * @file BeamControlConfig.h
 * @brief 定义 expert 波束控制配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_CONTROL_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_CONTROL_CONFIG_H_

#include "1q/airborne_radar/config/expert/beam/BeamPointingConfig.h"
#include "1q/airborne_radar/config/expert/beam/BeamSchedulerConfig.h"

namespace airborne_radar {
namespace config {
namespace expert {
namespace beam {

/**
 * @brief expert 波束控制聚合配置。
 */
struct BeamControlConfig {
  BeamPointingConfig pointing{}; /**< expert 波束指向配置。 */
  BeamSchedulerConfig scheduler{}; /**< expert 波束调度配置。 */
};

}  // namespace beam
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_CONTROL_CONFIG_H_
