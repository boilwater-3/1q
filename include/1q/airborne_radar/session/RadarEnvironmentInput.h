/**
 * @file RadarEnvironmentInput.h
 * @brief 定义 AR 单周期环境输入聚合类型。
 */

#ifndef AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_H_
#define AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarEnvironmentInput 聚合 AR 单周期环境事实输入。
 */
struct ONEQ_API RadarEnvironmentInput {
  environment::AtmosphericPhysicsConfig atmospheric_observation{}; /**< 当前周期气象/电离层输入 */
  environment::AtmosphericDerivedContext atmospheric_context{};    /**< 当前周期时间/空间天气输入 */
  environment::VegetationScatterPhysicsConfig surface_observation{}; /**< 当前周期地表/植被输入 */
  environment::JammerEmitterStateList jammer_sources{};              /**< 当前周期干扰源事实输入 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_H_
