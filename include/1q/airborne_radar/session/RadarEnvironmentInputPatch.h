/**
 * @file RadarEnvironmentInputPatch.h
 * @brief 定义 AR 单周期环境输入状态的局部更新载荷。
 */

#ifndef AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_PATCH_H_
#define AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_PATCH_H_

#include "1q/airborne_radar/session/RadarEnvironmentInput.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarEnvironmentInputPatch 表示调用方侧环境事实状态的局部更新。
 *
 * @note 本类型不直接进入 RadarSession::StepWithResult()。调用方应先用
 *       RadarEnvironmentInputState 合成完整 RadarEnvironmentInput 快照，再写入
 *       RadarCycleInput::environment。
 */
struct ONEQ_API RadarEnvironmentInputPatch {
  bool has_atmospheric_observation{false};                         /**< 是否更新气象/电离层输入 */
  environment::AtmosphericPhysicsConfig atmospheric_observation{}; /**< 新气象/电离层输入 */
  bool has_atmospheric_context{false};                             /**< 是否更新时间/空间天气输入 */
  environment::AtmosphericDerivedContext atmospheric_context{};    /**< 新时间/空间天气输入 */
  bool has_surface_observation{false};                             /**< 是否更新地表/植被输入 */
  environment::VegetationScatterPhysicsConfig surface_observation{}; /**< 新地表/植被输入 */
  bool has_jammer_sources{false};                                    /**< 是否更新干扰源列表 */
  environment::JammerEmitterStateList jammer_sources{};              /**< 新干扰源列表 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_PATCH_H_
