/**
 * @file EsrCycleInput.h
 * @brief 定义电子侦察模块单周期输入载荷。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
#include "1q/electronic_surveillance_radar/session/EsrSceneTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrCycleInput 描述电子侦察单周期输入。
 */
struct ONEQ_API EsrCycleInput {
  std::uint32_t cycle_index{0U};     /**< 当前周期号 */
  float dt_sec{1.0f};                /**< 当前周期步长（单位：s） */
  float platform_altitude_m{0.0f};   /**< 平台 WGS84 绝对海拔（单位：m）；启用大气物理时作为传播高度参考 */
  oneq::foundation::PoseState platform_pose{};      /**< 侦察平台局部姿态与运动状态 */
  EsrSceneEmitterList scene{};       /**< 当前周期场景辐射源输入列表 */
  EsrEnvironmentInput environment{}; /**< 本周期环境高层观测输入 */
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
