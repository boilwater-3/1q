/**
 * @file ArCycleInput.h
 * @brief 机载雷达单周期输入类型。
 *
 * 周期输入（周期号、场景、平台位姿、步长）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief ArCycleInput 描述单周期输入的周期号、场景、平台位姿与步长。
 */
struct ONEQ_API ArCycleInput {
  std::uint32_t cycle_index{0U};               /**< 当前周期号 */
  float dt_sec{1.0f};                          /**< 当前周期步长（单位：秒） */
  float platform_altitude_m{0.0f};             /**< 当前雷达平台 WGS84 绝对海拔（单位：m） */
  std::uint64_t platform_entity_id{0};         /**< 平台实体标识；0 仍是合法关联键 */
  oneq::foundation::PoseState platform_pose{}; /**< 当前周期平台局部位姿状态 */
  bool has_platform_ecef_kinematics{false};    /**< 是否提供工程 RF 链路所需的平台 ECEF 运动学 */
  oneq::coordinate::EcefPositionM platform_position_ecef_m{};     /**< 平台 ECEF 位置（m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_ecef_mps{}; /**< 平台 ECEF 速度（m/s） */
  ArSceneTargetList scene{};                                      /**< 当前周期场景目标输入列表 */
  bool has_environment{false};      /**< 是否提供当前周期环境高层观测快照 */
  ArEnvironmentInput environment{}; /**< 当前周期环境高层观测输入 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
