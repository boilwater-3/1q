/**
 * @file EsrCycleInput.h
 * @brief 定义电子侦察模块单周期输入载荷。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrCycleInput 描述电子侦察单周期输入。
 */
struct ONEQ_API EsrCycleInput {
  std::uint32_t cycle_index{0U};     /**< 当前周期号 */
  double cycle_start_time_s{0.0};    /**< 当前周期绝对 world-time 起点（单位：s）。 */
  float dt_sec{1.0f};                /**< 当前周期步长（单位：s） */
  std::uint64_t platform_entity_id{0U}; /**< 接收平台实体标识；用于同平台 RF 路径判定。 */
  bool has_platform_ecef_kinematics{false}; /**< 是否提供工程 RF 链路所需的 ECEF 运动学。 */
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 接收平台 ECEF 位置（m）。 */
  oneq::coordinate::EcefVelocityMps platform_velocity_ecef_mps{}; /**< 接收平台 ECEF 速度（m/s）。 */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{}; /**< 接收设备姿态（局部 yaw/pitch/roll，单位：deg）。 */
  oneq::electromagnetics::RfEmissionFrame interference{}; /**< 当前周期全部实际 RF 发射。 */
  EsrEnvironmentInput environment{}; /**< 本周期环境高层观测输入 */
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_INPUT_H_
