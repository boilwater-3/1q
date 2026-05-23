/**
 * @file FlightDynamicOutput.h
 * @brief 定义 FlightDynamicSession::Step() 的输出结构体。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_MODEL_FLIGHT_DYNAMIC_OUTPUT_H_
#define ONEQ_FLIGHT_DYNAMIC_MODEL_FLIGHT_DYNAMIC_OUTPUT_H_

#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/model/VehicleState.h"
#include "1q/api.hpp"

namespace flight_dynamic {
namespace model {

/**
 * @brief FlightDynamicSession::Step() 的单帧输出。
 *
 * 设计目标：直接产出 ExternalKinematics 格式，让现有传感器模块的
 * adapter（RadarExternalInputAdapter 等）无需修改即可消费：
 *
 * @code
 * auto out = flight_session.Step(fd_input);
 * RadarCycleInput radar_input;
 * radar_input.platform_altitude_m = static_cast<float>(out.state.altitude_msl_m);
 * // ExternalKinematics → PoseState 由 RadarExternalInputAdapter 完成
 * @endcode
 */
struct ONEQ_API FlightDynamicOutput {
  /**
   * @brief 供传感器模块直接消费的运动学状态。
   *
   * 始终使用 kEcef 位置帧（position_frame = kEcef）。
   * attitude_deg 为 Body→ENU 欧拉角（Z-Y-X），与 1Q 惯例一致。
   */
  oneq::coordinate::ExternalKinematics kinematics{};

  /**
   * @brief 扩展状态（包含 JSBSim 特有的气动/大气参考量）。
   *
   * 传感器模块不直接消费此字段；可供数据记录、大气模型协调等用途。
   */
  VehicleState state{};

  /**
   * @brief 积分是否成功（JSBSim Run() 返回值）。
   *
   * 若为 false，kinematics 和 state 保持上一成功周期的值。
   */
  bool ok{false};
};

}  // namespace model
}  // namespace flight_dynamic

#endif  // ONEQ_FLIGHT_DYNAMIC_MODEL_FLIGHT_DYNAMIC_OUTPUT_H_
