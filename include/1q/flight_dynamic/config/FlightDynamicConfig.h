#ifndef ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHTDYNAMICCONFIG_H_
#define ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHTDYNAMICCONFIG_H_

/**
 * @file FlightDynamicConfig.h
 * @brief 定义 flight_dynamic 会话配置。
 */

#include <string>

#include "1q/coordinate/types.h"

namespace oneq {
namespace flight_dynamic {
namespace config {

/**
 * @brief 初始速度向量的参考系。
 */
enum class InitialVelocityFrame {
  kBody = 0, /**< 按 JSBSim body UVW 速度注入，x 为机体系前向速度。 */
  kEcef = 1  /**< 按 ECEF 速度注入，并在初始 LLA 位置处转换为 NED 速度。 */
};

/**
 * @brief flight_dynamic 会话配置。
 */
struct FlightDynamicConfig {
  std::string aircraft_model;          /**< JSBSim 机型名称（对应 aircraft 目录下的机型） */
  std::string aircraft_root_dir;       /**< JSBSim aircraft 根目录路径 */
  double dt_sec = 0.005;               /**< 仿真步长（单位：s），默认 5 ms */
  bool do_trim = true;                 /**< 构造时是否执行初始配平 */
  bool silent_mode = true;             /**< 是否抑制 JSBSim 控制台输出 */
  int integrator_rate_rotational = 3;  /**< 角运动速率积分器类型（JSBSim 索引） */
  int integrator_rate_translational = 3; /**< 线运动速率积分器类型（JSBSim 索引） */
  int integrator_pos_rotational = 1;   /**< 角位置积分器类型（JSBSim 索引） */
  int integrator_pos_translational = 4; /**< 线位置积分器类型（JSBSim 索引） */
  int gravity_model = 1;               /**< 重力模型（JSBSim 索引） */
  InitialVelocityFrame initial_velocity_frame = InitialVelocityFrame::kBody; /**< 初始速度注入参考系 */
  coordinate::ExternalKinematics initial_kinematics; /**< 初始位置/速度/姿态 */
};

}  // namespace config
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHTDYNAMICCONFIG_H_
