/**
 * @file VehicleState.h
 * @brief 定义 JSBSim 飞行动力学积分后的载具完整状态快照。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLE_STATE_H_
#define ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLE_STATE_H_

#include "1q/api.hpp"

namespace flight_dynamic {
namespace model {

/**
 * @brief 载具 6-DOF 状态快照（SI 单位，1Q 坐标系）。
 *
 * 所有物理量均已从 JSBSim 内部的英制/NED 系转换为 1Q 标准：
 *   - 位置: ECEF，单位 m
 *   - 速度: ECEF，单位 m/s
 *   - 姿态: Body→ENU 欧拉角（Z-Y-X，单位 deg）
 *   - 角速度: Body 系，单位 rad/s
 *   - 加速度: Body 系，单位 m/s²
 */
struct ONEQ_API VehicleState {
  // ----- 位置 (ECEF, m) -----
  double position_ecef_x_m{0.0};
  double position_ecef_y_m{0.0};
  double position_ecef_z_m{0.0};

  // ----- 速度 (ECEF, m/s) -----
  double velocity_ecef_x_mps{0.0};
  double velocity_ecef_y_mps{0.0};
  double velocity_ecef_z_mps{0.0};

  // ----- 姿态 (Body→ENU 欧拉角, Z-Y-X, deg) -----
  double yaw_deg{0.0};    ///< 偏航角（航向），正北为 0，顺时针为正
  double pitch_deg{0.0};  ///< 俯仰角，正抬头为正
  double roll_deg{0.0};   ///< 滚转角，右滚为正

  // ----- 角速度 (Body 系, rad/s) -----
  double roll_rate_radps{0.0};   ///< p — 绕 x 轴
  double pitch_rate_radps{0.0};  ///< q — 绕 y 轴
  double yaw_rate_radps{0.0};    ///< r — 绕 z 轴

  // ----- 加速度 (Body 系, m/s²) -----
  double acc_x_mps2{0.0};
  double acc_y_mps2{0.0};
  double acc_z_mps2{0.0};

  // ----- 大气参考量 -----
  double altitude_msl_m{0.0};     ///< 海拔高度（MSL），单位 m
  double airspeed_mps{0.0};       ///< 空速，单位 m/s
  double ground_speed_mps{0.0};   ///< 地速，单位 m/s
  double mach{0.0};               ///< 马赫数
  double alpha_deg{0.0};          ///< 迎角 (AoA)，单位 deg
  double beta_deg{0.0};           ///< 侧滑角，单位 deg
};

}  // namespace model
}  // namespace flight_dynamic

#endif  // ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLE_STATE_H_
