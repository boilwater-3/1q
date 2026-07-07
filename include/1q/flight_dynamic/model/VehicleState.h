/**
 * @file VehicleState.h
 * @brief 定义 flight_dynamic 对外暴露的载机运动学/姿态状态快照。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATE_H_
#define ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATE_H_

#include "1q/coordinate/types.h"

namespace oneq {
namespace flight_dynamic {
namespace model {

/**
 * @brief 载机仿真状态快照（POD）。
 *
 * 由 JSBSim 状态映射得到，每个仿真步由 FlightManager::GetVehicleState() 返回。
 * 所有量均使用 SI 单位，姿态/角速率以弧度计；速度 u/v/w 为机体系分量。
 */
struct VehicleState {
  double sim_time_sec = 0.0;  /**< 仿真累计时间（单位：s） */

  // WGS84 position
  double latitude_rad = 0.0;  /**< 纬度（单位：rad） */
  double longitude_rad = 0.0; /**< 经度（单位：rad） */
  double altitude_geod_m = 0.0; /**< 大地高（单位：m，ASL） */
  double altitude_agl_m = 0.0;  /**< 相对地面高（单位：m，AGL） */

  // Velocity (body frame, m/s)
  double u_mps = 0.0; /**< 机体系前向速度（单位：m/s） */
  double v_mps = 0.0; /**< 机体系侧向速度（单位：m/s） */
  double w_mps = 0.0; /**< 机体系垂向速度（单位：m/s） */

  // Velocity (inertial magnitude, m/s)
  double v_inertial_mps = 0.0; /**< 惯性速度大小（单位：m/s） */

  // Airspeed
  double vc_mps = 0.0;   /**< 校准空速 CAS（单位：m/s） */
  double vtrue_mps = 0.0; /**< 真空速 TAS（单位：m/s） */
  double mach = 0.0;     /**< 马赫数（无量纲） */

  // Attitude (Euler, rad)
  double phi_rad = 0.0;   /**< 滚转角（单位：rad） */
  double theta_rad = 0.0; /**< 俯仰角（单位：rad） */
  double psi_rad = 0.0;   /**< 偏航角/航向（单位：rad） */

  // Body angular rates (rad/s)
  double p_rad_s = 0.0; /**< 滚转角速率 p（单位：rad/s） */
  double q_rad_s = 0.0; /**< 俯仰角速率 q（单位：rad/s） */
  double r_rad_s = 0.0; /**< 偏航角速率 r（单位：rad/s） */

  // Body accelerations (m/s^2)
  double ax_mps2 = 0.0; /**< 机体 X 向加速度（单位：m/s^2） */
  double ay_mps2 = 0.0; /**< 机体 Y 向加速度（单位：m/s^2） */
  double az_mps2 = 0.0; /**< 机体 Z 向加速度（单位：m/s^2） */

  // Angular accelerations (rad/s^2)
  double pdot_rad_s2 = 0.0; /**< 滚转角加速度（单位：rad/s^2） */
  double qdot_rad_s2 = 0.0; /**< 俯仰角加速度（单位：rad/s^2） */
  double rdot_rad_s2 = 0.0; /**< 偏航角加速度（单位：rad/s^2） */

  // Mass properties
  double mass_kg = 0.0;    /**< 载机质量（单位：kg） */
  double weight_lbs = 0.0; /**< 载机重量（单位：lbs） */

  // Atmosphere
  double rho_kg_m3 = 0.0; /**< 大气密度（单位：kg/m^3） */
  double qbar_pa = 0.0;   /**< 动压（单位：Pa） */
};

}  // namespace model
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATE_H_
