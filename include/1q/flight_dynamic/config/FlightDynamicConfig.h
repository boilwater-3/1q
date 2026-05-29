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
  std::string aircraft_model;
  std::string aircraft_root_dir;
  double dt_sec = 0.005;
  bool do_trim = true;
  bool silent_mode = true;
  int integrator_rate_rotational = 3;
  int integrator_rate_translational = 3;
  int integrator_pos_rotational = 1;
  int integrator_pos_translational = 4;
  int gravity_model = 1;
  InitialVelocityFrame initial_velocity_frame = InitialVelocityFrame::kBody;
  coordinate::ExternalKinematics initial_kinematics;
};

}  // namespace config
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHTDYNAMICCONFIG_H_
