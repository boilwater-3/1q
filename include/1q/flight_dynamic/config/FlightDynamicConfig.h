/**
 * @file FlightDynamicConfig.h
 * @brief 定义 FlightDynamicSession 会话级配置。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHT_DYNAMIC_CONFIG_H_
#define ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHT_DYNAMIC_CONFIG_H_

#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/config/AircraftDefinition.h"
#include "1q/api.hpp"

namespace flight_dynamic {
namespace config {

/**
 * @brief JSBSim 积分方案选项。
 */
enum class IntegratorType {
  kEuler = 0,           ///< 一阶欧拉（最快，精度最低）
  kTrapezoidal,         ///< 梯形法
  kAdamsBashforth2,     ///< Adams-Bashforth 2 阶
  kAdamsBashforth3,     ///< Adams-Bashforth 3 阶
  kAdamsBashforth4,     ///< Adams-Bashforth 4 阶（默认）
  kAdamsBashforth5,     ///< Adams-Bashforth 5 阶
};

/**
 * @brief FlightDynamicSession 会话配置。
 */
struct ONEQ_API FlightDynamicConfig {
  /** @brief 飞行器机型定义（必填）。 */
  AircraftDefinition aircraft{};

  /**
   * @brief 初始运动学状态。
   *
   * position_frame 决定使用 position_ecef_m 还是 position_lla_deg_m；
   * velocity_mps 始终为 ECEF 系；
   * attitude_deg 为 Body→ENU 欧拉角（Z-Y-X）。
   */
  oneq::coordinate::ExternalKinematics initial_kinematics{};

  /** @brief 积分方案，默认 Adams-Bashforth 4 阶。 */
  IntegratorType integrator{IntegratorType::kAdamsBashforth4};

  /**
   * @brief 是否将 JSBSim 控制台日志输出重定向到 /dev/null（静默模式）。
   *
   * 生产环境建议设为 true，调试时可设为 false 以观察 JSBSim 内部日志。
   */
  bool silent{true};

  /**
   * @brief 仿真开始前是否自动执行 JSBSim 配平 (Trim)。
   *
   * 空中重置（Mid-air spawn）且有初始速度时必须设为 true，否则初始不平衡力矩会导致 NaNs。
   */
  bool do_trim{true};
};

}  // namespace config
}  // namespace flight_dynamic

#endif  // ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHT_DYNAMIC_CONFIG_H_
