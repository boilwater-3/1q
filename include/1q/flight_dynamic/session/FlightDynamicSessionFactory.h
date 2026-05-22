/**
 * @file FlightDynamicSessionFactory.h
 * @brief 定义 FlightDynamicSession 的公共创建入口。
 */

#ifndef FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_FACTORY_H_
#define FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_FACTORY_H_

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/session/FlightDynamicSession.h"
#include "1q/api.hpp"

namespace flight_dynamic {
namespace session {

/**
 * @brief FlightDynamicSessionFactory 负责装配并创建 FlightDynamicSession。
 *
 * 使用示例：
 * @code
 * flight_dynamic::config::FlightDynamicConfig cfg;
 * cfg.aircraft.root_dir = "/path/to/jsbsim/data";
 * cfg.aircraft.model_name = "c172";
 * cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
 * cfg.initial_kinematics.position_lla_deg_m = {39.9, 116.4, 1000.0};
 * cfg.initial_kinematics.attitude_deg = {90.0, 0.0, 0.0};  // 航向 90°
 * cfg.silent = true;
 *
 * auto session = FlightDynamicSessionFactory::Create(cfg);
 * @endcode
 */
class ONEQ_API FlightDynamicSessionFactory {
 public:
  /**
   * @brief 创建并返回一个 FlightDynamicSession 实例。
   *
   * @param config 会话配置（机型定义、初始运动学状态、积分方案等）。
   * @return 已初始化的 FlightDynamicSession（by value，move 语义）。
   * @throws std::runtime_error 若 JSBSim 模型加载失败（aircraft.root_dir 不存在或
   *         model_name 对应的 XML 文件缺失）。
   */
  static FlightDynamicSession Create(const config::FlightDynamicConfig& config = {});
};

}  // namespace session
}  // namespace flight_dynamic

#endif  // FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_FACTORY_H_
