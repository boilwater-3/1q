/**
 * @file FlightDynamicInput.h
 * @brief 定义 FlightDynamicSession::Step() 的输入结构体。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_MODEL_FLIGHT_DYNAMIC_INPUT_H_
#define ONEQ_FLIGHT_DYNAMIC_MODEL_FLIGHT_DYNAMIC_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"

namespace flight_dynamic {
namespace model {

/**
 * @brief 控制面输入（归一化，范围 [-1, 1]，除非另有说明）。
 *
 * AP 指令定位为"建议"：若飞机模型无对应 AP 配置，字段值被忽略。
 * 哨兵值 < 0 表示不激活该 AP 模式。
 */
struct ONEQ_API ControlInput {
  double throttle{0.0};   ///< 油门 [0, 1]，0=慢车，1=全加力
  double aileron{0.0};    ///< 副翼，右滚为正
  double elevator{0.0};   ///< 升降舵，抬头为正
  double rudder{0.0};     ///< 方向舵，右偏为正

  // -- AP 指令（仅飞机模型有对应 autopilot 配置时生效） --
  double heading_setpoint_deg{-1.0};   ///< 目标航向 (deg, 0=北, 90=东)，<0 不激活
  bool heading_hold{false};            ///< 激活航向保持
  double altitude_setpoint_m{-1.0};    ///< 目标海拔 (m, MSL)，<0 不激活
  bool altitude_hold{false};           ///< 激活高度保持
  double airspeed_setpoint_mps{-1.0};  ///< 目标空速 (m/s, 真空速或指示空速)，<0 不激活
  bool airspeed_hold{false};           ///< 激活速度保持（自动油门）
};

/**
 * @brief 注入 JSBSim 的外部力（Body 系，SI 单位）。
 *
 * 用于模拟推力矢量、武器投放反力、外挂物分离等外部扰动。
 * 若无需注入外部力，保持默认零值即可。
 */
struct ONEQ_API ExternalForceInput {
  double force_x_n{0.0};   ///< Body x 轴力，N
  double force_y_n{0.0};   ///< Body y 轴力，N
  double force_z_n{0.0};   ///< Body z 轴力，N
  double moment_x_nm{0.0}; ///< Body x 轴力矩，N·m
  double moment_y_nm{0.0}; ///< Body y 轴力矩，N·m
  double moment_z_nm{0.0}; ///< Body z 轴力矩，N·m
};

/**
 * @brief FlightDynamicSession::Step() 的单帧输入。
 *
 * 与 RadarCycleInput 的语义差异：
 *   - RadarCycleInput 是近似无状态的传感器处理输入（每帧独立）。
 *   - FlightDynamicInput 是有状态积分器输入，每步结果依赖前一步状态。
 */
struct ONEQ_API FlightDynamicInput {
  std::uint32_t cycle_index{0U};    ///< 当前周期号
  float dt_sec{0.01f};              ///< 时间步长，单位秒（典型值：0.01 s = 100 Hz）
  ControlInput control{};           ///< 控制面输入
  ExternalForceInput ext_force{};   ///< 外部力/力矩注入（可选，默认零）
};

}  // namespace model
}  // namespace flight_dynamic

#endif  // ONEQ_FLIGHT_DYNAMIC_MODEL_FLIGHT_DYNAMIC_INPUT_H_
