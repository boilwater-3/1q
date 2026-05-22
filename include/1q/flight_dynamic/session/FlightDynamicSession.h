/**
 * @file FlightDynamicSession.h
 * @brief 定义 6-DOF 飞行动力学会话门面（有状态积分器）。
 */

#ifndef FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_H_
#define FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_H_

#include <memory>

#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/maneuver/ManeuverTypes.h"
#include "1q/flight_dynamic/model/FlightDynamicInput.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"
#include "1q/api.hpp"

namespace flight_dynamic {
namespace session {
class FlightDynamicSessionFactory;
}  // namespace session
}  // namespace flight_dynamic

namespace flight_dynamic {
namespace session {

/**
 * @brief FlightDynamicSession 提供「一步一帧」的 6-DOF 飞行动力学积分门面。
 *
 * ## 与 RadarSession 的语义差异
 *
 * RadarSession::Step() 是近似无状态的传感器处理器：输入位姿+场景，输出检测/跟踪结果，
 * 各帧间相互独立（近似）。
 *
 * FlightDynamicSession::Step() 是有状态积分器：接收控制面输入，推进内部 6-DOF 状态，
 * 每步结果强依赖前一步状态。不可跳步或乱序调用。
 *
 * ## 线程安全
 *
 * 单个 FlightDynamicSession 实例不保证线程安全（与 JSBSim FGFDMExec 一致）。
 * 多载具并发仿真场景下，每个载具应持有独立的 FlightDynamicSession 实例。
 *
 * ## 使用示例
 *
 * @code
 * auto session = FlightDynamicSessionFactory::Create(config);
 *
 * for (uint32_t i = 0; i < 1000; ++i) {
 *   flight_dynamic::model::FlightDynamicInput input;
 *   input.cycle_index = i;
 *   input.dt_sec = 0.01f;
 *   input.control.throttle = 0.8;
 *   input.control.elevator = 0.0;
 *
 *   auto output = session.Step(input);
 *   if (!output.ok) { break; }
 *
 *   // 直接注入传感器模块
 *   RadarCycleInput radar_input;
 *   radar_input.platform_altitude_m = static_cast<float>(output.state.altitude_msl_m);
 *   // ... 使用 output.kinematics 通过 RadarExternalInputAdapter 填充 platform_pose
 * }
 * @endcode
 */
class ONEQ_API FlightDynamicSession {
 public:
  FlightDynamicSession();
  ~FlightDynamicSession();

  FlightDynamicSession(const FlightDynamicSession&) = delete;
  FlightDynamicSession& operator=(const FlightDynamicSession&) = delete;
  FlightDynamicSession(FlightDynamicSession&&) noexcept;
  FlightDynamicSession& operator=(FlightDynamicSession&&) noexcept;

  /**
   * @brief 推进一个积分步长。
   *
   * @param input 当前步控制输入（dt_sec、控制面、外部力）。
   * @return 积分后的状态输出；若积分失败，output.ok = false，
   *         kinematics/state 保持上一成功周期的值。
   */
  model::FlightDynamicOutput Step(const model::FlightDynamicInput& input);

  /**
   * @brief 重置内部状态到指定运动学初始条件。
   *
   * 等效于重新调用 FGFDMExec::RunIC()。常用于：
   *   - 切换任务段（如从起飞阶段切换到巡航阶段）
   *   - 测试用例间的状态隔离
   *
   * @param kinematics 新的初始运动学状态（支持 kEcef/kLla 两种位置帧）。
   */
  void Reset(const oneq::coordinate::ExternalKinematics& kinematics);

  /**
   * @brief 获取当前积分状态（不推进）。
   *
   * @return 最后一次成功 Step() 的输出，或 Reset() 后的初始状态。
   */
  model::FlightDynamicOutput GetCurrentState() const;

  /**
   * @brief 推进一个积分步长，使用机动制导自动计算控制输入。
   *
   * 根据 ManeuverRequest.mode 内部分发到对应的 Compute* 方法，
   * 自动管理机动状态（航路点索引、滚筒 PID 状态、仿真时钟等）。
   *
   * 机动切换（mode 变化）自动重置前一个机动状态。
   *
   * @param request 机动请求（模式 + 参数）。
   * @return 物理输出 + 机动状态。
   */
  maneuver::ManeuverStepResult StepManeuver(
      const maneuver::ManeuverRequest& request);

  /**
   * @brief 重置机动状态，不影响物理引擎状态。
   *
   * 清除航路点索引、滚筒 PID 状态、仿真时钟等。
   * 不调用 JSBSim RunIC。
   */
  void ResetManeuver();

 private:
  friend class FlightDynamicSessionFactory;

  struct Impl;
  explicit FlightDynamicSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace flight_dynamic

#endif  // FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_H_
