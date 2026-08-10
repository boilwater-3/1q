/**
 * @file flight_component.h
 * @brief 自定义实体-组件示例：飞行组件（六自由度机动 + 航点跟随）。
 *
 * 组件封装 flight_dynamic 模块：ONEQ_CA_FLIGHT_DYNAMIC_ENABLED 定义时
 * （CMake 按 ONEQ_ENABLE_FLIGHT_DYNAMIC 注入）接入 JSBSim 六自由度真实
 * 飞行仿真（c172x，子步进 10 ms）。机动逻辑从起飞开始设计（参照
 * FlightManager.h Step(dt) 的 @note 集成契约）：初始地面
 * 静止（不做空中配平，do_trim=false），机动队列 = kTakeoff（滑跑→抬轮→
 * 爬升到巡航高度）→ 航路点巡航 → kLand；关闭或初始化失败（aircraft
 * 数据缺失）时回退运动学近似（模拟起飞爬升 + 巡航直线）。
 *
 * FD 头经不透明持有者（.cpp 内定义）隔离，本头文件对 FD 零依赖。
 * 每周期推进后发布 PlatformStateEvent；航点完成判定（几何距离 ≤ 到达
 * 半径）时发布 WaypointReachedEvent 并推进 next_index。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FLIGHT_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FLIGHT_COMPONENT_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/navigation/RoutePoint.h"
#include "core/component.h"

// 前向声明（FD 头零依赖约束）：ManeuverCommand 完整定义在
// 1q/flight_dynamic/FlightManager.h，仅运行时机动接口按引用传递。
namespace oneq::flight_dynamic {
struct ManeuverCommand;
}

namespace component_attachment {

/**
 * @brief 飞行组件：推进平台位姿/航向/速度，维护航点进度。
 *
 * 状态经 position()/heading_deg()/speed_mps() 供同实体传感器组件类型化
 * 读取（周期内同步数据通路）；跨周期通知经 World 信号发布事件。
 */
class FlightComponent : public Component {
 public:
  /**
   * @param[in] initial_position 机场位置（度制 LLA；FD 模式下视为地面，
   *                             从静止起飞，高度由机动序列建立）
   * @param[in] initial_heading_deg 起飞航向（deg，北偏东）
   * @param[in] initial_speed_mps 巡航速度参考（m/s；FD 模式以性能面
   *                              profile 覆盖，运动学回退路径沿用）
   * @param[in] cruise_altitude_m 巡航高度（m；FD 起飞爬升目标高度，
   *                              运动学回退路径的爬升终点）
   * @param[in] route 巡航/巡逻航路（相邻航点直线航段，驱动属本组件职责）
   * @param[in] loop_route 循环巡逻（区域巡逻场景语义）：航路飞完后从第一个
   *                       航点重新开始。运动学回退路径索引回绕；FD 模式
   *                       机动队列不加降落，kCompleted 后以当前载机状态
   *                       Reset 重建续飞（库状态机契约，见
   *                       FlightDynamics::RestartPatrol）
   */
  FlightComponent(const oneq::coordinate::LlaPositionDegM& initial_position,
                  double initial_heading_deg, double initial_speed_mps,
                  double cruise_altitude_m, std::vector<navigation::RoutePoint> route,
                  bool loop_route = false);

  ~FlightComponent() override;

  FlightComponent(const FlightComponent&) = delete;
  FlightComponent& operator=(const FlightComponent&) = delete;

  const char* Name() const override { return "Flight"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 平台当前位置（度制 LLA）。 */
  const oneq::coordinate::LlaPositionDegM& position() const { return position_; }
  /** @brief 平台航向（deg，北偏东）。 */
  double heading_deg() const { return heading_deg_; }
  /** @brief 平台速度（m/s）。 */
  double speed_mps() const { return speed_mps_; }
  /** @brief 下一航点索引。 */
  std::size_t next_waypoint_index() const { return next_index_; }
  /** @brief 航路（只读）。 */
  const std::vector<navigation::RoutePoint>& route() const { return route_; }
  /** @brief FD 真实飞行是否激活（true = JSBSim 推进；false = 运动学回退，
   *          可视化 model 列区分用）。 */
  bool fd_active() const { return fd_ != nullptr; }

  /**
   * @brief 运行时机动指令入口：FD 可用时转发 FlightManager::PushManeuver
   *        （追加机动队列；kReady 时立即派发，执行中追加的指令排队）。
   * @param[in] cmd 机动指令（字段语义按 ManeuverCommand::type 重载）。
   * @return FD 已就绪且指令已入队返回 true；FD 未启用/初始化失败（运动学
   *         回退路径）返回 false（指令被丢弃）。
   */
  bool PushManeuver(const oneq::flight_dynamic::ManeuverCommand& cmd);
  /** @brief 清空机动队列（FD 可用返回 true，否则 false）。 */
  bool ClearManeuvers();
  /** @brief 中止当前机动（FD 可用返回 true，否则 false）。 */
  bool Abort();

 private:
  /// 航点完成判定并发布 WaypointReachedEvent（含事件日志宏记录）。
  void EmitWaypointReached(World& world, std::size_t reached_index, double distance_m);
  void CheckWaypointArrival(World& world, double t_sec);
  void AdvanceKinematicsFallback(double dt_sec);

  oneq::coordinate::LlaPositionDegM position_{};
  double heading_deg_{0.0};
  double speed_mps_{0.0};
  double cruise_altitude_m_{0.0};
  std::vector<navigation::RoutePoint> route_{};
  std::size_t next_index_{0U};
  bool loop_route_{false};
  /// FD 模式航点完成事件消费游标（GetWaypointEvents 按完成顺序追加；
  /// Reset 重建清空事件记录后回落归零）。
  std::size_t waypoint_events_consumed_{0U};

  /** @brief FD 不透明持有者（定义在 .cpp，FD 头不外泄；为空时运动学回退）。 */
  class FlightDynamics;
  std::unique_ptr<FlightDynamics> fd_{};
  Entity* host_{nullptr};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FLIGHT_COMPONENT_H_
