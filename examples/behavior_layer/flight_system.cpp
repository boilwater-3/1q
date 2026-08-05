/**
 * @file flight_system.cpp
 * @brief 飞行系统实现：RoutePlan → FlightManager 适配（消费方职责）+ 运动学回退。
 *
 * ONEQ_BL_FLIGHT_DYNAMIC_ENABLED 定义时（CMake 按 ONEQ_ENABLE_FLIGHT_DYNAMIC
 * 注入）接入 flight_dynamic 真实飞行仿真（JSBSim c172x，子步进 100 ms）；
 * 关闭或初始化失败（aircraft 数据缺失/配平失败）时回退运动学近似。本文件是
 * 唯一包含 FD 头的文件，assembly/components 对 FD 零依赖（不透明持有者）。
 *
 * 平台动力学边界（冻结契约 §5）：行为层只管任务/侦察/决策，平台怎么飞是
 * 消费方世界模型的职责——本系统即该职责在示例侧的实现：航路版本变化时把
 * RoutePlanComponent 剩余航点适配为 kFlyToWaypoint 机动队列（deg→rad 单位
 * 转换属业务层适配职责，见 include/1q/navigation/RoutePoint.h），每行为周期
 * 子步进推进 FlightManager，VehicleState 映射回 FleetStatusComponent 并同步
 * 到传感器实体（recon 消费同一平台位姿）。
 */

#include "flight_system.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>

#include "1q/coordinate/types.h"
#include "assembly.h"
#include "components.h"
#include "systems.h"

#if defined(ONEQ_BL_FLIGHT_DYNAMIC_ENABLED)
#include "1q/coordinate/velocity_transform.h"
#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/guidance/Maneuver.h"
#include "1q/flight_dynamic/guidance/Waypoint.h"
#include "1q/flight_dynamic/model/VehicleState.h"
#endif

namespace behavior_layer {

#if defined(ONEQ_BL_FLIGHT_DYNAMIC_ENABLED)

/**
 * @brief 飞行动力学持有者：FlightManager + 业务层适配状态（registry ctx 持有）。
 */
struct FlightDynamicsHolder {
  explicit FlightDynamicsHolder(const oneq::flight_dynamic::config::FlightDynamicConfig& config)
      : manager(config) {}
  oneq::flight_dynamic::FlightManager manager; /**< JSBSim 飞行仿真会话 */
  double cruise_speed_mps{0.0};                 /**< 机型巡航速度（性能面派生，m/s） */
  std::uint32_t route_version{0U};              /**< 已入队航路版本（变化时重建队列） */
  bool terminated{false};                       /**< 终端状态（kCompleted/kAborted）：冻结位姿 */
};

#endif  // ONEQ_BL_FLIGHT_DYNAMIC_ENABLED

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// 平台 LLA 近似推进（运动学回退路径；FD 关闭或不可用时使用）。
void AdvancePlatformKinematics(FleetStatusComponent& fleet, double dt_s) {
  constexpr double kEarthRadiusM = 6371000.0;
  const double heading_rad = fleet.heading_deg * kDegToRad;
  const double vlat_rad_s = fleet.speed_mps * std::cos(heading_rad) / kEarthRadiusM;
  const double vlon_rad_s = fleet.speed_mps * std::sin(heading_rad) /
                            (kEarthRadiusM * std::cos(fleet.position.latitude_deg * kDegToRad));
  fleet.position.latitude_deg += vlat_rad_s * dt_s * kRadToDeg;
  fleet.position.longitude_deg += vlon_rad_s * dt_s * kRadToDeg;
}

#if defined(ONEQ_BL_FLIGHT_DYNAMIC_ENABLED)

/// 飞行子步进（s）：行为周期 1 s / 10 步 = 100 ms。c172x 巡航段 100 ms
/// 积分稳定（fd 测试用 10-20 ms 覆盖机动段；本示例仅巡航 + 航点转弯，
/// 经运行验证无失稳）；100 ms 使 200 周期 × 10 步 = 2000 步，debug 冒烟
/// 运行时可控。
constexpr double kFlightSubstepDtSec = 0.1;

/// 航点到达半径下限（m）：规划半径过小时防收敛颤振（业务层取 max）。
constexpr double kMinArrivalRadiusM = 100.0;

/// 球面大圆距离（m，haversine；与航点到达判定同量纲）。
double GreatCircleDistanceM(const oneq::flight_dynamic::model::VehicleState& state,
                            const navigation::RoutePoint& wp) {
  constexpr double kEarthRadiusM = 6371000.0;
  const double lat1 = state.latitude_rad;
  const double lon1 = state.longitude_rad;
  const double lat2 = wp.position.latitude_deg * kDegToRad;
  const double lon2 = wp.position.longitude_deg * kDegToRad;
  const double dlat = lat2 - lat1;
  const double dlon = lon2 - lon1;
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  return 2.0 * kEarthRadiusM * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

/// 把 RoutePlanComponent 剩余航点适配为 kFlyToWaypoint 机动队列（deg→rad 业务层适配）。
void PushRouteManeuvers(FlightDynamicsHolder& holder, const RoutePlanComponent& route) {
  holder.manager.ClearManeuvers();
  for (std::size_t i = route.next_index; i < route.route.size(); ++i) {
    const auto& wp = route.route[i];
    oneq::flight_dynamic::ManeuverCommand cmd;
    cmd.type = oneq::flight_dynamic::guidance::ManeuverType::kFlyToWaypoint;
    cmd.target.latitude_rad = wp.position.latitude_deg * kDegToRad;
    cmd.target.longitude_rad = wp.position.longitude_deg * kDegToRad;
    cmd.target.altitude_m = wp.position.altitude_m;
    cmd.target.radius_m = std::max(wp.radius_m, kMinArrivalRadiusM);
    cmd.target.speed_mps = wp.speed_mps > 0.0 ? wp.speed_mps : holder.cruise_speed_mps;
    holder.manager.PushManeuver(cmd);
  }
}

/// FD 路径：子步进推进 + VehicleState → FleetStatusComponent 映射 + 航点到达推进。
void AdvanceFlightDynamics(FlightDynamicsHolder& holder, entt::registry& registry,
                           entt::entity lead, FleetStatusComponent& fleet) {
  auto& route = registry.get<RoutePlanComponent>(lead);

  // 航路版本变化 → 重建机动队列（重规划语义；首次规划在 cycle 2 由
  // maneuver_system 写入，cycle 3 起入队生效）。
  if (!holder.terminated && route.version != holder.route_version) {
    holder.route_version = route.version;
    PushRouteManeuvers(holder, route);
  }

  // 子步进推进（终端状态 kCompleted/kAborted 后冻结最后位姿）。
  if (!holder.terminated) {
    constexpr int kSubstepsPerCycle = static_cast<int>(kBehaviorDtSec / kFlightSubstepDtSec);
    for (int i = 0; i < kSubstepsPerCycle; ++i) {
      if (!holder.manager.Step(kFlightSubstepDtSec)) {
        holder.terminated = true;
        break;
      }
    }
  }

  // VehicleState（弧度 LLA / 真速 / psi 航向）→ 编队状态（度制，北偏东）。
  const auto& state = holder.manager.GetVehicleState();
  fleet.position.latitude_deg = state.latitude_rad * kRadToDeg;
  fleet.position.longitude_deg = state.longitude_rad * kRadToDeg;
  fleet.position.altitude_m = state.altitude_geod_m;
  fleet.heading_deg = state.psi_rad * kRadToDeg;
  fleet.speed_mps = state.vtrue_mps;

  // 几何到达判定推进 next_index（消费方驱动语义，components.h:84；实际飞行由
  // 机动队列 FSM 串行执行，本判定仅维护进度簿记）。
  while (route.next_index < route.route.size() &&
         GreatCircleDistanceM(state, route.route[route.next_index]) <=
             std::max(route.route[route.next_index].radius_m, kMinArrivalRadiusM)) {
    ++route.next_index;
  }
  if (holder.terminated && route.next_index < route.route.size()) {
    route.next_index = route.route.size();  // 终端结束：视为航路完成
  }
}

#endif  // ONEQ_BL_FLIGHT_DYNAMIC_ENABLED

}  // namespace

void CreateFlightDynamics(entt::registry& registry, const FleetStatusComponent& initial_fleet) {
#if defined(ONEQ_BL_FLIGHT_DYNAMIC_ENABLED)
  if (GetFlightDynamics(registry) != nullptr) {
    return;  // 已创建（幂等）
  }
  oneq::flight_dynamic::config::FlightDynamicConfig cfg;
  cfg.aircraft_model = "c172x";
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kFlightSubstepDtSec;
  cfg.do_trim = true;
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m = initial_fleet.position;
  // 初始姿态：水平直飞，航向 = 编队航向（JSBSim psi 即"北偏东"航向，与
  // FleetStatusComponent.heading_deg 同约定；首周期 psi_rad 回读验证由冒烟覆盖）。
  cfg.initial_kinematics.attitude_deg.roll_deg = 0.0;
  cfg.initial_kinematics.attitude_deg.pitch_deg = 0.0;
  cfg.initial_kinematics.attitude_deg.yaw_deg = initial_fleet.heading_deg;
  // 初始速度：heading/speed → ENU → ECEF（与 recon ResolvePlatformEcef 同构）。
  const double heading_rad = initial_fleet.heading_deg * kDegToRad;
  oneq::coordinate::EnuVelocityMps enu;
  enu.east_mps = initial_fleet.speed_mps * std::sin(heading_rad);
  enu.north_mps = initial_fleet.speed_mps * std::cos(heading_rad);
  enu.up_mps = 0.0;
  if (!oneq::coordinate::TryEnuToEcefVelocity(enu, initial_fleet.position,
                                              &cfg.initial_kinematics.velocity_mps)) {
    return;  // 初始速度转换失败：回退运动学
  }
  cfg.initial_velocity_frame = oneq::flight_dynamic::config::InitialVelocityFrame::kEcef;

  auto holder = std::make_unique<FlightDynamicsHolder>(cfg);
  if (holder->manager.GetState() != oneq::flight_dynamic::FlightManagerState::kReady) {
    return;  // 初始化失败（aircraft 数据缺失/配平失败）：回退运动学
  }
  holder->cruise_speed_mps = holder->manager.GetAutopilot().GetControlProfile().cruise_speed_mps;
  if (!(holder->cruise_speed_mps > 0.0)) {
    holder->cruise_speed_mps = initial_fleet.speed_mps;  // 性能面不可用：沿用初始速度
  }
  registry.ctx().emplace<std::unique_ptr<FlightDynamicsHolder>>(std::move(holder));
#else
  (void)registry;
  (void)initial_fleet;
#endif
}

FlightDynamicsHolder* GetFlightDynamics(entt::registry& registry) {
#if defined(ONEQ_BL_FLIGHT_DYNAMIC_ENABLED)
  const auto* holder = registry.ctx().find<std::unique_ptr<FlightDynamicsHolder>>();
  return holder != nullptr ? holder->get() : nullptr;
#else
  (void)registry;
  return nullptr;
#endif
}

double FlightCruiseSpeedMps(const FlightDynamicsHolder& holder) {
#if defined(ONEQ_BL_FLIGHT_DYNAMIC_ENABLED)
  return holder.cruise_speed_mps;
#else
  (void)holder;
  return 0.0;
#endif
}

void flight_system(entt::registry& registry) {
  // 长机承载平台动力学（层级显式输入：wingman 有上级 → 零计算，冻结契约 §5）。
  entt::entity lead = entt::null;
  const auto tasking_view =
      registry.view<TaskingComponent, FleetStatusComponent, RoutePlanComponent>();
  for (const auto entity : tasking_view) {
    if (tasking_view.get<TaskingComponent>(entity).role == Role::kLead) {
      lead = entity;
      break;
    }
  }
  if (lead == entt::null) {
    return;  // 无长机（空场景）：无动力学可驱动
  }
  auto& fleet = registry.get<FleetStatusComponent>(lead);

#if defined(ONEQ_BL_FLIGHT_DYNAMIC_ENABLED)
  FlightDynamicsHolder* holder = GetFlightDynamics(registry);
  if (holder != nullptr) {
    AdvanceFlightDynamics(*holder, registry, lead, fleet);
  } else {
    static bool fallback_warned = false;
    if (!fallback_warned) {
      std::cerr << "flight_system: flight_dynamic unavailable (aircraft data missing or "
                   "init failed); falling back to kinematic advance\n";
      fallback_warned = true;
    }
    AdvancePlatformKinematics(fleet, kBehaviorDtSec);
  }
#else
  AdvancePlatformKinematics(fleet, kBehaviorDtSec);
#endif

  // 平台状态聚合注入：三传感器实体与长机共享同一平台位姿（recon 消费）。
  for (const auto entity : registry.view<SensorObservationComponent>()) {
    registry.get<FleetStatusComponent>(entity) = fleet;
  }
}

}  // namespace behavior_layer
