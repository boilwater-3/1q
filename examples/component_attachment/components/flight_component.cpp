/**
 * @file flight_component.cpp
 * @brief 飞行组件实现：六自由度真实飞行（JSBSim）与运动学回退两路径。
 *
 * 1. ONEQ_CA_FLIGHT_DYNAMIC_ENABLED 定义时以六自由度机动仿真推进（子步进
 *    10 ms，FlightManager.h Step(dt) @note 集成契约：初始地面静止不做空中配平，
 *    机动队列 = 起飞 → 航路点巡航 → 降落），VehicleState 映射回度制状态；
 * 2. 否则回退运动学近似（模拟起飞爬升 + 巡航航点寻的，段间瞬时转向）；
 * 3. 航点完成判定统一用几何距离（haversine ≤ max(radius, 100 m)）；
 * 4. 循环巡逻（loop_route）：航路耗尽后回绕首个航点继续（运动学索引回绕 /
 *    FD 重建无起飞降落的航点队列）。
 */

#include "flight_component.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

#include "1q/coordinate/position_transform.h"
#include "core/events.h"
#include "core/world.h"
#include "logger/logger.h"

#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
#include "1q/coordinate/velocity_transform.h"
#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/guidance/Maneuver.h"
#include "1q/flight_dynamic/guidance/WaypointSequencingEvent.h"
#include "1q/flight_dynamic/model/VehicleState.h"
#endif

namespace component_attachment {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371000.0;

/// 航点到达半径下限（m）：规划半径过小时防收敛颤振（业务层取 max）。
constexpr double kMinArrivalRadiusM = 100.0;

/// 平台 LLA 水平推进（运动学回退路径；FD 关闭或不可用时使用）。
void AdvanceKinematics(oneq::coordinate::LlaPositionDegM* position, double heading_deg,
                       double speed_mps, double dt_s) {
  const double heading_rad = heading_deg * kDegToRad;
  const double vlat_rad_s =
      speed_mps * std::cos(heading_rad) / kEarthRadiusM;
  const double vlon_rad_s = speed_mps * std::sin(heading_rad) /
                            (kEarthRadiusM * std::cos(position->latitude_deg * kDegToRad));
  position->latitude_deg += vlat_rad_s * dt_s * kRadToDeg;
  position->longitude_deg += vlon_rad_s * dt_s * kRadToDeg;
}

/// 球面大圆距离（m，haversine；与航点到达判定同量纲，忽略高度差）。
double GreatCircleDistanceM(const oneq::coordinate::LlaPositionDegM& from,
                            const navigation::RoutePoint& wp) {
  const double lat1 = from.latitude_deg * kDegToRad;
  const double lon1 = from.longitude_deg * kDegToRad;
  const double lat2 = wp.position.latitude_deg * kDegToRad;
  const double lon2 = wp.position.longitude_deg * kDegToRad;
  const double dlat = lat2 - lat1;
  const double dlon = lon2 - lon1;
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  return 2.0 * kEarthRadiusM * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

}  // namespace

#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)

/// 飞行子步进（s）：行为周期 1 s / 100 步 = 10 ms。六自由度机动含地面
/// 滑跑/起落架动力学（快动态），100 ms 步长会失控发散（起飞段 roll 可达
/// 180° 量级后数值崩溃）；10 ms 与 FlightManager.h Step(dt) @note 集成
/// 契约一致（回归见 fd_takeoff_substep_test）。
constexpr double kFlightSubstepDtSec = 0.01;
constexpr int kSubstepsPerCycle = static_cast<int>(1.0 / kFlightSubstepDtSec);

/**
 * @brief FD 持有者：FlightManager + 巡航参数（定义在 .cpp，FD 头不外泄）。
 */
class FlightComponent::FlightDynamics {
 public:
  FlightDynamics(const oneq::coordinate::LlaPositionDegM& airfield_position,
                 double heading_deg, double speed_mps, double cruise_altitude_m,
                 const std::vector<navigation::RoutePoint>& route, bool loop)
      : manager_(MakeConfig(airfield_position, heading_deg)) {
    cruise_speed_mps_ = manager_.GetAutopilot().GetControlProfile().cruise_speed_mps;
    if (!(cruise_speed_mps_ > 0.0)) {
      cruise_speed_mps_ = speed_mps;  // 性能面不可用：沿用巡航速度参考
    }
    PushMission(cruise_altitude_m, heading_deg, route, loop);
  }

  /// 构造成功与否：kReady（就绪）或 kExecuting（已派发首个机动）均可用；
  /// 失败由调用方回退运动学。
  bool ready() const {
    const auto state = manager_.GetState();
    return state == oneq::flight_dynamic::FlightManagerState::kReady ||
           state == oneq::flight_dynamic::FlightManagerState::kExecuting;
  }

  /// 终端状态（kCompleted/kAborted）：机动队列全部结束或中止。
  bool terminated() const {
    const auto state = manager_.GetState();
    return state == oneq::flight_dynamic::FlightManagerState::kCompleted ||
           state == oneq::flight_dynamic::FlightManagerState::kAborted;
  }

  /// 当前状态（诊断透出）。
  int state_int() const { return static_cast<int>(manager_.GetState()); }

  /// 初始化失败原因（诊断透出，供回退告警）。
  const std::string& failure_reason() const {
    return manager_.GetDiagnostics().last_failure_reason;
  }

  /// 仅航点机动队列（无起飞/降落）：RestartPatrol 续飞用。
  void PushWaypointsOnly(const std::vector<navigation::RoutePoint>& route) {
    manager_.ClearManeuvers();
    for (const auto& wp : route) {
      oneq::flight_dynamic::ManeuverCommand cmd;
      cmd.type = oneq::flight_dynamic::guidance::ManeuverType::kFlyToWaypoint;
      cmd.target.latitude_rad = wp.position.latitude_deg * kDegToRad;
      cmd.target.longitude_rad = wp.position.longitude_deg * kDegToRad;
      cmd.target.altitude_m = wp.position.altitude_m;
      cmd.target.radius_m = std::max(wp.radius_m, kMinArrivalRadiusM);
      cmd.target.speed_mps = wp.speed_mps > 0.0 ? wp.speed_mps : cruise_speed_mps_;
      manager_.PushManeuver(cmd);
    }
  }

  /// 机动队列：kTakeoff（滑跑→抬轮→爬升）→ 航路点巡航 → kLand。
  /// 循环巡逻（loop=true）时省略 kLand（巡逻不降落，队列完成后由
  /// RestartPatrol 以当前状态 Reset 重建续飞）。
  /// @param[in] takeoff_heading_deg 起飞目标航向（deg，北偏东）：kTakeoff 的
  ///           target.latitude_rad 即目标航向（库契约，默认 0 = 正北）。
  void PushMission(double cruise_altitude_m, double takeoff_heading_deg,
                   const std::vector<navigation::RoutePoint>& route, bool loop) {
    manager_.ClearManeuvers();
    oneq::flight_dynamic::ManeuverCommand takeoff;
    takeoff.type = oneq::flight_dynamic::guidance::ManeuverType::kTakeoff;
    takeoff.target.altitude_m = cruise_altitude_m;
    takeoff.target.latitude_rad = takeoff_heading_deg * kDegToRad;  // 起飞目标航向
    manager_.PushManeuver(takeoff);
    PushWaypointsOnly(route);
    if (!loop && !route.empty()) {
      // 降落目标 = 航路终点（场景简化：着陆点取最后航点位置）。
      oneq::flight_dynamic::ManeuverCommand land;
      land.type = oneq::flight_dynamic::guidance::ManeuverType::kLand;
      land.target.latitude_rad = route.back().position.latitude_deg * kDegToRad;
      land.target.longitude_rad = route.back().position.longitude_deg * kDegToRad;
      land.target.altitude_m = 0.0;
      land.value = 0.0;
      manager_.PushManeuver(land);
    }
  }

  /// 循环巡逻队列重建：kCompleted/kAborted 后以**当前载机状态** Reset 重建
  /// 会话（库状态机契约：kCompleted/kAborted 必须 Reset() 恢复 kReady，
  /// PushManeuver 不会重启执行），随后重推航点机动队列（无起飞/降落——
  /// 平台已在巡航）。初始运动学取自 VehicleState（大地高/psi 航向/真空速 →
  /// ECEF 速度、姿态原样注入），do_trim=false 不做空中配平（权威用法：
  /// 配平求解不稳定）。
  void RestartPatrol(const std::vector<navigation::RoutePoint>& route) {
    const auto& s = manager_.GetVehicleState();
    oneq::flight_dynamic::config::FlightDynamicConfig cfg;
    cfg.aircraft_model = "c172x";
    cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    cfg.dt_sec = kFlightSubstepDtSec;
    cfg.do_trim = false;
    cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
    cfg.initial_kinematics.position_lla_deg_m.latitude_deg = s.latitude_rad * kRadToDeg;
    cfg.initial_kinematics.position_lla_deg_m.longitude_deg = s.longitude_rad * kRadToDeg;
    cfg.initial_kinematics.position_lla_deg_m.altitude_m = s.altitude_geod_m;
    cfg.initial_velocity_frame =
        oneq::flight_dynamic::config::InitialVelocityFrame::kEcef;
    oneq::coordinate::TryMakeEcefVelocityFromHeading(
        s.psi_rad * kRadToDeg, s.vtrue_mps, cfg.initial_kinematics.position_lla_deg_m,
        &cfg.initial_kinematics.velocity_mps);
    cfg.initial_kinematics.attitude_deg.roll_deg = s.phi_rad * kRadToDeg;
    cfg.initial_kinematics.attitude_deg.pitch_deg = s.theta_rad * kRadToDeg;
    cfg.initial_kinematics.attitude_deg.yaw_deg = s.psi_rad * kRadToDeg;
    manager_.Reset(cfg);
    PushWaypointsOnly(route);
  }

  /// kFlyToWaypoint 完成事件记录（决策快照，见 FlightManager::GetWaypointEvents）。
  const std::vector<oneq::flight_dynamic::guidance::WaypointSequencingEvent>&
  waypoint_events() const {
    return manager_.GetWaypointEvents();
  }

  /// 子步进推进；返回是否仍在运行（kCompleted/kAborted 后 false）。
  bool Step(double dt_sec) { return manager_.Step(dt_sec); }

  /// 运行时机动指令入口（追加队列；kReady 立即派发，执行中追加排队）。
  void PushManeuver(const oneq::flight_dynamic::ManeuverCommand& cmd) {
    manager_.PushManeuver(cmd);
  }

  /// 清空机动队列并复位执行索引（不影响当前载机状态）。
  void ClearManeuvers() { manager_.ClearManeuvers(); }

  /// 中止当前机动，释放自动驾驶保持，状态转为 kAborted。
  void Abort() { manager_.Abort(); }

  /// 当前飞行状态（弧度 LLA / 真速 / psi 航向，北偏东）。
  const oneq::flight_dynamic::model::VehicleState& state() const {
    return manager_.GetVehicleState();
  }

 private:
  /// 起飞场景配置：地面静止（不做空中配平），姿态水平，零速度。
  static oneq::flight_dynamic::config::FlightDynamicConfig MakeConfig(
      const oneq::coordinate::LlaPositionDegM& airfield_position, double heading_deg) {
    oneq::flight_dynamic::config::FlightDynamicConfig cfg;
    cfg.aircraft_model = "c172x";
    cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    cfg.dt_sec = kFlightSubstepDtSec;
    cfg.do_trim = false;  // 六自由度机动从起飞开始：不做空中配平
    cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
    cfg.initial_kinematics.position_lla_deg_m.latitude_deg = airfield_position.latitude_deg;
    cfg.initial_kinematics.position_lla_deg_m.longitude_deg = airfield_position.longitude_deg;
    cfg.initial_kinematics.position_lla_deg_m.altitude_m = 0.0;  // 地面
    // 初始姿态：水平直飞，航向 = 起飞航向（JSBSim psi 即"北偏东"航向）。
    cfg.initial_kinematics.attitude_deg.roll_deg = 0.0;
    cfg.initial_kinematics.attitude_deg.pitch_deg = 0.0;
    cfg.initial_kinematics.attitude_deg.yaw_deg = heading_deg;
    // 初始速度：静止（起飞滑跑由 kTakeoff 机动驱动）。
    cfg.initial_kinematics.velocity_mps.x_mps = 0.0;
    cfg.initial_kinematics.velocity_mps.y_mps = 0.0;
    cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
    return cfg;
  }

  oneq::flight_dynamic::FlightManager manager_;
  double cruise_speed_mps_{0.0};
};

#endif  // ONEQ_CA_FLIGHT_DYNAMIC_ENABLED

#if !defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
/// FD 未启用：空壳类型（满足 unique_ptr 成员的完整类型需求；成员恒为空，
/// 组件走运动学回退路径）。
class FlightComponent::FlightDynamics {};
#endif

FlightComponent::FlightComponent(const oneq::coordinate::LlaPositionDegM& initial_position,
                                 double initial_heading_deg, double initial_speed_mps,
                                 double cruise_altitude_m,
                                 std::vector<navigation::RoutePoint> route, bool loop_route)
    : position_(initial_position),
      heading_deg_(initial_heading_deg),
      speed_mps_(initial_speed_mps),
      cruise_altitude_m_(cruise_altitude_m),
      route_(std::move(route)),
      loop_route_(loop_route) {
#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  auto fd = std::make_unique<FlightDynamics>(position_, heading_deg_, speed_mps_,
                                             cruise_altitude_m_, route_, loop_route_);
  if (fd->ready()) {
    fd_ = std::move(fd);
  } else {
    std::cerr << "FlightComponent: flight_dynamic init failed: state=" << fd->state_int()
              << " reason='" << fd->failure_reason()
              << "'; falling back to kinematic advance\n";
  }
#else
  (void)0;  // FD 未启用：运动学回退
#endif
}

FlightComponent::~FlightComponent() = default;

bool FlightComponent::PushManeuver(const oneq::flight_dynamic::ManeuverCommand& cmd) {
#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  if (fd_ != nullptr) {
    fd_->PushManeuver(cmd);
    return true;
  }
#endif
  (void)cmd;  // FD 不可用（未启用/初始化失败）：机动指令无效
  return false;
}

bool FlightComponent::ClearManeuvers() {
#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  if (fd_ != nullptr) {
    fd_->ClearManeuvers();
    return true;
  }
#endif
  return false;
}

bool FlightComponent::Abort() {
#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  if (fd_ != nullptr) {
    fd_->Abort();
    return true;
  }
#endif
  return false;
}

void FlightComponent::Step(World& world, double dt_sec) {
#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  if (fd_ != nullptr) {
    // 子步进推进（终端状态后冻结最后位姿：Step 返回 false 即停止推进）。
    for (int i = 0; i < kSubstepsPerCycle; ++i) {
      if (!fd_->Step(kFlightSubstepDtSec)) {
        break;
      }
    }
    // VehicleState（弧度 LLA / 真速 / psi 航向）→ 度制状态。
    const auto& state = fd_->state();
    position_.latitude_deg = state.latitude_rad * kRadToDeg;
    position_.longitude_deg = state.longitude_rad * kRadToDeg;
    position_.altitude_m = state.altitude_geod_m;
    heading_deg_ = state.psi_rad * kRadToDeg;
    speed_mps_ = state.vtrue_mps;
    // 航点簿记以库完成事件为准（GetWaypointEvents：中间航点法平面穿越 /
    // 到达圈、最终航点转弯捕获圈——组件自研几何判定与此双层语义必然错位，
    // 不参与 FD 模式）：每条完成事件推进 next_index_ 并发组件事件。Reset
    // 重建（循环巡逻）会清空事件记录 → 游标回落归零。
    const auto& events = fd_->waypoint_events();
    if (events.size() < waypoint_events_consumed_) {
      waypoint_events_consumed_ = 0U;
    }
    while (waypoint_events_consumed_ < events.size()) {
      const auto& event = events[waypoint_events_consumed_];
      ++waypoint_events_consumed_;
      EmitWaypointReached(world, next_index_, event.distance_m);
      ++next_index_;
    }
    // 终端状态（队列完成/中止）：循环巡逻 → 以当前载机状态 Reset 重建续飞
    // （库契约：kCompleted/kAborted 必须 Reset 恢复 kReady；见
    // FlightDynamics::RestartPatrol）；单次飞行 → 剩余航点按几何簿记补发
    // 完成事件（终端视为航路完成语义一致，kAborted 兜底；kCompleted 时
    // 事件驱动已对齐，不触发）。
    if (fd_->terminated()) {
      if (loop_route_ && !route_.empty()) {
        fd_->RestartPatrol(route_);
        next_index_ = 0U;
        waypoint_events_consumed_ = 0U;
        CA_LOG_EVENT(world, "patrol_loop_restart", "巡逻循环重启：FD 以当前载机状态重建续飞");
      } else if (next_index_ < route_.size()) {
        const double t_sec = world.scene_state().t_sec;
        while (next_index_ < route_.size()) {
          const std::size_t reached_index = next_index_;
          ++next_index_;
          EmitWaypointReached(world, reached_index, 0.0);  // 终端补发：无几何到达距离
        }
      }
    }
  } else {
    AdvanceKinematicsFallback(dt_sec);
    CheckWaypointArrival(world, world.scene_state().t_sec);
  }
#else
  AdvanceKinematicsFallback(dt_sec);
  CheckWaypointArrival(world, world.scene_state().t_sec);
#endif

  // 发布平台状态事件（传感器/日志订阅）。
  PlatformStateEvent event;
  event.cycle = world.scene_state().cycle;
  event.t_sec = world.scene_state().t_sec;
  oneq::coordinate::EcefPositionM ecef;
  if (oneq::coordinate::TryLlaToEcef(position_, &ecef)) {
    event.position_ecef_m = ecef;
  }
  event.altitude_m = position_.altitude_m;
  event.heading_deg = heading_deg_;
  event.speed_mps = speed_mps_;
  event.waypoint_index = next_index_;
  event.waypoint_count = route_.size();
  // 平台状态事件每周期重复：事件模式一下不落盘（信号照常发布）。
  CA_LOG_EVENT_DUP(world, "platform_state", "位置=({:.4f},{:.4f},{:.1f})m 航向={:.1f}° 速度={:.1f}m/s 航点={}/{}",
                   event.position_ecef_m.x_m, event.position_ecef_m.y_m, event.altitude_m,
                   event.heading_deg, event.speed_mps, event.waypoint_index, event.waypoint_count);
  world.signals().on_platform_state(event);
}

void FlightComponent::AdvanceKinematicsFallback(double dt_sec) {
  // 运动学回退路径：模拟起飞爬升（固定爬升率到巡航高度）→ 巡航平飞。
  if (position_.altitude_m < cruise_altitude_m_) {
    constexpr double kClimbRateMps = 5.0;  // c172x 量级爬升率近似
    position_.altitude_m =
        std::min(cruise_altitude_m_, position_.altitude_m + kClimbRateMps * dt_sec);
  }
  // 航点寻的：有未完成航点时航向指向下一航点（段间瞬时转向——运动学简化；
  // 显式航路都在直线上时保持原航向，行为与历史"巡航直线"一致；无航路或
  // 航路耗尽维持原航向直飞）。
  if (next_index_ < route_.size()) {
    const navigation::RoutePoint& target = route_[next_index_];
    const double dlat = target.position.latitude_deg - position_.latitude_deg;
    const double dlon = (target.position.longitude_deg - position_.longitude_deg) *
                        std::cos(position_.latitude_deg * kDegToRad);
    heading_deg_ = std::atan2(dlon, dlat) * kRadToDeg;
    if (heading_deg_ < 0.0) {
      heading_deg_ += 360.0;
    }
  }
  AdvanceKinematics(&position_, heading_deg_, speed_mps_, dt_sec);
}

void FlightComponent::CheckWaypointArrival(World& world, double t_sec) {
  while (next_index_ < route_.size() &&
         GreatCircleDistanceM(position_, route_[next_index_]) <=
             std::max(route_[next_index_].radius_m, kMinArrivalRadiusM)) {
    const double distance_m = GreatCircleDistanceM(position_, route_[next_index_]);
    const std::size_t reached_index = next_index_;
    ++next_index_;
    EmitWaypointReached(world, reached_index, distance_m);
  }
  // 循环巡逻：航路耗尽后回绕到第一个航点（平台已在上一条扫描线末端，过渡
  // 段自然飞回起点；空航路由 !route_.empty() 守卫，loop 且空航路维持直飞）。
  if (loop_route_ && !route_.empty() && next_index_ >= route_.size()) {
    next_index_ = 0U;
    CA_LOG_EVENT(world, "patrol_loop_restart", "巡逻循环重启：航路完成，回到航点 0");
  }
}

void FlightComponent::EmitWaypointReached(World& world, std::size_t reached_index,
                                          double distance_m) {
  WaypointReachedEvent event;
  event.t_sec = world.scene_state().t_sec;
  event.waypoint_index = reached_index;
  event.distance_m = distance_m;
  CA_LOG_EVENT(world, "waypoint_reached", "航点={} 到达距离={:.1f}m", event.waypoint_index,
               event.distance_m);
  world.signals().on_waypoint_reached(event);
}

}  // namespace component_attachment
