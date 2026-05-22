/**
 * @file ManeuverTypes.h
 * @brief 定义机动控制公共类型：机动模式、参数结构体、请求/状态/结果。
 */

#ifndef FLIGHT_DYNAMIC_MANEUVER_MANEUVER_TYPES_H_
#define FLIGHT_DYNAMIC_MANEUVER_MANEUVER_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"

namespace flight_dynamic {
namespace maneuver {

/**
 * @brief 机动模式枚举。
 */
enum class ONEQ_API ManeuverMode {
  kManual = 0,      ///< 手动控制（使用 Step + ControlInput）
  kPointToPoint,    ///< 飞向目标 LLA 点
  kWaypoint,        ///< 航路点序列导航
  kWeave,           ///< 蛇形机动（正弦航向偏置）
  kBarrelRoll,      ///< 滚筒机动
  kOrbit,           ///< 绕圈盘旋
  kEvasion          ///< 规避机动
};

// ---- 机动参数 ----

/** @brief 固定点机动参数。 */
struct ONEQ_API PointToPointParams {
  oneq::coordinate::LlaPositionDegM target_lla{};  ///< 目标点
  double arrival_distance_m{50.0};                   ///< 到达判定距离阈值 (m)
  double heading_tolerance_deg{5.0};                 ///< 到达判定航向误差阈值 (deg)
  double cruise_speed_mps{50.0};                     ///< 巡航速度 (m/s)
  double base_throttle{0.75};                        ///< 基础油门
};

/** @brief 蛇形机动参数。 */
struct ONEQ_API WeaveParams {
  double base_heading_deg{0.0};     ///< 基础航向 (deg)
  double amplitude_deg{30.0};       ///< 摆动幅度 (deg)
  double period_s{20.0};            ///< 摆动周期 (s)
};

/** @brief 航路点列表。 */
using ONEQ_API WaypointList = std::vector<oneq::coordinate::LlaPositionDegM>;

/** @brief 航路点机动参数。 */
struct ONEQ_API WaypointParams {
  PointToPointParams segment_params{};    ///< 单段制导参数（复用固定点逻辑）
  double turn_anticipation_m{200.0};       ///< 转弯提前量 (m)
};

/** @brief 滚筒机动参数。 */
struct ONEQ_API BarrelRollParams {
  double target_roll_deg{360.0};       ///< 总滚转角度
  double roll_rate_degps{40.0};        ///< 目标滚转率 (deg/s)
  double base_altitude_m{1000.0};      ///< 基准高度 (m)
  double cruise_speed_mps{55.0};       ///< 进入速度 (m/s)
  double max_altitude_loss_m{200.0};   ///< 最大允许高度损失 (m)
  double roll_kp{0.03};               ///< 滚转 PID 比例增益
  double roll_ki{0.005};              ///< 滚转 PID 积分增益
  double alt_kp{0.01};                ///< 高度 PID 比例增益
  double alt_ki{0.002};              ///< 高度 PID 积分增益
};

/** @brief 滚筒机动阶段。 */
enum class ONEQ_API BarrelRollPhase {
  kRolling,    ///< 滚转中
  kRecovery,   ///< 恢复平飞
  kCompleted,  ///< 完成
  kAborted     ///< 安全中止
};

/** @brief 滚筒机动状态（由 Session 内部持有，跨帧持久化）。 */
struct ONEQ_API BarrelRollState {
  BarrelRollPhase phase{BarrelRollPhase::kRolling};
  double roll_start_time_s{0.0};       ///< 滚转开始时刻
  double initial_altitude_m{0.0};      ///< 滚转开始高度
  double cumulative_roll_deg{0.0};     ///< 累积滚转角
  double roll_integral{0.0};           ///< 滚转 PID 积分项
  double alt_integral{0.0};            ///< 高度 PID 积分项
  bool initialized{false};             ///< 首次调用初始化标志
};

/** @brief 绕圈盘旋参数。 */
struct ONEQ_API OrbitParams {
  oneq::coordinate::LlaPositionDegM center_lla{};  ///< 盘旋中心点
  double radius_m{1000.0};                          ///< 盘旋半径 (m)
  bool clockwise{true};                             ///< 顺时针（true）/ 逆时针
  double altitude_m{1000.0};                        ///< 目标高度 (m, MSL)
  double cruise_speed_mps{50.0};                    ///< 巡航速度 (m/s)
};

/** @brief 绕圈盘旋状态（由 Session 内部持有）。 */
struct ONEQ_API OrbitState {
  bool initialized{false};             ///< 首次进入轨道标志
};

/** @brief 规避机动阶段。 */
enum class ONEQ_API EvasionPhase {
  kBreaking,    ///< 破转——急转弯到规避航向
  kDescending,  ///< 下降规避
  kCompleted    ///< 规避完成（持续时间到期）
};

/** @brief 规避机动参数。 */
struct ONEQ_API EvasionParams {
  double evasion_heading_deg{0.0};     ///< 规避目标航向 (deg)
  double target_altitude_m{500.0};     ///< 规避目标高度 (m, MSL)
  double duration_s{10.0};            ///< 规避持续时间 (s)
  double cruise_speed_mps{60.0};      ///< 规避速度 (m/s)，通常加速
};

/** @brief 规避机动状态（由 Session 内部持有）。 */
struct ONEQ_API EvasionState {
  EvasionPhase phase{EvasionPhase::kBreaking};
  double start_time_s{0.0};           ///< 规避开始时刻
  bool initialized{false};
};

// ---- 请求 / 状态 / 结果 ----

/**
 * @brief 机动请求——描述要执行的机动模式及其参数。
 *
 * 根据 mode 值，对应参数字段生效；其余字段被忽略。
 */
struct ONEQ_API ManeuverRequest {
  ManeuverMode mode{ManeuverMode::kManual};  ///< 机动模式
  float dt_sec{0.05f};                       ///< 时间步长 (s)

  PointToPointParams point_to_point{};  ///< kPointToPoint 参数
  WaypointList waypoints{};             ///< kWaypoint 航路点列表
  WaypointParams waypoint_params{};     ///< kWaypoint 参数
  WeaveParams weave{};                  ///< kWeave 参数
  BarrelRollParams barrel_roll{};       ///< kBarrelRoll 参数
  OrbitParams orbit{};                  ///< kOrbit 参数
  EvasionParams evasion{};              ///< kEvasion 参数
};

/**
 * @brief 机动状态——描述当前机动执行的进度。
 */
struct ONEQ_API ManeuverStatus {
  ManeuverMode active_mode{ManeuverMode::kManual};  ///< 当前活动模式
  bool active{false};               ///< 机动进行中
  bool completed{false};            ///< 正常完成（到达目标 / 滚转结束）
  bool aborted{false};              ///< 安全中止
  std::size_t waypoint_index{0};    ///< kWaypoint: 当前活动航路点索引
  std::size_t waypoint_count{0};    ///< kWaypoint: 航路点总数
  BarrelRollPhase barrel_roll_phase{BarrelRollPhase::kRolling};  ///< kBarrelRoll 阶段
  double orbit_distance_m{0.0};       ///< kOrbit: 当前到盘旋中心的距离 (m)
  EvasionPhase evasion_phase{EvasionPhase::kBreaking};  ///< kEvasion 阶段
};

/**
 * @brief StepManeuver 的单帧输出——物理状态 + 机动状态。
 */
struct ONEQ_API ManeuverStepResult {
  model::FlightDynamicOutput output{};  ///< 物理积分输出
  ManeuverStatus status{};              ///< 机动执行状态
};

}  // namespace maneuver
}  // namespace flight_dynamic

#endif  // FLIGHT_DYNAMIC_MANEUVER_MANEUVER_TYPES_H_
