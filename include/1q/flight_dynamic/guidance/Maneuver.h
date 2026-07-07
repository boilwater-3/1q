/**
 * @file Maneuver.h
 * @brief 定义机动类型枚举与机动执行状态机（ManeuverExecutor）。
 */
#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_

#include <vector>

#include "1q/flight_dynamic/guidance/Waypoint.h"

namespace oneq {
namespace flight_dynamic {

namespace adapter {
class JsbsimAdapter;
}
namespace autopilot {
class Autopilot;
}
namespace guidance {
class WaypointManager;
}
namespace propulsion {
class EngineManager;
}

namespace guidance {

/**
 * @brief 机动类型枚举，决定 ManeuverCommand 各字段的语义重载。
 * @note 字段与机动的精确映射见 FlightManager.h 中 ManeuverCommand 的说明。
 */
enum class ManeuverType {
  kFlyToWaypoint, /**< 飞向指定航点 */
  kOrbit,         /**< 绕指定圆心盘旋 */
  kRacetrack,     /**< 跑道形往返航线（直线段+半圆转弯） */
  kFigure8,       /**< 8 字航线（两个反向圆弧） */
  kSTurn,         /**< S 型转弯（正弦航向摆动） */
  kSetHeading,    /**< 保持目标航向 */
  kSetAltitude,   /**< 爬升/下降到目标高度 */
  kSetPitch,      /**< 保持目标俯仰角一段时间 */
  kSetRoll,       /**< 设置滚转模式（改平/角度保持） */
  kTakeoff,       /**< 起飞（滑跑→抬轮→爬升） */
  kLand,          /**< 降落（减速→进近→拉平→接地） */
};

/**
 * @brief ManeuverExecutor 内部使用的机动描述。
 *
 * 字段语义与 FlightManager.h 的 ManeuverCommand 一致（随 ManeuverType 重载），
 * 由 FlightManager 在 ExecuteNextManeuver 中由 ManeuverCommand 转换而来。
 */
struct Maneuver {
  ManeuverType type;          /**< 机动类型 */
  Waypoint target;            /**< 航点目标 */
  double value = 0.0;         /**< 通用数值参数（语义随类型重载） */
  double duration_sec = 0.0;  /**< 通用时长/长度参数（语义随类型重载） */
  double heading_tolerance_rad = 0.035; /**< 航向容差或重载的摆幅/转弯半径 */
  double altitude_tolerance_m = 10.0;   /**< 高度容差或重载的周期/圈数 */
};

/**
 * @brief 机动执行状态机（FSM），驱动单个机动的相位推进。
 *
 * 由 FlightManager 持有，用户通常不直接调用：各类 Execute* 方法启动对应机动，
 * Update(dt) 在每步推进相位，IsManeuverComplete() 报告是否完成。
 */
class ManeuverExecutor {
 public:
  /**
   * @brief 构造机动执行器。
   * @param[in] adapter JSBSim 适配器引用。
   * @param[in] ap 自动驾驶控制器引用。
   * @param[in] wp_manager 航点管理器引用。
   * @param[in] engines 发动机管理器引用。
   * @note 按引用持有上述子组件，不承担所有权；调用期间须保证其存活。
   */
  ManeuverExecutor(adapter::JsbsimAdapter& adapter, autopilot::Autopilot& ap,
                   WaypointManager& wp_manager, propulsion::EngineManager& engines);

  /**
   * @brief 启动“飞向航点”机动。
   * @param[in] target 目标航点。
   */
  void ExecuteFlyTo(const Waypoint& target);
  /**
   * @brief 启动绕指定圆心盘旋机动。
   * @param[in] center 圆心航点。
   * @param[in] radius_m 盘旋半径（单位：m）。
   * @param[in] duration_sec 持续时间（单位：s，0 表示绕行一圈即完成）。
   */
  void ExecuteOrbit(const Waypoint& center, double radius_m, double duration_sec = 0.0);
  /**
   * @brief 启动跑道形往返航线机动（直线段 + 半圆转弯）。
   * @param[in] start 起点航点。
   * @param[in] heading_rad 直线段航向（单位：rad）。
   * @param[in] leg_length_m 直线段长度（单位：m）。
   * @param[in] turn_radius_m 转弯半径（单位：m）。
   * @param[in] num_laps 跑道圈数。
   */
  void ExecuteRacetrack(const Waypoint& start, double heading_rad,
                        double leg_length_m, double turn_radius_m, int num_laps);
  /**
   * @brief 启动 8 字航线机动（两个反向圆弧）。
   * @param[in] center 圆心航点（其中一个圆弧的中心）。
   * @param[in] radius_m 圆弧半径（单位：m）。
   * @param[in] axis_heading_rad 轴向航向（单位：rad）。
   * @param[in] num_cycles 周期数。
   */
  void ExecuteFigure8(const Waypoint& center, double radius_m,
                      double axis_heading_rad, int num_cycles);
  /**
   * @brief 启动 S 型转弯机动（正弦航向摆动）。
   * @param[in] base_heading_rad 基准航向（单位：rad）。
   * @param[in] amplitude_deg 航向摆幅（单位：deg）。
   * @param[in] period_sec 摆动周期（单位：s）。
   * @param[in] duration_sec 总持续时间（单位：s）。
   */
  void ExecuteSTurn(double base_heading_rad, double amplitude_deg,
                    double period_sec, double duration_sec);
  /**
   * @brief 启动航向保持机动。
   * @param[in] heading_rad 目标航向（单位：rad）。
   * @param[in] tolerance_rad 收敛容差（单位：rad，默认 0.035≈2°）。
   */
  void ExecuteSetHeading(double heading_rad, double tolerance_rad = 0.035);
  /**
   * @brief 启动爬升/下降到目标高度机动。
   * @param[in] altitude_m 目标高度（单位：m）。
   * @param[in] tolerance_m 收敛容差（单位：m，默认 10）。
   */
  void ExecuteSetAltitude(double altitude_m, double tolerance_m = 10.0);
  /**
   * @brief 启动保持目标俯仰角一段时间机动。
   * @param[in] pitch_deg 目标俯仰角（单位：deg）。
   * @param[in] duration_sec 持续时间（单位：s）。
   */
  void ExecuteSetPitch(double pitch_deg, double duration_sec);
  /**
   * @brief 启动设置滚转模式机动。
   * @param[in] roll_mode 滚转模式（0=改平，1=角度保持）。
   */
  void ExecuteSetRoll(int roll_mode);
  /**
   * @brief 启动起飞机动（滑跑 → 抬轮 → 爬升）。
   * @param[in] target_altitude_m 目标爬升高度（单位：m）。
   * @param[in] target_heading_rad 目标航向（单位：rad）。
   * @param[in] target_speed_mps 目标速度（单位：m/s，0=机型默认）。
   */
  void ExecuteTakeoff(double target_altitude_m, double target_heading_rad,
                      double target_speed_mps = 0.0);
  /**
   * @brief 启动降落机动（减速 → 进近 → 拉平 → 接地）。
   * @param[in] target 目标跑道航点。
   * @param[in] approach_speed_mps 进近速度（单位：m/s，0=机型默认）。
   */
  void ExecuteLand(const Waypoint& target, double approach_speed_mps = 0.0);

  /**
   * @brief 当前机动是否已完成（含收敛、坠毁、中止等判定）。
   * @return 完成返回 true。
   */
  bool IsManeuverComplete() const;
  /**
   * @brief 载机是否已接触地面。
   * @return 接地返回 true。
   */
  bool IsTouchingGround() const;
  /**
   * @brief 推进一个仿真步长，执行当前机动的相位状态机。
   * @param[in] dt_sec 步长（单位：s）。
   */
  void Update(double dt_sec);
  /**
   * @brief 中止当前机动，置为非激活。
   */
  void Abort();

 private:
  enum class TakeoffPhase {
    kEngineStart,
    kStaticRunup,
    kTakeoffRoll,
    kRotateAndClimb,
    kComplete,
  };

  enum class LandPhase {
    kDecelerate,
    kApproach,
    kFinalDescent,
    kFlare,
    kTouchdown,
    kRollout,
    kComplete,
  };

  void StartEngine();
  void ConfigureForTakeoffRoll();
  void ConfigureForClimb(double target_altitude_m, double target_heading_rad,
                         double target_speed_mps);
  void ConfigureForApproach(const Waypoint& target, double approach_speed_mps);
  void ConfigureForLanding();

  adapter::JsbsimAdapter& adapter_;
  autopilot::Autopilot& ap_;
  WaypointManager& wp_manager_;
  propulsion::EngineManager& engines_;
  Maneuver current_maneuver_;
  bool active_ = false;
  double elapsed_sec_ = 0.0;
  TakeoffPhase takeoff_phase_ = TakeoffPhase::kEngineStart;
  double takeoff_target_altitude_m_ = 0.0;
  double takeoff_target_heading_rad_ = 0.0;
  double takeoff_phase_elapsed_sec_ = 0.0;
  double rotation_elapsed_sec_ = 0.0;
  double rotation_ramp_origin_ = 0.0;  // elevator level from pre-rotation phase
  double takeoff_vr_kts_ = 0.0;       // cached Vr for airspeed checks during climb
  LandPhase land_phase_ = LandPhase::kApproach;
  double land_approach_speed_mps_ = 0.0;
  double land_target_alt_m_ = 0.0;
  double prev_alt_m_ = 0.0;
  double sink_rate_mps_ = 0.0;
  double flare_elapsed_sec_ = 0.0;

  // ── Racetrack FSM ──
  enum class RacetrackPhase { kApproach, kLeg1, kTurn1, kLeg2, kTurn2, kComplete };
  RacetrackPhase racetrack_phase_ = RacetrackPhase::kComplete;
  double racetrack_heading_ = 0.0;
  double racetrack_leg_len_ = 0.0;
  double racetrack_turn_r_ = 0.0;
  int racetrack_target_laps_ = 1;
  int racetrack_lap_ = 0;
  Waypoint racetrack_entry_;  // position at current phase start
  Waypoint racetrack_center1_;
  Waypoint racetrack_center2_;
  Waypoint racetrack_leg1_entry_; // Absolute geographic start of Leg 1
  Waypoint racetrack_leg2_entry_; // Absolute geographic start of Leg 2

  // ── Racetrack speed scheduling ──
  double racetrack_cruise_spd_ = 0.0;  // target speed on straight legs
  double racetrack_turn_spd_   = 0.0;  // target speed in turns

  // ── Figure-8 FSM ──
  enum class Figure8Phase { kCw, kCcw, kComplete };
  Figure8Phase figure8_phase_ = Figure8Phase::kComplete;
  int figure8_cycle_ = 0;
  int figure8_target_cycles_ = 1;
  double figure8_bearing_accum_ = 0.0;
  double figure8_prev_bearing_ = 0.0;
  Waypoint figure8_center_;   // CW lobe center
  Waypoint figure8_center2_;  // CCW lobe center
  double figure8_radius_ = 0.0;

  // ── S-Turn state ──
  double sturn_base_heading_ = 0.0;
  double sturn_amplitude_rad_ = 0.0;
  double sturn_freq_ = 0.0;  // 2π / period_sec
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_
