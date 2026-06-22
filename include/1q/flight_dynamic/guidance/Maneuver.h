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
  ManeuverExecutor(adapter::JsbsimAdapter& adapter, autopilot::Autopilot& ap,
                   WaypointManager& wp_manager, propulsion::EngineManager& engines);

  void ExecuteFlyTo(const Waypoint& target);
  void ExecuteOrbit(const Waypoint& center, double radius_m, double duration_sec = 0.0);
  void ExecuteRacetrack(const Waypoint& start, double heading_rad,
                        double leg_length_m, double turn_radius_m, int num_laps);
  void ExecuteFigure8(const Waypoint& center, double radius_m,
                      double axis_heading_rad, int num_cycles);
  void ExecuteSTurn(double base_heading_rad, double amplitude_deg,
                    double period_sec, double duration_sec);
  void ExecuteSetHeading(double heading_rad, double tolerance_rad = 0.035);
  void ExecuteSetAltitude(double altitude_m, double tolerance_m = 10.0);
  void ExecuteSetPitch(double pitch_deg, double duration_sec);
  void ExecuteSetRoll(int roll_mode);
  void ExecuteTakeoff(double target_altitude_m, double target_heading_rad,
                      double target_speed_mps = 0.0);
  void ExecuteLand(const Waypoint& target, double approach_speed_mps = 0.0);

  bool IsManeuverComplete() const;
  bool IsTouchingGround() const;
  void Update(double dt_sec);
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
