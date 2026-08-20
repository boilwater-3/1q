/**
 * @file Autopilot.h
 * @brief 定义机型自适应的自动驾驶控制器（Autopilot），封装航向/高度/速度/姿态保持与油门管理。
 *
 * Autopilot 根据 AircraftControlProfile 选择直接舵面、JSBSim 原生自动驾驶或飞控速率指令等
 * 不同控制路径，对外统一暴露 Set / Get / Update 接口。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_
#define ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_

#include <string>

namespace oneq {
namespace flight_dynamic {
namespace adapter {
class JsbsimAdapter;
}
}  // namespace flight_dynamic
}  // namespace oneq

namespace oneq {
namespace flight_dynamic {
namespace autopilot {

/**
 * @brief 横向控制接口类型，决定航向通道的指令注入路径。
 */
enum class LateralControlInterface {
  kDirectSurface,          /**< 直接驱动副翼/方向舵舵面 */
  kGenericAutopilotBridge, /**< 通过 JSBSim 通用自动驾驶属性桥接 */
  kOwnAutopilot,           /**< 使用 C++ 自有自动驾驶解算 */
  kFbwRateCommand,         /**< 飞控（FBW）滚转速率指令 */
};

/**
 * @brief 飞控（FBW）子类型，仅在 kFbwRateCommand 下生效。
 */
enum class FbwSubtype {
  kNone,                    /**< 未使用飞控 */
  kRollRatePid,             /**< 滚转速率 PID */
  kRateIntegratorActuator,  /**< 带积分器的速率-舵机回路 */
};

/**
 * @brief 俯仰控制接口类型，决定俯仰通道的指令注入路径。
 */
enum class PitchControlInterface {
  kDirectSurface,    /**< 直接驱动升降舵 */
  kFbwScheduled,     /**< 按飞行包线调度的飞控 */
  kNativeAutopilot,  /**< JSBSim 原生自动驾驶 */
};

/**
 * @brief 横向引导模式。
 */
enum class LateralGuidanceMode {
  kHeading, /**< 航向保持 */
  kOrbit,   /**< 绕点盘旋 */
};

/**
 * @brief 机型控制能力画像（POD），描述某机型支持的控制接口与物理性能参数。
 *
 * 能量管理相关字段由机型物理量（失速速度、翼载荷）推导，而非硬编码分类；
 * V_stall = sqrt(2W / ρ·S·CLmax) 将当前重量、当前大气密度、机翼面积与最大升力能力
 * 编码为单一基准速度。
 * @note Autopilot 每个 Update() 开始时刷新动态 TAS 字段；调用方应把 GetControlProfile()
 * 返回的引用视为当前周期快照，不应跨周期缓存其中的速度值。
 */
struct AircraftControlProfile {
  LateralControlInterface lateral_interface = LateralControlInterface::kDirectSurface; /**< 横向控制接口 */
  PitchControlInterface pitch_interface = PitchControlInterface::kDirectSurface;       /**< 俯仰控制接口 */
  FbwSubtype fbw_subtype = FbwSubtype::kNone;                                          /**< 飞控子类型 */

  bool has_own_autopilot = false;       /**< 是否具备 C++ 自有自动驾驶 */
  bool has_generic_autopilot = false;   /**< 是否具备 JSBSim 通用自动驾驶 */
  bool has_fbw_override = false;        /**< 是否使用飞控覆盖 */
  bool has_roll_rate_command = false;   /**< 是否支持滚转速率指令 */
  bool has_aileron_command = false;     /**< 是否支持副翼指令 */

  bool indexed_throttle = false;        /**< 是否使用按发动机索引的油门 */
  int engine_count = 0;                 /**< 发动机数量 */
  bool has_mixture = false;             /**< 是否具备混合比控制 */

  std::string yaw_input_property;       /**< 偏航输入对应的 JSBSim 属性名 */

  // --- Energy management profile ---
  // Derived from aircraft physics (V_stall, wing loading), not hardcoded
  // categories.  V_stall = sqrt(2W / ρ·S·CLmax) encodes actual weight,
  // wing area, and lift capability into a single base speed.
  double v_stall_mps = 0.0;           /**< 当前重量/密度下的净构型失速 TAS（单位：m/s） */
  double wing_loading_lbs_ft2 = 0.0;  /**< 翼载荷 = 重量 / 机翼面积（单位：lbs/ft²） */
  double thrust_to_weight = 0.0;      /**< 总静推力 / 重量（无量纲） */
  double ref_speed_mps = 0.0;         /**< 当前有效能量分配参考 TAS（单位：m/s） */
  double min_speed_mps = 0.0;         /**< 当前有效最低 TAS（含失速余度，单位：m/s） */
  double max_speed_mps = 0.0;         /**< 当前有效最高 TAS（结构/推力限制，单位：m/s） */
  double cruise_speed_mps = 0.0;      /**< 当前有效巡航 TAS（单位：m/s） */
  double ceiling_m = 0.0;             /**< 实用升限（单位：m，0 = 由物理量推导） */
  double max_pitch_command_deg = 20.0;  /**< 高度保持中的俯仰指令上限（单位：deg） */
  double max_roll_angle_deg = 45.0;     /**< 结构/气动滚转限制（单位：deg，亦用于盘旋半径） */
  double max_throttle = 1.0;             /**< 油门上限（归一化） */
  double min_throttle = 0.15;            /**< 油门下限（归一化） */
  bool speed_energy_priority = false;  /**< true 表示在能量管理中优先保速而非保高 */

  // --- Rotation / takeoff parameters (scaled by pitch MOI) ---
  double pitch_moi_lbsft2 = 0.0;        /**< 绕机体横轴转动惯量（属性树读取，单位：lbs·ft²） */
  double rotation_ramp_sec = 3.0;       /**< 抬轮段升降舵渐升时长（单位：s） */
  double rotation_max_elevator = 0.30;   /**< 抬轮段升降舵峰值偏转（归一化） */
  double rotation_climb_rate_mps = 5.0;  /**< 抬轮后目标爬升率（单位：m/s） */

  // --- Landing / approach parameters ---
  double landing_approach_speed_mps = 0.0;        /**< 进近速度（单位：m/s，0 = 由指令/Vr 推导） */
  double landing_high_descent_agl_m = 3000.0;     /**< 高下降切换高度 AGL（单位：m） */
  double landing_staging_agl_m = 3000.0;          /**< 高速减速滞空高度 AGL（单位：m） */
  double landing_pattern_agl_m = 200.0;           /**< 最终下降交接高度 AGL（单位：m） */
  bool landing_high_descent_orbit = true;         /**< 高下降段是否盘旋减速 */
  double landing_descent_throttle = -1.0;         /**< 下降油门（<0 = 按发动机类型默认） */
  double landing_approach_flaps_norm = 0.5;       /**< 进近段襟翼位置（归一化 [0,1]） */
  double landing_final_flaps_norm = 1.0;          /**< 最终进近襟翼位置（归一化 [0,1]） */
  double landing_final_throttle_cap = 0.60;  /**< 最终进近油门上限（默认 0.60 为空操作；B747 XML 覆盖为 0.05） */
  double landing_flare_initial_elevator = 0.0;    /**< 拉平起始升降舵（0 = 由机型类别推导） */
  bool landing_heavy_flare = false;               /**< true = 使用运输类弹跳/漂浮拉平律 */
  double landing_touchdown_agl_m = 3.0;           /**< 接地判定高度 AGL（单位：m） */
};

/**
 * @brief 机型自适应自动驾驶控制器。
 *
 * 持有 AircraftControlProfile 与对 JsbsimAdapter 的引用，对外提供航向/高度/俯仰/
 * 滚转/速度/油门/偏航阻尼等通道的 Set / Get 接口，并在每步通过 Update() 驱动能量管理与
 * 各通道保持律。控制路径依据机型画像在直接舵面、JSBSim 原生自动驾驶、C++ 自有自动驾驶、
 * 飞控速率指令之间选择。
 * @note 按 adapter_ 引用持有 JsbsimAdapter，不承担所有权；调用期间须保证 adapter 存活。
 */
class Autopilot {
 public:
  /**
   * @brief 构造自动驾驶控制器并按机型推导控制能力画像。
   * @param[in] adapter JSBSim 适配器引用（调用方保证生命周期）。
   */
  explicit Autopilot(adapter::JsbsimAdapter& adapter);

  // --- Heading ---
  /**
   * @brief 设置目标航向。
   * @param[in] heading_rad 目标航向（单位：rad）。
   */
  void SetHeadingTargetRad(double heading_rad);
  /**
   * @brief 开关航向保持模式。
   * @param[in] on 是否开启。
   */
  void SetHeadingHold(bool on);
  /**
   * @brief 标记航向目标是否来自航点（影响航向源属性切换）。
   * @param[in] from_waypoint true 表示航向由航点引导给出。
   */
  void SetHeadingSourceIsWaypoint(bool from_waypoint);

  // --- Altitude ---
  /**
   * @brief 设置目标高度。
   * @param[in] altitude_m 目标高度（单位：m，ASL）。
   */
  void SetAltitudeTargetM(double altitude_m);
  /**
   * @brief 开关高度保持模式。
   * @param[in] on 是否开启。
   */
  void SetAltitudeHold(bool on);

  // --- Speed / energy management ---
  /**
   * @brief 设置目标真空速。
   * @param[in] speed_mps 目标速度（单位：m/s）。
   */
  void SetSpeedTargetMps(double speed_mps);
  /**
   * @brief 开关速度保持模式。
   * @param[in] on 是否开启。
   */
  void SetSpeedHold(bool on);
  /**
   * @brief 读取当前真空速 TAS。
   * @return 当前真空速（单位：m/s）。
   */
  double GetTrueSpeedMps() const;

  // --- Pitch ---
  /**
   * @brief 设置目标俯仰角。
   * @param[in] pitch_deg 目标俯仰角（单位：deg）。
   */
  void SetPitchTargetDeg(double pitch_deg);
  /**
   * @brief 开关俯仰保持模式。
   * @param[in] on 是否开启。
   */
  void SetPitchHold(bool on);

  // --- Roll ---
  /**
   * @brief 设置横向引导模式（航向保持/绕点盘旋）。
   * @param[in] mode 横向引导模式。
   */
  void SetLateralGuidanceMode(LateralGuidanceMode mode);
  /**
   * @brief 设置滚转姿态模式。
   * @param[in] mode 0=改平（wings level），1=角度保持。
   */
  void SetRollAttitudeMode(int mode);
  /**
   * @brief 开关滚转自动驾驶。
   * @param[in] on 是否开启。
   */
  void SetRollAutopilotOn(bool on);

  // --- Throttle ---
  /**
   * @brief 设置归一化油门指令（必要时同步下发到所有发动机）。
   * @param[in] value 归一化油门值。
   */
  void SetThrottleCmdNorm(double value);
  /**
   * @brief 设置指定发动机的油门指令。
   * @param[in] engine 发动机索引（从 0 起）。
   * @param[in] value 归一化油门值。
   */
  void SetThrottleCmd(int engine, double value);

  // --- Yaw damper ---
  /**
   * @brief 开关偏航阻尼器。
   * @param[in] on 是否开启。
   */
  void SetYawDamper(bool on);

  // --- State management ---
  /**
   * @brief 释放航向/高度/俯仰/速度/滚转等所有保持，并复位为默认横向引导模式。
   */
  void ReleaseHolds();

  // --- Status queries ---
  /**
   * @brief 当前朝向到目标航向的有符号夹角（归一化到 [-π, π]）。
   * @return 夹角（单位：rad）。
   */
  double GetAngleToHeadingRad() const;
  /**
   * @return 当前相对地面高度 AGL（单位：m）。
   */
  double GetAltitudeAGLM() const;
  /**
   * @return 当前大地高 ASL（单位：m）。
   */
  double GetAltitudeASLM() const;
  /** @return 当前控制能力画像；其中 TAS 包线在每个 Update() 开始时刷新。 */
  const AircraftControlProfile& GetControlProfile() const { return control_profile_; }

  /**
   * @brief 推进一个控制步长，先按上一 JSBSim step 的状态刷新 TAS 包线，再更新控制通道。
   * @param[in] dt_sec 步长（单位：s）。
   */
  void Update(double dt_sec);

 private:
  void UpdateOwnAutopilot();
  void UpdateGenericApBridge();
  void UpdateFbwRateCommandLateral();
  void UpdateRollAnglePD();
  void UpdateDirectHeadingLateral();
  void UpdatePitchChannel();
  void ApplyNativeHeadingSetpoint();
  void ApplyYawDamping(double yaw_rate_rad_sec);
  void UpdateEnergyManagement();

  adapter::JsbsimAdapter& adapter_;
  AircraftControlProfile control_profile_;
  bool use_cpp_ap_ = false;
  double roll_int_ = 0.0;

  bool heading_hold_ = false;
  double target_heading_rad_ = 0.0;
  bool heading_src_wp_ = false;
  LateralGuidanceMode lateral_guidance_mode_ = LateralGuidanceMode::kHeading;

  bool altitude_hold_ = false;
  double target_altitude_m_ = 0.0;

  bool pitch_hold_ = false;
  double target_pitch_deg_ = 0.0;

  int roll_mode_ = 0;
  bool roll_ap_on_ = false;

  bool speed_hold_ = false;
  double target_speed_mps_ = 0.0;
};

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_
