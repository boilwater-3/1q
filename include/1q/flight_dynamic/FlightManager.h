/**
 * @file FlightManager.h
 * @brief flight_dynamic 模块的顶层入口，封装 JSBSim 动力学仿真与机动队列调度。
 * @note 用户通过 FlightManager 构造会话、PushManeuver 下发机动、Step 步进并读取 VehicleState。
 */
#ifndef ONEQ_FLIGHT_DYNAMIC_FLIGHTMANAGER_H_
#define ONEQ_FLIGHT_DYNAMIC_FLIGHTMANAGER_H_

#include <memory>
#include <vector>

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/guidance/Maneuver.h"
#include "1q/flight_dynamic/guidance/Waypoint.h"
#include "1q/flight_dynamic/model/VehicleState.h"

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

/**
 * @brief 单次机动的结果分类，由 FlightManager 在步进中根据收敛、坠毁、中止等条件置位。
 */
enum class ManeuverOutcome {
  kNone,      /**< 机动仍在执行中，尚未结束 */
  kCompleted, /**< 成功到达目标或满足完成条件 */
  kNearPass,  /**< 近距通过：接近目标但未完全到达 */
  kCrashed,   /**< 坠毁：高度降至地面以下 */
  kTimeout,   /**< 超过最大步数仍未完成 */
  kAborted,   /**< 被外部调用 Abort() 中止 */
};

/**
 * @brief FlightManager 会话状态机，反映当前机动队列的执行阶段。
 */
enum class FlightManagerState {
  kIdle,      /**< 默认初始值，构造完成后即转为 kReady */
  kReady,     /**< 已就绪：完成配平与子组件装配，等待下发机动 */
  kExecuting, /**< 正在执行机动队列中的某个机动 */
  kCompleted, /**< 队列中所有机动均已执行完毕 */
  kAborted,   /**< 因坠毁或外部 Abort() 而中止 */
};

/**
 * @brief 用户下发的机动指令，通过 FlightManager::PushManeuver 入队。
 *
 * @warning 该结构体字段语义随 type 不同而**高度重载**：同一个 value /
 * duration_sec / heading_tolerance_rad / altitude_tolerance_m 在不同机动类型下
 * 表示完全不同的物理量，某些机动还会复用 target 的子字段。字段含义的权威定义
 * 见 src/flight_dynamic/FlightManager.cpp 中 ExecuteNextManeuver 的 dispatch。
 *
 * 各机动类型的字段约定：
 * - kFlyToWaypoint：target=目标航点(lat/lon/alt/radius_m)；
 *   heading_tolerance_rad=航向收敛容差(rad，默认 0.035≈2°)；
 *   altitude_tolerance_m=高度收敛容差(m，默认 10)。value/duration_sec 未使用。
 *   完成语义区分中间/最终航点：队列中后继仍是 kFlyToWaypoint 的中间航点按导航语义
 *   完成（法平面穿越 / 到达半径 max(radius_m, 100m)）；最终航点（单航点或队列末尾）
 *   按转弯量级捕获圈 max(radius_m, 1.5×转弯半径) 完成（容差按机型/速度实时推导，
 *   不同型号不可一概而论）。详见 docs/flight_dynamic/algorithms.md 航路点到达语义。
 * - kOrbit：target=圆心航点；value=盘旋半径(m)；
 *   duration_sec=持续时间(s，0 表示绕行一圈即完成)。其余字段未使用。
 * - kSetHeading：value=目标航向(rad)；heading_tolerance_rad=收敛容差(rad)。
 * - kSetAltitude：value=目标高度(m)；altitude_tolerance_m=收敛容差(m)。
 * - kSetPitch：value=目标俯仰角(°)；duration_sec=持续时间(s)。
 * - kSetRoll：value=滚转模式(int，0=改平 / 1=角度保持)。
 * - kSTurn：value=基准航向(rad)；duration_sec=总持续时间(s)；
 *   heading_tolerance_rad=**航向摆幅(°，注意非 rad)**；
 *   altitude_tolerance_m=**摆动周期(s，注意非 m)**。
 * - kRacetrack：target=起点航点；value=航向(rad)；duration_sec=直航段长度(m)；
 *   heading_tolerance_rad=转弯半径(m)；altitude_tolerance_m=圈数(int)。
 * - kFigure8：target=圆心航点；value=半径(m)；duration_sec=轴向航向(rad)；
 *   heading_tolerance_rad=周期数(int)。
 * - kTakeoff：target.altitude_m=目标爬升高度(m)；
 *   **target.latitude_rad=目标航向(rad)**；value=目标速度(m/s，0=机型默认)。
 * - kLand：target=目标跑道航点；value=进近速度(m/s，0=机型默认)。
 */
struct ManeuverCommand {
  guidance::ManeuverType type;         /**< 机动类型，决定其余字段的语义（见上方重载说明） */
  guidance::Waypoint target;           /**< 航点目标，其具体含义随机动类型而异 */
  double value = 0.0;                  /**< 通用数值参数，含义随机动类型重载（航向/高度/半径/速度等） */
  double duration_sec = 0.0;           /**< 通用时长/长度参数，含义随机动类型重载（持续时间或直航段长度） */
  double heading_tolerance_rad = 0.035; /**< 默认为 SetHeading 航向收敛容差(rad)；在 kSTurn/kRacetrack 下被重载 */
  double altitude_tolerance_m = 10.0;  /**< 默认为 SetAltitude 高度收敛容差(m)；在 kSTurn/kRacetrack 下被重载 */
};

/**
 * @brief 单次机动的诊断统计，在步进过程中由 FlightManager 累积，供用户查询轨迹极值与结果。
 */
struct ManeuverDiagnostics {
  guidance::ManeuverType current_type = guidance::ManeuverType::kFlyToWaypoint; /**< 当前机动类型 */
  ManeuverOutcome outcome = ManeuverOutcome::kNone; /**< 机动结果，完成后置位 */
  double min_altitude_m = 1e9;  /**< 本次机动过程中的最低几何高度(m) */
  double min_speed_mps = 1e9;   /**< 本次机动过程中的最低真空速(m/s) */
  double max_roll_deg = 0.0;    /**< 最大滚转角(°) */
  double max_pitch_deg = 0.0;   /**< 最大俯仰角(°) */
  int steps = 0;                /**< 已执行的仿真步数 */
  double total_time_sec = 0.0;  /**< 累计仿真时长(s) */
  bool crashed = false;         /**< 是否发生坠毁 */
  std::string last_failure_reason; /**< 最近一次失败的原因描述 */

  /** @brief 用最新载机状态更新诊断极值与坠毁标记。 */
  void Update(const model::VehicleState& s);
  /** @brief 向 stdout 打印诊断摘要。 */
  void Print() const;
};

/**
 * @brief flight_dynamic 的顶层会话入口。
 *
 * 持有 JSBSim 适配器、自动驾驶、航点管理与机动执行器等子组件，对外只暴露
 * “配置 → 下发机动 → 步进 → 读状态”的使用范式。典型用法：构造后通过
 * PushManeuver 将一条或多条 ManeuverCommand 入队，循环调用 Step(dt) 推进仿真，
 * 并在每步用 GetVehicleState / GetDiagnostics 读取载机状态与诊断。机动按入队
 * 顺序串行执行，前一个完成（或坠毁）后自动切换到下一个。
 * @note 不可拷贝；重置场景请使用 Reset()。
 */
class FlightManager {
 public:
  /**
   * @brief 装配子组件并完成初始配平，构造完成后状态为 kReady。
   * @param[in] config 会话配置（机型、初始运动学、积分器等）
   */
  explicit FlightManager(const config::FlightDynamicConfig& config);
  ~FlightManager();

  FlightManager(const FlightManager&) = delete;            /**< 不可拷贝 */
  FlightManager& operator=(const FlightManager&) = delete; /**< 不可赋值 */

  /**
   * @brief 推进一个仿真步长。
   * @param[in] dt_sec 步长(s)
   * @return 仿真是否仍在运行；处于 kCompleted/kAborted 时返回 false
   */
  bool Step(double dt_sec);
  /**
   * @brief 丢弃当前会话并以新配置重建子组件，状态回到 kReady。
   * @param[in] config 新的会话配置
   */
  void Reset(const config::FlightDynamicConfig& config);
  /** @brief 中止当前机动，释放自动驾驶保持，状态转为 kAborted。 */
  void Abort();

  // Maneuver sequencing
  /** @brief 将机动指令追加到队尾；若当前空闲则立即开始执行队首机动。 */
  void PushManeuver(const ManeuverCommand& cmd);
  /** @brief 清空机动队列并复位执行索引（不影响当前载机状态）。 */
  void ClearManeuvers();

  // State queries
  /** @return 当前会话状态机阶段。 */
  FlightManagerState GetState() const { return state_; }
  /** @return 最新载机运动学/姿态状态（每步由 JSBSim 映射更新）。 */
  const model::VehicleState& GetVehicleState() const { return vehicle_state_; }
  /** @return 当前机动的诊断统计（const 重载）。 */
  const ManeuverDiagnostics& GetDiagnostics() const { return diagnostics_; }
  /** @return 当前机动的诊断统计（可写重载，便于手动重置极值）。 */
  ManeuverDiagnostics& GetDiagnostics() { return diagnostics_; }

  // Direct access for lower-level control
  adapter::JsbsimAdapter& GetAdapter() { return *adapter_; }          /**< @return JSBSim 适配器，用于低层直接控制 */
  autopilot::Autopilot& GetAutopilot() { return *ap_; }               /**< @return 自动驾驶控制器，用于低层直接控制 */
  propulsion::EngineManager& GetEngineManager() { return *engines_; } /**< @return 发动机管理器 */
  const propulsion::EngineManager& GetEngineManager() const { return *engines_; } /**< @return 发动机管理器（const 重载） */
  guidance::WaypointManager& GetWaypointManager() { return *wp_manager_; } /**< @return 航点管理器，用于低层直接控制 */

 private:
  void ExecuteNextManeuver();

  std::unique_ptr<adapter::JsbsimAdapter> adapter_;
  std::unique_ptr<autopilot::Autopilot> ap_;
  std::unique_ptr<propulsion::EngineManager> engines_;
  std::unique_ptr<guidance::WaypointManager> wp_manager_;
  std::unique_ptr<guidance::ManeuverExecutor> maneuver_exec_;

  model::VehicleState vehicle_state_;
  ManeuverDiagnostics diagnostics_;
  std::vector<ManeuverCommand> maneuver_queue_;
  size_t current_maneuver_index_ = 0;
  FlightManagerState state_ = FlightManagerState::kIdle;
  double sim_time_sec_ = 0.0;
};

}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_FLIGHTMANAGER_H_
