/**
 * @file WaypointManager.h
 * @brief 定义航点序列管理器，维护航点队列并在仿真中切换/查询当前目标航点。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINTMANAGER_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINTMANAGER_H_

#include <vector>

#include "1q/flight_dynamic/guidance/Waypoint.h"

namespace oneq {
namespace flight_dynamic {

namespace adapter {
class JsbsimAdapter;
}

namespace guidance {

/**
 * @brief 单次解析的激活航点邻近快照。
 *
 * 一次位置读取 + 一次航迹几何解析同时给出距离、航向、侧距/沿航迹与法平面
 * 穿越判定，供 IsAtTarget/IsAtOrPastTarget 与完成事件记录共用，避免每步
 * 对同一帧位置重复计算。
 */
struct WaypointProximity {
  bool valid = false;          /**< 航段几何解析成功（退化航段为 false，保守判定不越过） */
  double distance_m = 0.0;     /**< 到激活航点的水平距离（m）；未启动或无航点时 0 */
  double heading_to_rad = 0.0; /**< 指向激活航点的航向（rad）；未启动或无航点时 0 */
  double cross_track_m = 0.0;  /**< 相对航段（leg start→航点大圆）的侧距（m） */
  double along_track_m = 0.0;  /**< 沿航迹距离（m） */
  double leg_length_m = 0.0;   /**< 航段长度（m） */
  bool plane_crossed = false;  /**< 已在 corridor（max(3000, 3×radius)）内越过法平面 */
};

/**
 * @brief 航点序列管理器。
 *
 * 维护一个航点队列，按顺序激活航点并把目标航点信息下发到 JSBSim 属性树。
 * 切换逻辑由调用方在每步根据 IsAtTarget/IsAtOrPastTarget/AdvanceToNext 驱动。
 * @note 该类按引用持有 adapter_，不承担所有权；调用期间须保证 adapter 存活。
 */
class WaypointManager {
 public:
  /**
   * @brief 构造航点管理器。
   * @param[in] adapter JSBSim 适配器引用（调用方保证生命周期）。
   */
  explicit WaypointManager(adapter::JsbsimAdapter& adapter);

  /**
   * @brief 追加一个航点到队列末尾。
   * @param[in] wp 待追加的航点。
   */
  void AddWaypoint(const Waypoint& wp);
  /**
   * @brief 清空航点队列并复位激活索引与 started 状态。
   */
  void ClearWaypoints();
  /**
   * @brief 直接将激活索引设为指定值（越界则忽略），并据此刷新航段起点与下发属性。
   * @param[in] index 目标激活航点下标。
   */
  void SetActiveWaypoint(size_t index);
  /**
   * @brief 从第 0 个航点开始激活；以当前位置作为首段航段起点。队列为空时为空操作。
   */
  void Start();

  /**
   * @brief 切换到下一个航点；成功切换返回 true，已是最后一个则返回 false。
   * @return 是否成功推进到下一个航点。
   */
  bool AdvanceToNext();
  /**
   * @brief 解析当前激活航点的邻近快照（距离 + 航迹几何 + 法平面穿越判定）。
   * @return 单帧位置的邻近快照；未启动或无航点时返回全默认值。
   */
  WaypointProximity ResolveProximity() const;
  /**
   * @brief 当前到激活航点的水平距离是否在判定半径内。
   * @param[in] threshold_m 判定半径（单位：m）；<=0 时使用航点自身的 radius_m。
   * @return 进入半径范围返回 true，否则返回 false。
   */
  bool IsAtTarget(double threshold_m = -1.0) const;
  /**
   * @brief 已到达或在航向上越过当前目标航点（用于切出判定）。
   * @param[in] threshold_m 到达判定半径（单位：m）；<=0 时使用航点 radius_m。
   * @return 已到达或沿航段越过目标返回 true。
   */
  bool IsAtOrPastTarget(double threshold_m = -1.0) const;
  /**
   * @brief 序列是否已结束（未 Start 或激活索引越界）。
   * @return 已结束返回 true。
   */
  bool IsFinished() const;

  /**
   * @brief 当前载机到激活航点的水平大圆距离。
   * @return 距离（单位：m）；未启动或无航点时返回 0。
   */
  double GetDistanceToActiveM() const;
  /**
   * @brief 从当前载机位置指向激活航点的航向角。
   * @return 航向（单位：rad）；未启动或无航点时返回 0。
   */
  double GetHeadingToActiveRad() const;
  /** @return 当前激活航点下标。 */
  size_t GetActiveIndex() const { return active_index_; }
  /** @return 队列中的航点数量。 */
  size_t GetWaypointCount() const { return waypoints_.size(); }

 private:
  void SetLegStartFromCurrentLocation();
  void ApplyActiveWaypoint();

  adapter::JsbsimAdapter& adapter_;
  std::vector<Waypoint> waypoints_;
  size_t active_index_ = 0;
  bool started_ = false;
  double leg_start_latitude_rad_ = 0.0;
  double leg_start_longitude_rad_ = 0.0;
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINTMANAGER_H_
