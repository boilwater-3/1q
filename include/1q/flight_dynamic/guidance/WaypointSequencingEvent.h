/**
 * @file WaypointSequencingEvent.h
 * @brief 定义 kFlyToWaypoint 完成事件记录与完成门枚举。
 * @note 该类型是 flight_dynamic 的决策诊断面：完成判定命中时保留"哪个门、
 * 距离/侧距/沿航迹、有效阈值"等决策输入快照，供查询与调试（配合每步
 * PROJECT_LOG_DEBUG 决策轨迹与完成时 PROJECT_LOG_INFO 一行）。
 */
#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_SEQUENCING_EVENT_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_SEQUENCING_EVENT_H_

#include <cstddef>

namespace oneq {
namespace flight_dynamic {
namespace guidance {

/**
 * @brief kFlyToWaypoint 完成判定的命中门。
 */
enum class WaypointCompletionGate {
  kWithinRadius,     /**< 距离 < 有效到达半径（中间航点 = max(radius_m, 100m)；
                          最终航点 = 转弯量级捕获圈 max(radius_m, 1.5×转弯半径)） */
  kPlaneCrossing,    /**< 法平面穿越（cross-track corridor 内） */
  kFlyPastHeuristic, /**< 兜底：航点已明显偏后（机头角 >115°、距离 > 3×阈值、>10 s） */
};

/**
 * @brief 单个 kFlyToWaypoint 完成事件的决策快照。
 * @note 由 ManeuverExecutor 在完成判定命中时填充门/几何/阈值，FlightManager
 * 补齐仿真时间与队列索引后追加到事件环形记录（GetWaypointEvents 查询）。
 */
struct WaypointSequencingEvent {
  double sim_time_sec = 0.0;                  /**< 完成时刻绝对仿真时间（s） */
  std::size_t waypoint_index = 0;             /**< 机动队列中的航点索引（完成时） */
  bool intermediate = false;                  /**< 中间航点语义（true）/ 最终航点语义（false） */
  WaypointCompletionGate gate = WaypointCompletionGate::kWithinRadius; /**< 命中门 */
  double distance_m = 0.0;                    /**< 完成时刻到航点的水平距离（m） */
  double cross_track_m = 0.0;                 /**< 完成时刻相对航段的侧距（m） */
  double along_track_m = 0.0;                 /**< 完成时刻沿航迹距离（m） */
  double threshold_m = 0.0;                   /**< 完成时刻有效到达半径（m） */
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_SEQUENCING_EVENT_H_
