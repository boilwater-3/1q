/**
 * @file TrackOutputFrame.h
 * @brief 定义供输出管理模块消费和发布的中性轨迹输出帧。
 */

#ifndef AIRBORNE_RADAR_COMMON_TRACK_OUTPUT_FRAME_H_
#define AIRBORNE_RADAR_COMMON_TRACK_OUTPUT_FRAME_H_

#include <cstddef>
#include <cstdint>

#include "1q/airborne_radar/common/model/DecisionTrackSnapshot.h"

namespace airborne_radar {
namespace common {
namespace output {

/**
 * @brief TrackOutputFrame 表示单周期稳定的中性轨迹输出帧。
 */
struct TrackOutputFrame {
  std::uint32_t cycle_index{0};              /**< 当前周期号 */
  std::uint64_t batch_id{0};                 /**< 当前批号 */
  model::DecisionTrackSnapshotList tracks{}; /**< 当前周期发布的轨迹快照列表 */
  std::size_t published_track_count{0};      /**< 输出帧内实际发布的轨迹数 */
  std::size_t confirmed_track_count{0};      /**< 输出帧内已确认轨迹数 */
  bool contains_lost_tracks{false};          /**< 输出帧内是否包含 lost 轨迹 */
};

}  // namespace output
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_TRACK_OUTPUT_FRAME_H_
