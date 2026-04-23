/**
 * @file TrackOutputFrame.h
 * @brief 定义供输出管理模块消费和发布的中性轨迹输出帧。
 */

#ifndef AIRBORNE_RADAR_COMMON_TRACK_OUTPUT_FRAME_H_
#define AIRBORNE_RADAR_COMMON_TRACK_OUTPUT_FRAME_H_

#include <cstdint>

#include "1q/airborne_radar/model/TrackStateSnapshot.h"

namespace airborne_radar {
namespace output {

/**
 * @brief TrackOutputFrame 表示单周期稳定的中性轨迹输出帧。
 */
struct TrackOutputFrame {
  std::uint32_t cycle_index{0};              /**< 当前周期号 */
  std::uint64_t batch_id{0};                 /**< 当前批号 */
  model::TrackStateSnapshotList tracks{};         /**< 当前周期发布的轨迹快照列表 */
};

}  // namespace output
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_TRACK_OUTPUT_FRAME_H_
