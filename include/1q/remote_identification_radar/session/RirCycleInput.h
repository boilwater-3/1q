/**
 * @file RirCycleInput.h
 * @brief 远程识别雷达单周期输入类型。
 *
 * 周期输入（时间戳、平台状态、场景目标、航迹供给）的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "1q/remote_identification_radar/session/RirTrackFeedTypes.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirCycleInput 描述单周期输入。
 *
 * @note 帧约定：`platform_altitude_m` 为平台绝对海拔（识别高度观测 =
 *       平台海拔 + `RirTrackFeedEntry::position_z`，ENU 局部切平面上向分量）；
 *       `sim_time_sec` 为调用方提供的仿真时间（等价性对比测试与 AR 侧
 *       sim_time 对齐）。
 */
struct ONEQ_API RirCycleInput {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号（≠0 合法）。 */
  std::uint64_t batch_id{0U};          /**< 输入批号。 */
  double dt_sec{0.0};                  /**< 周期步长（s），有限且为正。 */
  float sim_time_sec{0.0f};            /**< 当前仿真时间（s）。 */
  float platform_altitude_m{0.0f};     /**< 平台绝对海拔（m）。 */

  RirSceneTargetList scene_targets{}; /**< 场景目标（含识别特征真值）。 */
  RirTrackFeed track_feed{};          /**< 外部雷达航迹供给（已确认航迹）。 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
