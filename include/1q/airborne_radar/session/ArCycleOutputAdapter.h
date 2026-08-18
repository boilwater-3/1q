/**
 * @file ArCycleOutputAdapter.h
 * @brief 机载雷达周期输出适配器。
 *
 * 周期输出适配（内部轨迹帧转 ECEF 输出帧）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_OUTPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_OUTPUT_ADAPTER_H_

#include "1q/airborne_radar/session/ArExternalInputAdapter.h"
#include "1q/airborne_radar/session/ArExternalOutputAdapter.h"
#include "1q/airborne_radar/session/ArTrackOutput.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief 外部可消费的单周期轨迹输出帧。
 * @note `TrackOutputFrame` 保留雷达局部/相对雷达语义；本结构将轨迹位置和速度转换回 ECEF。
 */
struct ONEQ_API ArExternalTrackOutputFrame {
  std::uint32_t cycle_index{0};              /**< 当前周期号 */
  std::uint64_t batch_id{0};                 /**< 当前批号 */
  ArExternalTrackKinematicsList tracks{}; /**< 当前周期发布的外部轨迹运动学 */
};

/**
 * @brief 输出侧适配器：把内部 TrackOutputFrame 转换为外部 ECEF 输出帧。
 */
struct ONEQ_API ArCycleOutputAdapter {
  /**
   * @brief 从外部平台运动学和内部输出帧一步构建外部 ECEF 输出帧。
   * @param[in] platform 当前周期外部平台运动学输入，应与生成该输出帧的输入周期一致。
   * @param[in] frame 内部雷达局部轨迹输出帧。
   * @param[out] output 外部 ECEF 轨迹输出帧；可为 nullptr。
   * @return 转换成功返回 true。
   */
  static bool Build(const ArExternalPoseInput& platform, const TrackOutputFrame& frame,
                    ArExternalTrackOutputFrame* output);

  /**
   * @brief 从已解析的雷达局部参考系和内部输出帧构建外部 ECEF 输出帧。
   * @param[in] reference 雷达局部参考系信息。
   * @param[in] radar_local_velocity_mps 雷达平台在雷达局部坐标系下的速度。
   * @param[in] frame 内部雷达局部轨迹输出帧。
   * @param[out] output 外部 ECEF 轨迹输出帧；可为 nullptr。
   * @return 转换成功返回 true。
   */
  static bool Build(const oneq::coordinate::LocalFrameReference& reference,
                    oneq::foundation::Vector3f radar_local_velocity_mps,
                    const TrackOutputFrame& frame, ArExternalTrackOutputFrame* output);

 private:
  ArCycleOutputAdapter() = delete;
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_OUTPUT_ADAPTER_H_
