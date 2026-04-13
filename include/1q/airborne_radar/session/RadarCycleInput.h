/**
 * @file RadarCycleInput.h
 * @brief 定义按处理周期向雷达链路注入的标准输入载荷。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_

#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarCycleInput 描述单周期输入的目标、平台位姿与步长。
 */
struct RadarCycleInput {
  model::TargetFeatureList target_features{}; /**< 当前周期的目标特征列表 */
  oneq::foundation::PoseState platform_pose{}; /**< 当前周期平台位姿状态 */
  float dt_sec{1.0f};                           /**< 当前周期步长（单位：秒） */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_
