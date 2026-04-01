/**
 * @file RadarCycleInput.h
 * @brief 定义按处理周期向雷达链路注入的标准输入载荷。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_

#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/airborne_radar/config/RadarOrientationConfig.h"

namespace airborne_radar {
namespace core {
namespace context {

/**
 * @brief RadarCycleInput 描述单周期输入的目标、姿态与步长。
 */
struct RadarCycleInput {
  common::model::TargetFeatureList target_features{}; /**< 当前周期的目标特征列表 */
  common::config::PlatformAttitudeDeg
      platform_attitude_deg{}; /**< 当前周期的搭载平台姿态角（单位：度） */
  float dt_sec{1.0f};          /**< 当前周期步长（单位：秒） */
};

}  // namespace context
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_RADAR_CYCLE_INPUT_H_
