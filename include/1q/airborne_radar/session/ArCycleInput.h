/**
 * @file ArCycleInput.h
 * @brief 机载雷达单周期输入类型。
 *
 * 周期输入（时间、平台、目标、自然环境和外部 RF 干扰）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/airborne_radar/session/ArExternalInputAdapter.h"
#include "1q/api.hpp"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace session {

using ArPlatformInput = ArExternalPoseInput;
using ArTargetInput = ArExternalTargetInput;
using ArTargetInputList = std::vector<ArTargetInput>;

/** @brief AR 单周期用户输入；不暴露 RF prepare/complete 状态机。 */
struct ONEQ_API ArCycleInput {
  std::uint32_t cycle_index{0U};                 /**< 当前世界周期号；必须非零。 */
  double cycle_start_time_s{0.0};                /**< 当前周期绝对起点（s）。 */
  double dt_sec{1.0};                            /**< 当前周期时长（s）。 */
  ArPlatformInput platform{};                    /**< 雷达平台世界运动学和安装姿态。 */
  ArTargetInputList targets{};                   /**< 世界坐标目标事实。 */
  ArEnvironmentInput environment{};              /**< 大气、空间天气和地表事实。 */
  oneq::electromagnetics::RfEmissionFrame interference{}; /**< ECM/外部 RF 发射事实。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
