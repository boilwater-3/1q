/**
 * @file ArCycleInput.h
 * @brief 机载雷达单周期输入类型。
 *
 * 周期输入（时间、平台、目标、外部 RF 干扰）的主头文件。
 * 场景目标为平台锚点 ENU：集成侧用 `TryMakeEnuSceneState` 直填 `targets`。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArPlatformInput.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace session {

/** @brief AR 单周期用户输入；不暴露 RF prepare/complete 状态机。 */
struct ONEQ_API ArCycleInput {
  std::uint32_t cycle_index{0U};                 /**< 当前世界周期号；必须非零。 */
  double cycle_start_time_s{0.0};                /**< 当前周期绝对起点（s）；须随周期单调推进（≥上一窗口结束）。 */
  double dt_sec{1.0};                            /**< 当前周期时长（s）。 */
  ArPlatformInput platform{};                    /**< 雷达平台世界运动学。 */
  ArTargetInputList targets{};                   /**< 平台锚点 radar-local ENU 场景目标（见 ArTargetInput）。 */
  oneq::electromagnetics::RfEmissionFrame interference{}; /**< ECM/外部 RF 发射事实。非空时其 envelope（world_cycle_index/window_start_time_s/window_duration_s）须与本周期权威时间一致；空帧不校验 envelope。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_INPUT_H_
