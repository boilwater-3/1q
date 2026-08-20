/**
 * @file RirCycleInput.h
 * @brief 远程识别雷达单周期输入类型。
 *
 * 周期输入（时间戳、平台 ECEF、场景目标、外部 RF 场景）的主头文件。
 * 阶段 2-S 起为独立输入面：不再消费外部雷达航迹供给（RirTrackFeed 已退役）。
 * 环境事实经 `RirSessionConfig.environment` / 运行期补丁注入，禁止周期携带。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirCycleInput 描述单周期输入。
 *
 * @note 帧约定：`platform_position` 为平台 ECEF（必填，有限且模长 > 0）；库内由
 *       `TryEcefToLla` 派生绝对海拔供运动特征（识别高度 = 平台海拔 + 航迹 ENU z）。
 *       `sim_time_sec` 为调用方提供的仿真时间。外部 RF 干扰经 `rf_scene` 注入
 *       （仅非本机 emission）；库内合并自发射后求解 incident links。
 * @note `platform_position` 同时作为场景 radar-local ENU 的绝对锚点——不改变场景
 *       目标 ENU 语义，为特征量测出口（出口①）提供 sensor_origin 与地理参考。
 * @note 场景目标 `position_x/y/z` 仍为 radar-local ENU；集成层用户侧以 ECEF 描述
 *       目标，适配层在边界完成 ECEF→ENU 转换后再填入 `scene_targets`。
 */
struct ONEQ_API RirCycleInput {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号（≠0 合法）。 */
  double dt_sec{0.0};                  /**< 周期步长（s），有限且为正。 */
  float sim_time_sec{0.0f};            /**< 当前仿真时间（s）。 */
  oneq::coordinate::EcefPositionM platform_position{}; /**< 平台 ECEF 位置（m，必填）。 */

  RirSceneTargetList scene_targets{}; /**< 场景目标（含识别特征真值与运动事实）。 */

  /**
   * @brief 外部 RF 发射场景（仅含非本机 emission；可为空表示无外部干扰）。
   *
   * 库内每周期构建自发射并合并本帧后，经 `TryResolveRirRfFrontEnd` 求解
   * incident links。窗口须与 `sim_time_sec` / 会话 `recognition_dwell_sec` 对齐。
   */
  oneq::electromagnetics::RfSceneFrame rf_scene{};
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
