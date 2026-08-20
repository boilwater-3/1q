/**
 * @file ArSceneTargetUtils.h
 * @brief 雷达场景目标构造与几何规范化工具（内部头，不对外暴露）。
 */

#ifndef AIRBORNE_RADAR_SESSION_AR_SCENE_TARGET_UTILS_H_
#define AIRBORNE_RADAR_SESSION_AR_SCENE_TARGET_UTILS_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 根据雷达局部笛卡尔坐标构造场景目标。
 * @param[in] external_target_id 外部目标唯一标识。
 * @param[in] position_x 局部笛卡尔 X 坐标（单位：m）。
 * @param[in] position_y 局部笛卡尔 Y 坐标（单位：m）。
 * @param[in] position_z 局部笛卡尔 Z 坐标（单位：m）。
 * @param[in] velocity_x X 方向速度（单位：m/s）。
 * @param[in] velocity_y Y 方向速度（单位：m/s）。
 * @param[in] velocity_z Z 方向速度（单位：m/s）。
 * @param[in] rcs 雷达散射截面积（单位：m²）。
 * @param[in] swerling_type Swerling 起伏模型类型，默认 0（非起伏）。
 * @param[in] target_name 可选目标名称，默认为空。
 * @return 已规范化几何派生量（range 等）的 ArSceneTarget。
 */
ArSceneTarget MakeSceneTarget(std::uint64_t external_target_id, float position_x,
                              float position_y, float position_z, float velocity_x,
                              float velocity_y, float velocity_z, float rcs,
                              int swerling_type = 0, std::string target_name = {});

/**
 * @brief 规范化单个场景目标的几何派生量（若 range_m 未提供则从位置范数回填）。
 * @param[in,out] target 待规范化的场景目标，原地更新其 range_m 等派生量。
 */
void NormalizeSceneTargetGeometry(ArSceneTarget* target);

/**
 * @brief 批量规范化场景目标几何派生量。
 * @param[in,out] targets 待规范化的目标列表，原地逐条更新派生量。
 */
void NormalizeSceneTargetGeometry(ArSceneTargetList* targets);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_SCENE_TARGET_UTILS_H_
