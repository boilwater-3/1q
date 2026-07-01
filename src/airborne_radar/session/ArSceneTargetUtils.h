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
 */
ArSceneTarget MakeSceneTarget(std::uint64_t external_target_id, float position_x,
                              float position_y, float position_z, float velocity_x,
                              float velocity_y, float velocity_z, float rcs,
                              int swerling_type = 0, std::string target_name = {});

/**
 * @brief 规范化单个场景目标的几何派生量（若 range_m 未提供则从位置范数回填）。
 */
void NormalizeSceneTargetGeometry(ArSceneTarget* target);

/**
 * @brief 批量规范化场景目标几何派生量。
 */
void NormalizeSceneTargetGeometry(ArSceneTargetList* targets);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_SCENE_TARGET_UTILS_H_
