/**
 * @file RadarSceneTargetUtils.h
 * @brief 定义面向公开场景输入的雷达目标构造与几何规范化工具。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SCENE_TARGET_UTILS_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SCENE_TARGET_UTILS_H_

#include <cstdint>

#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief 根据雷达局部笛卡尔坐标构造公开场景目标输入。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] position_z 目标局部 z 坐标（米）。
 * @param[in] velocity_x 目标速度 x 分量（m/s）。
 * @param[in] velocity_y 目标速度 y 分量（m/s）。
 * @param[in] velocity_z 目标速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return 已写入位置、速度与斜距的公开场景目标。
 */
ONEQ_API RadarSceneTarget MakeSceneTarget(std::uint64_t external_target_id, float position_x,
                                          float position_y, float position_z, float velocity_x,
                                          float velocity_y, float velocity_z, float rcs,
                                          int swerling_type = 0);

/**
 * @brief 构造地面场景目标（z=0）。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] velocity_x 地面目标速度 x 分量（m/s）。
 * @param[in] velocity_y 地面目标速度 y 分量（m/s）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return z=0 的公开场景目标。
 */
ONEQ_API RadarSceneTarget MakeGroundSceneTarget(std::uint64_t external_target_id, float position_x,
                                                float position_y, float rcs = 1.0f,
                                                float velocity_x = 0.0f, float velocity_y = 0.0f,
                                                int swerling_type = 0);

/**
 * @brief 构造空中场景目标。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] position_z 目标局部 z 坐标（米）。
 * @param[in] velocity_x 目标速度 x 分量（m/s）。
 * @param[in] velocity_y 目标速度 y 分量（m/s）。
 * @param[in] velocity_z 目标速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return 已写入三维位置与斜距的公开场景目标。
 */
ONEQ_API RadarSceneTarget MakeAirSceneTarget(std::uint64_t external_target_id, float position_x,
                                             float position_y, float position_z, float velocity_x,
                                             float velocity_y, float velocity_z, float rcs = 1.0f,
                                             int swerling_type = 0);

/**
 * @brief 规范化单个场景目标的几何派生量。
 * @details 当 range_m 未提供（<=0）时，从笛卡尔位置范数回填斜距。
 * @param[in,out] target 场景目标指针，可为 nullptr。
 */
ONEQ_API void NormalizeSceneTargetGeometry(RadarSceneTarget* target);

/**
 * @brief 批量规范化场景目标几何派生量。
 * @param[in,out] targets 场景目标列表指针，可为 nullptr。
 */
ONEQ_API void NormalizeSceneTargetGeometry(RadarSceneTargetList* targets);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SCENE_TARGET_UTILS_H_
