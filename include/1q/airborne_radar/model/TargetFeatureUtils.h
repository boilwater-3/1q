/**
 * @file TargetFeatureUtils.h
 * @brief 定义面向外部调用方的目标构造与几何规范化工具。
 */

#ifndef AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_
#define AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_

#include <cstdint>

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/api.hpp"
#include "1q/foundation/coordinate_transform.h"

namespace airborne_radar {
namespace model {

/**
 * @brief 根据雷达局部笛卡尔坐标构造单个目标。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] position_z 目标局部 z 坐标（米）。
 * @param[in] velocity_x 目标速度 x 分量（m/s）。
 * @param[in] velocity_y 目标速度 y 分量（m/s）。
 * @param[in] velocity_z 目标速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return 已写入位置、速度与斜距的目标特征。
 */
ONEQ_API model::TargetFeature MakeTargetFromCartesian(std::uint64_t external_target_id,
                                                      float position_x, float position_y,
                                                      float position_z, float velocity_x,
                                                      float velocity_y, float velocity_z, float rcs,
                                                      int swerling_type = 0);

/**
 * @brief 构造地面目标。
 * @note 仅用于测试代码；仿真输入不应显式区分空中/地面目标。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] velocity_x 地面目标速度 x 分量（m/s）。
 * @param[in] velocity_y 地面目标速度 y 分量（m/s）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return `z=0` 的目标特征。
 */
ONEQ_API
model::TargetFeature MakeGroundTarget(std::uint64_t external_target_id, float position_x,
                                      float position_y, float rcs = 1.0f, float velocity_x = 0.0f,
                                      float velocity_y = 0.0f, int swerling_type = 0);

/**
 * @brief 构造空中目标。
 * @note 仅用于测试代码；仿真输入不应显式区分空中/地面目标。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] position_z 目标局部 z 坐标（米）。
 * @param[in] velocity_x 目标速度 x 分量（m/s）。
 * @param[in] velocity_y 目标速度 y 分量（m/s）。
 * @param[in] velocity_z 目标速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return 已写入三维位置与斜距的目标特征。
 */
ONEQ_API
model::TargetFeature MakeAirTarget(std::uint64_t external_target_id, float position_x,
                                   float position_y, float position_z, float velocity_x,
                                   float velocity_y, float velocity_z, float rcs = 1.0f,
                                   int swerling_type = 0);

/**
 * @brief 规范化单个目标的几何派生量。
 * @details 当 `range_m <= 0` 且 `has_cartesian_position=true` 时，会按位置范数回填斜距；
 *          当 `range_m <= 0` 且位置全零时，不做静默修复。
 * @param[in,out] target 目标特征指针，可为 nullptr。
 */
ONEQ_API void NormalizeTargetGeometry(model::TargetFeature* target);

/**
 * @brief 批量规范化目标几何派生量。
 * @param[in,out] targets 目标特征列表指针，可为 nullptr。
 */
ONEQ_API void NormalizeTargetGeometry(model::TargetFeatureList* targets);

}  // namespace model
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_
