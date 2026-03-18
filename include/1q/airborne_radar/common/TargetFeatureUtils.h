// Copyright 2026. All Rights Reserved.
//
// @file TargetFeatureUtils.h
// @brief 定义面向外部调用方的目标构造与几何规范化工具。

#ifndef AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_
#define AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_

#include <cstdint>

#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace common {

/// @brief 根据雷达局部笛卡尔坐标构造单个目标。
/// @param external_target_id 外部目标标识符。
/// @param position_x 目标局部 x 坐标（米）。
/// @param position_y 目标局部 y 坐标（米）。
/// @param position_z 目标局部 z 坐标（米）。
/// @param velocity_x 目标速度 x 分量（m/s）。
/// @param velocity_y 目标速度 y 分量（m/s）。
/// @param velocity_z 目标速度 z 分量（m/s）。
/// @param rcs 目标 RCS（平方米）。
/// @param acceleration_x 目标加速度 x 分量（m/s^2）。
/// @param acceleration_y 目标加速度 y 分量（m/s^2）。
/// @param acceleration_z 目标加速度 z 分量（m/s^2）。
/// @param swerling_type 目标起伏模型编号。
/// @return 已写入位置、速度与斜距的目标特征。
TargetFeature MakeTargetFromCartesian(
    std::uint64_t external_target_id,
    float position_x,
    float position_y,
    float position_z,
    float velocity_x,
    float velocity_y,
    float velocity_z,
    float rcs,
    float acceleration_x = 0.0f,
    float acceleration_y = 0.0f,
    float acceleration_z = 0.0f,
    int swerling_type = 0);

/// @brief 构造地面目标。
/// @param external_target_id 外部目标标识符。
/// @param position_x 目标局部 x 坐标（米）。
/// @param position_y 目标局部 y 坐标（米）。
/// @param rcs 目标 RCS（平方米）。
/// @param velocity_x 地面目标速度 x 分量（m/s）。
/// @param velocity_y 地面目标速度 y 分量（m/s）。
/// @param acceleration_x 地面目标加速度 x 分量（m/s^2）。
/// @param acceleration_y 地面目标加速度 y 分量（m/s^2）。
/// @param swerling_type 目标起伏模型编号。
/// @return `z=0` 的目标特征。
TargetFeature MakeGroundTarget(
    std::uint64_t external_target_id,
    float position_x,
    float position_y,
    float rcs = 1.0f,
    float velocity_x = 0.0f,
    float velocity_y = 0.0f,
    float acceleration_x = 0.0f,
    float acceleration_y = 0.0f,
    int swerling_type = 0);

/// @brief 构造空中目标。
/// @param external_target_id 外部目标标识符。
/// @param position_x 目标局部 x 坐标（米）。
/// @param position_y 目标局部 y 坐标（米）。
/// @param position_z 目标局部 z 坐标（米）。
/// @param velocity_x 目标速度 x 分量（m/s）。
/// @param velocity_y 目标速度 y 分量（m/s）。
/// @param velocity_z 目标速度 z 分量（m/s）。
/// @param rcs 目标 RCS（平方米）。
/// @param acceleration_x 目标加速度 x 分量（m/s^2）。
/// @param acceleration_y 目标加速度 y 分量（m/s^2）。
/// @param acceleration_z 目标加速度 z 分量（m/s^2）。
/// @param swerling_type 目标起伏模型编号。
/// @return 已写入三维位置与斜距的目标特征。
TargetFeature MakeAirTarget(
    std::uint64_t external_target_id,
    float position_x,
    float position_y,
    float position_z,
    float velocity_x,
    float velocity_y,
    float velocity_z,
    float rcs = 1.0f,
    float acceleration_x = 0.0f,
    float acceleration_y = 0.0f,
    float acceleration_z = 0.0f,
    int swerling_type = 0);

/// @brief 规范化单个目标的几何派生量。
/// @details 当 `range_m <= 0` 且存在笛卡尔位置时，会按位置范数回填斜距；
///          当 `range_m <= 0` 且位置全零时，不做静默修复。
/// @param target 目标特征指针，可为 nullptr。
void NormalizeTargetGeometry(TargetFeature* target);

/// @brief 批量规范化目标几何派生量。
/// @param targets 目标特征列表指针，可为 nullptr。
void NormalizeTargetGeometry(TargetFeatureList* targets);

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_
