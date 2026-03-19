// Copyright 2026. All Rights Reserved.
//
// @file TargetFeatureUtils.cpp
// @brief 实现目标构造与几何规范化工具。

#include "1q/airborne_radar/common/TargetFeatureUtils.h"

#include <cmath>

namespace airborne_radar {
namespace common {

namespace {

/**
 * @brief 计算三维向量的欧氏范数。
 * @param x x 分量。
 * @param y y 分量。
 * @param z z 分量。
 * @return 三维向量模长。
 */
float ComputeNorm3(float x, float y, float z) {
  return std::sqrt(x * x + y * y + z * z);
}

/**
 * @brief 判断目标是否携带可用的笛卡尔位置分量。
 * @param target 目标特征。
 * @return 位置向量至少有一个非零分量时返回 `true`。
 */
bool HasCartesianPosition(const TargetFeature& target) {
  return target.position_x != 0.0f || target.position_y != 0.0f ||
         target.position_z != 0.0f;
}

/**
 * @brief 刷新目标中由速度和加速度向量派生出的标量字段。
 * @param[in,out] target 待更新的目标特征指针。
 */
void RefreshDerivedKinematics(TargetFeature* target) {
  if (target == nullptr) {
    return;
  }

  target->current_track_speed = ComputeNorm3(
      target->current_track_velocity_x, target->current_track_velocity_y,
      target->current_track_velocity_z);
  target->current_track_acceleration = ComputeNorm3(
      target->current_track_acceleration_x,
      target->current_track_acceleration_y,
      target->current_track_acceleration_z);
}

} // namespace

TargetFeature MakeTargetFromCartesian(
    std::uint64_t external_target_id,
    float position_x,
    float position_y,
    float position_z,
    float velocity_x,
    float velocity_y,
    float velocity_z,
    float rcs,
    float acceleration_x,
    float acceleration_y,
    float acceleration_z,
    int swerling_type) {
  TargetFeature target(velocity_x, velocity_y, velocity_z, rcs, acceleration_x,
                       acceleration_y, acceleration_z, 0.0f, swerling_type,
                       external_target_id);
  target.position_x = position_x;
  target.position_y = position_y;
  target.position_z = position_z;
  NormalizeTargetGeometry(&target);
  return target;
}

TargetFeature MakeGroundTarget(
    std::uint64_t external_target_id,
    float position_x,
    float position_y,
    float rcs,
    float velocity_x,
    float velocity_y,
    float acceleration_x,
    float acceleration_y,
    int swerling_type) {
  return MakeTargetFromCartesian(external_target_id, position_x, position_y,
                                 0.0f, velocity_x, velocity_y, 0.0f, rcs,
                                 acceleration_x, acceleration_y, 0.0f,
                                 swerling_type);
}

TargetFeature MakeAirTarget(
    std::uint64_t external_target_id,
    float position_x,
    float position_y,
    float position_z,
    float velocity_x,
    float velocity_y,
    float velocity_z,
    float rcs,
    float acceleration_x,
    float acceleration_y,
    float acceleration_z,
    int swerling_type) {
  return MakeTargetFromCartesian(external_target_id, position_x, position_y,
                                 position_z, velocity_x, velocity_y,
                                 velocity_z, rcs, acceleration_x,
                                 acceleration_y, acceleration_z,
                                 swerling_type);
}

void NormalizeTargetGeometry(TargetFeature* target) {
  if (target == nullptr) {
    return;
  }

  RefreshDerivedKinematics(target);
  if (target->range_m > 0.0f || !HasCartesianPosition(*target)) {
    return;
  }

  target->range_m =
      ComputeNorm3(target->position_x, target->position_y, target->position_z);
}

void NormalizeTargetGeometry(TargetFeatureList* targets) {
  if (targets == nullptr) {
    return;
  }

  for (std::size_t i = 0; i < targets->size(); ++i) {
    NormalizeTargetGeometry(&(*targets)[i]);
  }
}

} // namespace common
} // namespace airborne_radar
