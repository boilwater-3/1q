// Copyright 2026. All Rights Reserved.
//
// Description: 定义表示当前雷达处理周期内目标的核心状态数据结构。

#ifndef AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_
#define AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_

#include <cmath>
#include <cstdint>
#include <vector>

namespace airborne_radar {
namespace common {

/// @brief TargetFeature 封装了单个处理周期的目标输入状态。
/// 它作为只读的有效载荷在行为决策层管线中传递。
/// TODO 目前仅包含几个核心特征，后续可根据需要扩展更多特征字段。
struct TargetFeature {
  // @brief 外部输入原始目标标识符（0 表示未知/未提供）。
  std::uint64_t external_target_id{0};

  // @brief 目标速度向量（单位：m/s），[vx, vy, vz]。
  float current_track_velocity_x{0.0f};
  float current_track_velocity_y{0.0f};
  float current_track_velocity_z{0.0f};

  // @brief 当前被跟踪主要目标速度模长（单位：m/s）。
  // @note 由构造函数根据速度向量自动计算赋值，不作为输入参数。
  float current_track_speed{0.0f};

  // @brief 目标的估计雷达散射截面积（RCS）（单位：平方米）。
  float current_track_rcs{0.0f};

  // @brief 目标加速度向量（单位：m/s^2），[ax, ay, az]。
  float current_track_acceleration_x{0.0f};
  float current_track_acceleration_y{0.0f};
  float current_track_acceleration_z{0.0f};

  // @brief 当前被跟踪主要目标加速度模长（单位：m/s^2）。
  // @note 由构造函数根据加速度向量自动计算赋值，不作为输入参数。
  float current_track_acceleration{0.0f};

  // @brief 目标到雷达的斜距（单位：m）。
  float range_m{0.0f};

  // @brief 雷达局部笛卡尔坐标位置（单位：m），[x, y, z]。
  // @note 该坐标系以当前雷达为原点，仅接受雷达局部坐标输入。
  // 注：不使用 Eigen 以保持 POD 布局，Pipeline 内部按需转换。
  float position_x{0.0f};
  float position_y{0.0f};
  float position_z{0.0f};

  // @brief 目标的 Swerling 起伏模型，默认 0 (kSwerling0，无起伏)
  // 保持整数类型以避免在公开头文件中引入底层检测枚举依赖
  int target_swerling_type{0};

  // @brief 默认构造函数
  TargetFeature() = default;

  // @brief 带参数的构造函数，为了方便初始化。
  // @note 速度/加速度模长由 [vx,vy,vz] 与 [ax,ay,az] 自动计算，调用方只输入向量。
  TargetFeature(float velocity_x, float velocity_y, float velocity_z,
      float rcs,
      float acceleration_x = 0.0f, float acceleration_y = 0.0f,
      float acceleration_z = 0.0f,
      float range = 0.0f, int swerling_type = 0,
      std::uint64_t ext_target_id = 0)
      : external_target_id(ext_target_id),
        current_track_velocity_x(velocity_x),
        current_track_velocity_y(velocity_y),
        current_track_velocity_z(velocity_z),
        current_track_speed(std::sqrt(velocity_x * velocity_x +
                                      velocity_y * velocity_y +
                                      velocity_z * velocity_z)),
        current_track_rcs(rcs),
        current_track_acceleration_x(acceleration_x),
        current_track_acceleration_y(acceleration_y),
        current_track_acceleration_z(acceleration_z),
        current_track_acceleration(std::sqrt(acceleration_x * acceleration_x +
                                             acceleration_y * acceleration_y +
                                             acceleration_z * acceleration_z)),
        range_m(range),
        target_swerling_type(swerling_type) {}
};

/// @brief TargetFeatureList 是 TargetFeature 的列表，表示当前处理周期内所有相关目标的特征集合。
using TargetFeatureList = std::vector<TargetFeature>;

} // namespace common
} // namespace airborne_radar


#endif // AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_
