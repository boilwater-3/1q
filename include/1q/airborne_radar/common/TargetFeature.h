// Copyright 2026. All Rights Reserved.
//
// Description: 定义表示当前雷达处理周期内目标的核心状态数据结构。

#ifndef AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_
#define AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_

#include <vector>

namespace airborne_radar {
namespace common {

/// @brief TargetFeature 封装了单个处理周期的目标输入状态。
/// 它作为只读的有效载荷在行为决策层管线中传递。
/// TODO 目前仅包含几个核心特征，后续可根据需要扩展更多特征字段。
struct TargetFeature {
  // @brief 当前被跟踪主要目标的速度（单位：m/s）。
  float current_track_speed{0.0f};

  // @brief 目标的估计雷达散射截面积（RCS）（单位：平方米）。
  float current_track_rcs{0.0f};

  // @brief 指示环境层是否检测到活跃的电子干扰。
  bool check_jamming_detected{false};

  // @brief 保留用于其他航迹特征，例如：加速度、高度等。
  float current_track_acceleration{0.0f};

  // @brief 目标到雷达的斜距（单位：m）。
  float range_m{0.0f};

  // @brief 笛卡尔空间位置（单位：m），[x, y, z]。
  // 注：不使用 Eigen 以保持 POD 布局，Pipeline 内部按需转换。
  float position_x{0.0f};
  float position_y{0.0f};
  float position_z{0.0f};

  // @brief 目标的 Swerling 起伏模型，默认 0 (kSwerling0，无起伏)
  // 保持整数类型以避免在公开头文件中引入底层检测枚举依赖
  int target_swerling_type{0};

  // @brief 默认构造函数
  TargetFeature() = default;

  // @brief 带参数的构造函数，为了方便初始化
  TargetFeature(float speed, float rcs, bool jamming_detected,
                float accel = 0.0f, float range = 0.0f, int swerling_type = 0)
      : current_track_speed(speed), current_track_rcs(rcs),
        check_jamming_detected(jamming_detected),
        current_track_acceleration(accel),
        range_m(range),
        target_swerling_type(swerling_type) {}
};

/// @brief TargetFeatureList 是 TargetFeature 的列表，表示当前处理周期内所有相关目标的特征集合。
using TargetFeatureList = std::vector<TargetFeature>;

} // namespace common
} // namespace airborne_radar


#endif // AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_
