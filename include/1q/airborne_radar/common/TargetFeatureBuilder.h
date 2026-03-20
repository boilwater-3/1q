/**
 * @file TargetFeatureBuilder.h
 * @brief 提供链式构造目标特征的 Builder，避免高参数构造函数的位置依赖错误。
 */

#ifndef AIRBORNE_RADAR_COMMON_TARGET_FEATURE_BUILDER_H_
#define AIRBORNE_RADAR_COMMON_TARGET_FEATURE_BUILDER_H_

#include <cstdint>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/common/TargetFeatureUtils.h"

namespace airborne_radar {
namespace common {

/**
 * @brief 目标特征链式构造器。
 *
 * 示例：
 * @code
 * auto target = TargetFeatureBuilder(1001U)
 *     .Position(100.0f, 50.0f, 3000.0f)
 *     .Velocity(200.0f, 0.0f, 0.0f)
 *     .Rcs(2.5f)
 *     .Build();
 * @endcode
 *
 * @note 调用 `Build()` 前会自动规范化斜距（同 `NormalizeTargetGeometry`）。
 */
class TargetFeatureBuilder {
 public:
  /**
   * @brief 构造函数，需提供外部目标标识符。
   * @param external_target_id 外部目标标识符（0 表示未知/未提供）。
   */
  explicit TargetFeatureBuilder(std::uint64_t external_target_id) {
    target_.external_target_id = external_target_id;
  }

  /**
   * @brief 设置雷达局部坐标系三维位置（单位：m）。
   */
  TargetFeatureBuilder& Position(float x, float y, float z) {
    target_.position_x = x;
    target_.position_y = y;
    target_.position_z = z;
    return *this;
  }

  /**
   * @brief 设置目标速度向量（单位：m/s）。
   */
  TargetFeatureBuilder& Velocity(float vx, float vy, float vz) {
    target_.current_track_velocity_x = vx;
    target_.current_track_velocity_y = vy;
    target_.current_track_velocity_z = vz;
    return *this;
  }

  /**
   * @brief 设置目标加速度向量（单位：m/s^2）。默认全零。
   */
  TargetFeatureBuilder& Acceleration(float ax, float ay, float az) {
    target_.current_track_acceleration_x = ax;
    target_.current_track_acceleration_y = ay;
    target_.current_track_acceleration_z = az;
    return *this;
  }

  /**
   * @brief 设置目标 RCS（单位：m^2）。默认 1.0f。
   */
  TargetFeatureBuilder& Rcs(float rcs) {
    target_.current_track_rcs = rcs;
    return *this;
  }

  /**
   * @brief 设置目标起伏模型编号（Swerling type）。默认 0。
   */
  TargetFeatureBuilder& SwerlingType(int swerling_type) {
    target_.target_swerling_type = swerling_type;
    return *this;
  }

  /**
   * @brief 生成目标特征，并自动规范化斜距。
   * @return 已规范化几何派生量的目标特征。
   */
  TargetFeature Build() {
    NormalizeTargetGeometry(&target_);
    return target_;
  }

 private:
  TargetFeature target_{};
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_TARGET_FEATURE_BUILDER_H_
