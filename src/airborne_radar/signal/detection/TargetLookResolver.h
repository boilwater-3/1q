/**
 * @file TargetLookResolver.h
 * @brief 定义基于雷达局部坐标解析目标 look angle 的私有工具。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_LOOK_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_LOOK_RESOLVER_H_

#include <cmath>

#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace signal {
namespace detection {
/**
 * @brief TargetLookAnglesDeg 表示目标在雷达局部坐标系下的 look angle。
 */
struct TargetLookAnglesDeg {
  float look_az_deg{0.0f};   /**< 目标方位角（单位：度）。 */
  float look_el_deg{0.0f};   /**< 目标俯仰角（单位：度）。 */
  bool has_look_angles{false};  /**< 是否成功解析到有效角度。 */
};
/**
 * @brief TargetLookResolver 负责从雷达局部坐标解析目标 look angle。
 */
class TargetLookResolver {
 public:
/**
 * @brief 从目标局部坐标位置解析 look angle。
 * @param target 目标输入特征。
 * @return 目标 look angle 结果。
 * @note 本函数只接受雷达局部笛卡尔坐标；缺失位置时返回 has_look_angles=false。
 */
  static TargetLookAnglesDeg Resolve(const common::TargetFeature& target) {
    TargetLookAnglesDeg result;
    const float position_x = target.position_x;
    const float position_y = target.position_y;
    const float position_z = target.position_z;
    const float horizontal_norm =
        std::sqrt(position_x * position_x + position_y * position_y);
    const float position_norm = std::sqrt(
        position_x * position_x + position_y * position_y +
        position_z * position_z);
    if (position_norm <= 0.1f) {
      return result;
    }

    const float kRad2Deg = 180.0f / 3.14159265358979f;
    result.look_az_deg = std::atan2(position_y, position_x) * kRad2Deg;
    result.look_el_deg = std::atan2(position_z, horizontal_norm) * kRad2Deg;
    result.has_look_angles = true;
    return result;
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_LOOK_RESOLVER_H_
