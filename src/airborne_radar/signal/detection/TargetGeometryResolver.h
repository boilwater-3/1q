/**
 * @file TargetGeometryResolver.h
 * @brief 定义统一的目标几何解析私有工具。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_

#include <Eigen/Core>
#include <algorithm>

#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {
/**
 * @brief ResolvedTargetGeometry 表示单目标统一解析后的几何信息。
 */
struct ResolvedTargetGeometry {
  bool has_cartesian_position{false};                  /**< 是否具备雷达局部笛卡尔位置。 */
  Eigen::Vector3f position_m{Eigen::Vector3f::Zero()}; /**< 雷达局部笛卡尔位置向量。 */
  float range_m{50000.0f};                             /**< 统一解析后的斜距（m）。 */
  TargetLookAnglesDeg look_angles_deg;                 /**< 目标在雷达局部坐标系下的 look angle。 */
};
/**
 * @brief TargetGeometryResolver 负责统一解析目标几何真值源。
 */
class TargetGeometryResolver {
 public:
  /**
   * @brief 从目标特征解析统一几何信息。
   * @param target 输入目标。
   * @return 统一解析后的目标几何信息。
   */
  static ResolvedTargetGeometry Resolve(const common::model::TargetFeature& target) {
    ResolvedTargetGeometry geometry;
    geometry.position_m = Eigen::Vector3f(target.position_x, target.position_y, target.position_z);
    geometry.has_cartesian_position = target.has_cartesian_position;
    geometry.look_angles_deg = TargetLookResolver::Resolve(target);

    if (target.range_m > 0.0f) {
      geometry.range_m = target.range_m;
      return geometry;
    }

    if (geometry.has_cartesian_position) {
      geometry.range_m = std::max(geometry.position_m.norm(), 0.1f);
      return geometry;
    }

    geometry.range_m = 50000.0f;
    return geometry;
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_
