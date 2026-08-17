/**
 * @file TargetGeometryResolver.h
 * @brief 定义目标几何（位置、斜距、look angle）的统一解析器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_

#include <Eigen/Core>
#include <algorithm>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief 目标几何解析结果，统一提供给探测链路使用。
 */
struct ResolvedTargetGeometry {
  Eigen::Vector3f position_m{Eigen::Vector3f::Zero()}; /**< 目标在雷达局部坐标系下的位置（单位：m）。 */
  float range_m{50000.0f};                             /**< 目标到雷达的斜距（单位：m），默认回退为大值。 */
  TargetLookAnglesDeg look_angles_deg;                 /**< 目标在雷达局部坐标系下的 look angle。 */
};

/**
 * @brief TargetGeometryResolver 负责从目标特征统一解析位置、斜距与 look angle。
 */
class TargetGeometryResolver {
 public:
  /**
   * @brief 从目标特征解析几何信息。
   * @param target 目标输入特征。
   * @return 包含位置、斜距与 look angle 的解析结果。
   * @note 斜距优先使用 target.range_m；当其非正时按位置向量范数计算，并钳位到 0.1m 避免除零。
   */
  static ResolvedTargetGeometry Resolve(const session::ArSceneTarget& target) {
    ResolvedTargetGeometry geometry;
    geometry.position_m =
        Eigen::Vector3f(target.position_x, target.position_y, target.position_z);
    geometry.look_angles_deg = TargetLookResolver::Resolve(target);

    if (target.range_m > 0.0f) {
      geometry.range_m = target.range_m;
      return geometry;
    }

    geometry.range_m = std::max(geometry.position_m.norm(), 0.1f);
    return geometry;
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_
