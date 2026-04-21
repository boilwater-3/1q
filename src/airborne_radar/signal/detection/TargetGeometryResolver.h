/**
 * @file TargetGeometryResolver.h
 * @brief Unified target geometry resolver used by airborne detection pipeline.
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_

#include <Eigen/Core>
#include <algorithm>

#include "1q/airborne_radar/model/TargetFeature.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {

struct ResolvedTargetGeometry {
  bool has_cartesian_position{false};
  Eigen::Vector3f position_m{Eigen::Vector3f::Zero()};
  float range_m{50000.0f};
  TargetLookAnglesDeg look_angles_deg;
};

class TargetGeometryResolver {
 public:
  static ResolvedTargetGeometry Resolve(const model::TargetFeature& target) {
    ResolvedTargetGeometry geometry;
    geometry.position_m =
        Eigen::Vector3f(target.position_x, target.position_y, target.position_z);
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

    constexpr float kFallbackRangeMeters = 50000.0f;
    geometry.range_m = kFallbackRangeMeters;
    return geometry;
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_TARGET_GEOMETRY_RESOLVER_H_
