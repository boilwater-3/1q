#ifndef COMMON_COORDINATE_COORDINATE_UTILS_H_
#define COMMON_COORDINATE_COORDINATE_UTILS_H_

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/foundation/pose_types.h"

namespace oneq {
namespace internal {
namespace coordinate_utils {

inline oneq::foundation::EulerAnglesDeg ToFoundationEuler(
    const oneq::coordinate::EulerAnglesDeg& attitude_deg) {
  oneq::foundation::EulerAnglesDeg output;
  output.yaw_deg = attitude_deg.yaw_deg;
  output.pitch_deg = attitude_deg.pitch_deg;
  output.roll_deg = attitude_deg.roll_deg;
  return output;
}

inline oneq::foundation::Vector3f ToFoundationVector(
    const oneq::coordinate::Vector3d& v) {
  oneq::foundation::Vector3f out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

inline oneq::foundation::Vector3f RotateEnuPositionToLocal(
    const oneq::coordinate::EnuPositionM& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToFoundationVector(
      oneq::coordinate::RotateEnuToLocal(enu.east_m, enu.north_m, enu.up_m, local_attitude_deg));
}

inline oneq::foundation::Vector3f RotateEnuVelocityToLocal(
    const oneq::coordinate::EnuVelocityMps& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToFoundationVector(oneq::coordinate::RotateEnuToLocal(
      enu.east_mps, enu.north_mps, enu.up_mps, local_attitude_deg));
}

}  // namespace coordinate_utils
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_COORDINATE_COORDINATE_UTILS_H_
