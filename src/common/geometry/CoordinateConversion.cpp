#include "common/geometry/CoordinateConversion.h"

#include <cmath>

#include "common/geometry/GeometryTransform.h"

namespace oneq {
namespace internal {
namespace geometry {

namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(
    const oneq::foundation::EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

}  // namespace

oneq::foundation::Vector3f ConvertEnuToLocal(
    const oneq::foundation::EnuCoordinateM& enu,
    const oneq::foundation::EulerAnglesDeg& attitude_deg) {
  oneq::internal::geometry::Vector3f enu_vector;
  enu_vector.x = static_cast<float>(enu.x_m);
  enu_vector.y = static_cast<float>(enu.y_m);
  enu_vector.z = static_cast<float>(enu.z_m);
  const oneq::internal::geometry::Vector3f local_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(enu_vector,
                                                         ToGeometryEuler(attitude_deg));

  oneq::foundation::Vector3f local;
  local.x = local_vector.x;
  local.y = local_vector.y;
  local.z = local_vector.z;
  return local;
}

bool TryConvertEcefVelocityToEnu(
    const oneq::foundation::Vector3f& velocity_ecef_mps,
    const oneq::foundation::LlaCoordinateDegM& origin_lla,
    oneq::foundation::EnuCoordinateM* velocity_enu_mps) {
  if (velocity_enu_mps == nullptr || !oneq::foundation::IsValidLla(origin_lla) ||
      !IsFiniteVector3f(velocity_ecef_mps)) {
    return false;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double lat_rad = origin_lla.latitude_deg * kPi / 180.0;
  const double lon_rad = origin_lla.longitude_deg * kPi / 180.0;
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  const double vx = static_cast<double>(velocity_ecef_mps.x);
  const double vy = static_cast<double>(velocity_ecef_mps.y);
  const double vz = static_cast<double>(velocity_ecef_mps.z);

  velocity_enu_mps->x_m = -sin_lon * vx + cos_lon * vy;
  velocity_enu_mps->y_m = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz;
  velocity_enu_mps->z_m = cos_lat * cos_lon * vx + cos_lat * sin_lon * vy + sin_lat * vz;
  return std::isfinite(velocity_enu_mps->x_m) != 0 && std::isfinite(velocity_enu_mps->y_m) != 0 &&
         std::isfinite(velocity_enu_mps->z_m) != 0;
}

oneq::foundation::EnuCoordinateM ToEnuFromNed(
    const oneq::foundation::Vector3f& ned_mps) {
  oneq::foundation::EnuCoordinateM enu;
  enu.x_m = static_cast<double>(ned_mps.y);
  enu.y_m = static_cast<double>(ned_mps.x);
  enu.z_m = static_cast<double>(-ned_mps.z);
  return enu;
}

oneq::foundation::EnuCoordinateM ToEnuFromEnuVector(
    const oneq::foundation::Vector3f& enu_mps) {
  oneq::foundation::EnuCoordinateM enu;
  enu.x_m = static_cast<double>(enu_mps.x);
  enu.y_m = static_cast<double>(enu_mps.y);
  enu.z_m = static_cast<double>(enu_mps.z);
  return enu;
}

bool TryConvertEcefPositionToLocal(
    const oneq::foundation::EcefCoordinateM& ecef,
    const LocalFrameReference& reference,
    oneq::foundation::Vector3f* local) {
  if (local == nullptr) {
    return false;
  }

  oneq::foundation::EnuCoordinateM enu;
  if (!oneq::foundation::TryEcefToEnu(ecef, reference.origin_lla, &enu)) {
    return false;
  }
  *local = ConvertEnuToLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool TryConvertVelocityToLocal(
    const oneq::foundation::Vector3f& velocity,
    VelocityFrame frame,
    const LocalFrameReference& reference,
    oneq::foundation::Vector3f* local) {
  if (local == nullptr || !IsFiniteVector3f(velocity)) {
    return false;
  }

  switch (frame) {
    case VelocityFrame::kLocal:
      *local = velocity;
      return true;
    case VelocityFrame::kEcef: {
      oneq::foundation::EnuCoordinateM velocity_enu;
      if (!TryConvertEcefVelocityToEnu(velocity, reference.origin_lla, &velocity_enu)) {
        return false;
      }
      *local = ConvertEnuToLocal(velocity_enu, reference.frame_attitude_deg);
      return true;
    }
    case VelocityFrame::kEnu: {
      const oneq::foundation::EnuCoordinateM velocity_enu = ToEnuFromEnuVector(velocity);
      *local = ConvertEnuToLocal(velocity_enu, reference.frame_attitude_deg);
      return true;
    }
    case VelocityFrame::kNed: {
      const oneq::foundation::EnuCoordinateM velocity_enu = ToEnuFromNed(velocity);
      *local = ConvertEnuToLocal(velocity_enu, reference.frame_attitude_deg);
      return true;
    }
  }
  return false;
}

}  // namespace geometry
}  // namespace internal
}  // namespace oneq
