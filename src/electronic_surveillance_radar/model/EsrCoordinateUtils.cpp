#include "1q/electronic_surveillance_radar/model/EsrCoordinateUtils.h"

#include "common/geometry/GeometryTransform.h"

namespace electronic_surveillance_radar {
namespace model {

namespace {

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(const EsrEulerAngleDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

EsrVector3f ConvertEnuToEsrLocal(const oneq::common::EnuCoordinateM& enu,
                                 const EsrEulerAngleDeg& frame_attitude_deg) {
  oneq::internal::geometry::Vector3f enu_vector;
  enu_vector.x = static_cast<float>(enu.x_m);
  enu_vector.y = static_cast<float>(enu.y_m);
  enu_vector.z = static_cast<float>(enu.z_m);
  const oneq::internal::geometry::Vector3f local_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(enu_vector,
                                                         ToGeometryEuler(frame_attitude_deg));

  EsrVector3f local;
  local.x = local_vector.x;
  local.y = local_vector.y;
  local.z = local_vector.z;
  return local;
}

}  // namespace

bool TryConvertEcefToEsrLocal(const oneq::common::EcefCoordinateM& position_ecef_m,
                              const EsrCoordinateReference& reference,
                              EsrVector3f* position_local_m) {
  if (position_local_m == nullptr) {
    return false;
  }

  oneq::common::EnuCoordinateM enu;
  if (!oneq::common::TryEcefToEnu(position_ecef_m, reference.origin_lla, &enu)) {
    return false;
  }

  *position_local_m = ConvertEnuToEsrLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool TryConvertLlaToEsrLocal(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                             const EsrCoordinateReference& reference,
                             EsrVector3f* position_local_m) {
  if (position_local_m == nullptr) {
    return false;
  }

  oneq::common::EnuCoordinateM enu;
  if (!oneq::common::TryLlaToEnu(position_lla_deg_m, reference.origin_lla, &enu)) {
    return false;
  }

  *position_local_m = ConvertEnuToEsrLocal(enu, reference.frame_attitude_deg);
  return true;
}

bool TryMakeEsrPoseFromEcef(const oneq::common::EcefCoordinateM& position_ecef_m,
                            const EsrCoordinateReference& reference,
                            const EsrVector3f& velocity_local_mps,
                            const EsrEulerAngleDeg& attitude_deg, EsrPoseState* pose) {
  if (pose == nullptr) {
    return false;
  }

  EsrVector3f local_position;
  if (!TryConvertEcefToEsrLocal(position_ecef_m, reference, &local_position)) {
    return false;
  }

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = attitude_deg;
  return true;
}

bool TryMakeEsrPoseFromLla(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                           const EsrCoordinateReference& reference,
                           const EsrVector3f& velocity_local_mps,
                           const EsrEulerAngleDeg& attitude_deg, EsrPoseState* pose) {
  if (pose == nullptr) {
    return false;
  }

  EsrVector3f local_position;
  if (!TryConvertLlaToEsrLocal(position_lla_deg_m, reference, &local_position)) {
    return false;
  }

  pose->position_m = local_position;
  pose->velocity_mps = velocity_local_mps;
  pose->attitude_deg = attitude_deg;
  return true;
}

}  // namespace model
}  // namespace electronic_surveillance_radar
