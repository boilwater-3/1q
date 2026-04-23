#include <cmath>

#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "common/geometry/GeometryTransform.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

void SetStatus(EsrCoordinateStatus value, EsrCoordinateStatus* status) {
  if (status != nullptr) {
    *status = value;
  }
}

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(const model::EsrEulerAngleDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

model::EsrVector3f ConvertEnuToEsrLocal(const oneq::foundation::EnuCoordinateM& enu,
                                        const model::EsrEulerAngleDeg& frame_attitude_deg) {
  oneq::internal::geometry::Vector3f enu_vector;
  enu_vector.x = static_cast<float>(enu.x_m);
  enu_vector.y = static_cast<float>(enu.y_m);
  enu_vector.z = static_cast<float>(enu.z_m);
  const oneq::internal::geometry::Vector3f local_vector =
      oneq::internal::geometry::RotateVectorToLocalFrame(enu_vector,
                                                         ToGeometryEuler(frame_attitude_deg));

  model::EsrVector3f local;
  local.x = local_vector.x;
  local.y = local_vector.y;
  local.z = local_vector.z;
  return local;
}

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const model::EsrVector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

oneq::foundation::EnuCoordinateM ToEnuFromNed(const model::EsrVector3f& ned_mps) {
  oneq::foundation::EnuCoordinateM enu;
  enu.x_m = static_cast<double>(ned_mps.y);
  enu.y_m = static_cast<double>(ned_mps.x);
  enu.z_m = static_cast<double>(-ned_mps.z);
  return enu;
}

oneq::foundation::EnuCoordinateM ToEnuFromEnuVector(const model::EsrVector3f& enu_mps) {
  oneq::foundation::EnuCoordinateM enu;
  enu.x_m = static_cast<double>(enu_mps.x);
  enu.y_m = static_cast<double>(enu_mps.y);
  enu.z_m = static_cast<double>(enu_mps.z);
  return enu;
}

bool TryConvertEcefVelocityToEnu(const model::EsrVector3f& velocity_ecef_mps,
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

bool TryConvertEcefToEsrLocalInternal(const oneq::foundation::EcefCoordinateM& position_ecef_m,
                                      const EsrCoordinateReference& reference,
                                      model::EsrVector3f* position_local_m,
                                      EsrCoordinateStatus* status) {
  if (position_local_m == nullptr) {
    SetStatus(EsrCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::foundation::EnuCoordinateM enu;
  if (!oneq::foundation::TryEcefToEnu(position_ecef_m, reference.origin_lla, &enu)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  *position_local_m = ConvertEnuToEsrLocal(enu, reference.frame_attitude_deg);
  return true;
}

}  // namespace

bool TryMakeEsrPoseFromExternalKinematics(const EsrExternalPoseInput& input,
                                          const EsrCoordinateReference& reference,
                                          model::EsrPoseState* pose,
                                          EsrCoordinateStatus* status) {
  if (pose == nullptr) {
    SetStatus(EsrCoordinateStatus::kNullOutput, status);
    return false;
  }

  model::EsrVector3f local_position;
  if (!TryConvertEcefToEsrLocalInternal(input.platform_position_ecef_m, reference,
                                        &local_position, status)) {
    return false;
  }

  if (!IsFiniteVector3f(input.platform_velocity_mps)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  model::EsrVector3f local_velocity;
  switch (input.platform_velocity_frame) {
    case EsrVelocityFrame::kEsrLocal:
      local_velocity = input.platform_velocity_mps;
      break;
    case EsrVelocityFrame::kEcef: {
      oneq::foundation::EnuCoordinateM velocity_enu;
      if (!TryConvertEcefVelocityToEnu(input.platform_velocity_mps, reference.origin_lla,
                                       &velocity_enu)) {
        SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
        return false;
      }
      local_velocity = ConvertEnuToEsrLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
    case EsrVelocityFrame::kEnu: {
      const oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromEnuVector(input.platform_velocity_mps);
      local_velocity = ConvertEnuToEsrLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
    case EsrVelocityFrame::kNed: {
      const oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromNed(input.platform_velocity_mps);
      local_velocity = ConvertEnuToEsrLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
  }

  pose->position_m = local_position;
  pose->velocity_mps = local_velocity;
  pose->attitude_deg = input.platform_attitude_deg;
  SetStatus(EsrCoordinateStatus::kOk, status);
  return true;
}

bool TryMakeEsrSceneEmitterFromExternalInput(const EsrExternalEmitterInput& input,
                                             const EsrCoordinateReference& reference,
                                             EsrSceneEmitter* emitter,
                                             EsrCoordinateStatus* status) {
  if (emitter == nullptr) {
    SetStatus(EsrCoordinateStatus::kNullOutput, status);
    return false;
  }

  model::EsrVector3f local_position;
  if (!TryConvertEcefToEsrLocalInternal(input.emitter_position_ecef_m, reference, &local_position,
                                        status)) {
    return false;
  }

  if (!IsFiniteVector3f(input.emitter_velocity_mps)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  model::EsrVector3f local_velocity;
  switch (input.emitter_velocity_frame) {
    case EsrVelocityFrame::kEsrLocal:
      local_velocity = input.emitter_velocity_mps;
      break;
    case EsrVelocityFrame::kEcef: {
      oneq::foundation::EnuCoordinateM velocity_enu;
      if (!TryConvertEcefVelocityToEnu(input.emitter_velocity_mps, reference.origin_lla,
                                       &velocity_enu)) {
        SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
        return false;
      }
      local_velocity = ConvertEnuToEsrLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
    case EsrVelocityFrame::kEnu: {
      const oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromEnuVector(input.emitter_velocity_mps);
      local_velocity = ConvertEnuToEsrLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
    case EsrVelocityFrame::kNed: {
      const oneq::foundation::EnuCoordinateM velocity_enu =
          ToEnuFromNed(input.emitter_velocity_mps);
      local_velocity = ConvertEnuToEsrLocal(velocity_enu, reference.frame_attitude_deg);
      break;
    }
  }

  emitter->emitter_id = input.emitter_id;
  emitter->pose.position_m = local_position;
  emitter->pose.velocity_mps = local_velocity;
  emitter->pose.attitude_deg = input.emitter_attitude_deg;
  emitter->carrier_hz = input.carrier_hz;
  emitter->bandwidth_hz = input.bandwidth_hz;
  emitter->tx_power_w = input.tx_power_w;
  emitter->pulse_width_s = input.pulse_width_s;
  emitter->pri_s = input.pri_s;
  emitter->beam_state = input.beam_state;
  emitter->is_emitting = input.is_emitting;
  SetStatus(EsrCoordinateStatus::kOk, status);
  return true;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
