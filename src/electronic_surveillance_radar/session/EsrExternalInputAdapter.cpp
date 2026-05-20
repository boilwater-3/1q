#include <cmath>

#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

void SetStatus(EsrCoordinateStatus value, EsrCoordinateStatus* status) {
  if (status != nullptr) {
    *status = value;
  }
}

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const EsrVector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

EsrEulerAngleDeg ToEsrEuler(const oneq::coordinate::EulerAnglesDeg& attitude_deg) {
  EsrEulerAngleDeg output;
  output.yaw_deg = static_cast<float>(attitude_deg.yaw_deg);
  output.pitch_deg = static_cast<float>(attitude_deg.pitch_deg);
  output.roll_deg = static_cast<float>(attitude_deg.roll_deg);
  return output;
}

EsrVector3f ToEsrVector(const oneq::coordinate::Vector3d& v) {
  EsrVector3f out;
  out.x = static_cast<float>(v.x);
  out.y = static_cast<float>(v.y);
  out.z = static_cast<float>(v.z);
  return out;
}

EsrVector3f RotateEnuPositionToLocal(
    const oneq::coordinate::EnuPositionM& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToEsrVector(
      oneq::coordinate::RotateEnuToLocal(enu.east_m, enu.north_m, enu.up_m, local_attitude_deg));
}

EsrVector3f RotateEnuVelocityToLocal(
    const oneq::coordinate::EnuVelocityMps& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToEsrVector(oneq::coordinate::RotateEnuToLocal(
      enu.east_mps, enu.north_mps, enu.up_mps, local_attitude_deg));
}

bool TryConvertEcefPositionToLocal(const oneq::coordinate::EcefPositionM& ecef,
                                   const EsrCoordinateReference& reference,
                                   EsrVector3f* local) {
  if (local == nullptr) {
    return false;
  }
  oneq::coordinate::EnuPositionM enu;
  if (!oneq::coordinate::TryEcefToEnu(ecef, reference.origin_lla, &enu)) {
    return false;
  }
  *local = RotateEnuPositionToLocal(enu, reference.frame_attitude_deg);
  return IsFiniteVector3f(*local);
}

bool TryConvertLlaPositionToLocal(const oneq::coordinate::LlaPositionDegM& lla,
                                  const EsrCoordinateReference& reference,
                                  EsrVector3f* local) {
  if (local == nullptr) {
    return false;
  }
  oneq::coordinate::EnuPositionM enu;
  if (!oneq::coordinate::TryLlaToEnu(lla, reference.origin_lla, &enu)) {
    return false;
  }
  *local = RotateEnuPositionToLocal(enu, reference.frame_attitude_deg);
  return IsFiniteVector3f(*local);
}

bool TryConvertEcefVelocityToLocal(const oneq::coordinate::EcefVelocityMps& ecef,
                                   const EsrCoordinateReference& reference,
                                   EsrVector3f* local) {
  if (local == nullptr) {
    return false;
  }
  oneq::coordinate::EnuVelocityMps enu;
  if (!oneq::coordinate::TryEcefToEnuVelocity(ecef, reference.origin_lla, &enu)) {
    return false;
  }
  *local = RotateEnuVelocityToLocal(enu, reference.frame_attitude_deg);
  return IsFiniteVector3f(*local);
}

}  // namespace

bool TryMakeEsrPoseFromExternalKinematics(const EsrExternalPoseInput& input,
                                          const EsrCoordinateReference& reference,
                                          session::EsrPoseState* pose,
                                          EsrCoordinateStatus* status) {
  if (pose == nullptr) {
    SetStatus(EsrCoordinateStatus::kNullOutput, status);
    return false;
  }

  EsrVector3f local_position;
  if (!TryConvertEcefPositionToLocal(input.platform_position_ecef_m, reference,
                                     &local_position)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  if (!oneq::coordinate::IsFinite(input.platform_velocity_mps)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  EsrVector3f local_velocity;
  if (!TryConvertEcefVelocityToLocal(input.platform_velocity_mps, reference, &local_velocity)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  pose->position_m = local_position;
  pose->velocity_mps = local_velocity;
  pose->attitude_deg = ToEsrEuler(input.platform_attitude_deg);
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

  EsrVector3f local_position;
  if (input.kinematics.position_frame == oneq::coordinate::PositionFrame::kLla) {
    if (!TryConvertLlaPositionToLocal(input.kinematics.position_lla_deg_m, reference,
                                      &local_position)) {
      SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
      return false;
    }
  } else {
    if (!TryConvertEcefPositionToLocal(input.kinematics.position_ecef_m, reference,
                                       &local_position)) {
      SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
      return false;
    }
  }

  if (!oneq::coordinate::IsFinite(input.kinematics.velocity_mps)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  EsrVector3f local_velocity;
  if (!TryConvertEcefVelocityToLocal(input.kinematics.velocity_mps, reference, &local_velocity)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  emitter->emitter_id = input.emitter_id;
  emitter->pose.position_m = local_position;
  emitter->pose.velocity_mps = local_velocity;
  emitter->pose.attitude_deg = ToEsrEuler(input.kinematics.attitude_deg);
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
