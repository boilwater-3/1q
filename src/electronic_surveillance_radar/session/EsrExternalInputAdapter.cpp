#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"

#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "common/coordinate/CoordinateUtils.h"
#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

using oneq::internal::coordinate_utils::RotateEnuPositionToLocal;
using oneq::internal::coordinate_utils::RotateEnuVelocityToLocal;
using oneq::internal::coordinate_utils::ToFoundationEuler;
using oneq::internal::coordinate_utils::ToFoundationVector;

void SetStatus(EsrCoordinateStatus value, EsrCoordinateStatus* status) {
  if (status != nullptr) {
    *status = value;
  }
}

bool IsFiniteVector3f(const EsrVector3f& value) {
  return oneq::internal::validation::IsFinite(value.x) &&
         oneq::internal::validation::IsFinite(value.y) &&
         oneq::internal::validation::IsFinite(value.z);
}

bool TryConvertEcefPositionToLocal(const oneq::coordinate::EcefPositionM& ecef,
                                   const oneq::coordinate::LocalFrameReference& reference,
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
                                  const oneq::coordinate::LocalFrameReference& reference,
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
                                   const oneq::coordinate::LocalFrameReference& reference,
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
                                          const oneq::coordinate::LocalFrameReference& reference,
                                          oneq::foundation::PoseState* pose,
                                          EsrCoordinateStatus* status) {
  if (pose == nullptr) {
    SetStatus(EsrCoordinateStatus::kNullOutput, status);
    return false;
  }

  EsrVector3f local_position;
  if (!TryConvertEcefPositionToLocal(input.platform_position_ecef_m, reference, &local_position)) {
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
  pose->attitude_deg = ToFoundationEuler(input.platform_attitude_deg);
  SetStatus(EsrCoordinateStatus::kOk, status);
  return true;
}

bool TryMakeEsrSceneEmitterFromExternalInput(const EsrExternalEmitterInput& input,
                                             const oneq::coordinate::LocalFrameReference& reference,
                                             EsrSceneEmitter* emitter,
                                             EsrCoordinateStatus* status) {
  if (emitter == nullptr) {
    SetStatus(EsrCoordinateStatus::kNullOutput, status);
    return false;
  }

  EsrVector3f local_position;
  switch (input.kinematics.position_frame) {
    case oneq::coordinate::PositionFrame::kEcef:
      if (!TryConvertEcefPositionToLocal(input.kinematics.position_ecef_m, reference,
                                         &local_position)) {
        SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
        return false;
      }
      break;
    case oneq::coordinate::PositionFrame::kLla:
      if (!TryConvertLlaPositionToLocal(input.kinematics.position_lla_deg_m, reference,
                                        &local_position)) {
        SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
        return false;
      }
      break;
    default:
      SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
      return false;
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
  emitter->emitter_name = input.emitter_name;
  emitter->pose.position_m = local_position;
  emitter->pose.velocity_mps = local_velocity;
  emitter->pose.attitude_deg = ToFoundationEuler(input.kinematics.attitude_deg);
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
