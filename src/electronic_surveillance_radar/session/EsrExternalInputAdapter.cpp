#include <cmath>

#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "common/geometry/CoordinateConversion.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

void SetStatus(EsrCoordinateStatus value, EsrCoordinateStatus* status) {
  if (status != nullptr) {
    *status = value;
  }
}

/// @brief 将 EsrCoordinateReference 映射到内部共享 LocalFrameReference。
oneq::internal::geometry::LocalFrameReference ToSharedReference(
    const EsrCoordinateReference& ref) {
  oneq::internal::geometry::LocalFrameReference shared;
  shared.origin_lla = ref.origin_lla;
  shared.frame_attitude_deg = ref.frame_attitude_deg;
  return shared;
}

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const EsrVector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
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

  const auto shared_ref = ToSharedReference(reference);

  EsrVector3f local_position;
  if (!oneq::internal::geometry::TryConvertEcefPositionToLocal(
          input.platform_position_ecef_m, shared_ref, &local_position)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  if (!IsFiniteVector3f(input.platform_velocity_mps)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  EsrVector3f local_velocity;
  if (!oneq::internal::geometry::TryConvertVelocityToLocal(
          input.platform_velocity_mps, oneq::internal::geometry::VelocityFrame::kEcef,
          shared_ref, &local_velocity)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
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

  const auto shared_ref = ToSharedReference(reference);

  EsrVector3f local_position;
  if (!oneq::internal::geometry::TryConvertEcefPositionToLocal(
          input.emitter_position_ecef_m, shared_ref, &local_position)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  if (!IsFiniteVector3f(input.emitter_velocity_mps)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  EsrVector3f local_velocity;
  if (!oneq::internal::geometry::TryConvertVelocityToLocal(
          input.emitter_velocity_mps, oneq::internal::geometry::VelocityFrame::kEcef,
          shared_ref, &local_velocity)) {
    SetStatus(EsrCoordinateStatus::kCoordinateTransformFail, status);
    return false;
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
