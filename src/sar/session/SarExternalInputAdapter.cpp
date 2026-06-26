// @file SarExternalInputAdapter.cpp
// @brief Implementation of SAR external pulse coordinate adaptation.

#include "1q/sar/session/SarExternalInputAdapter.h"

#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace sar {
namespace session {

namespace {

bool IsFiniteEcefPosition(const oneq::coordinate::EcefPositionM& value) {
  return std::isfinite(value.x_m) && std::isfinite(value.y_m) && std::isfinite(value.z_m);
}

bool IsFiniteEcefVelocity(const oneq::coordinate::EcefVelocityMps& value) {
  return std::isfinite(value.x_mps) && std::isfinite(value.y_mps) && std::isfinite(value.z_mps);
}

}  // namespace

oneq::coordinate::LocalFrameReference BuildSceneCenterReference(
    const config::SarMissionConfig& mission) {
  oneq::coordinate::LocalFrameReference reference;
  reference.origin_lla.latitude_deg = mission.scene_center_latitude_deg;
  reference.origin_lla.longitude_deg = mission.scene_center_longitude_deg;
  reference.origin_lla.altitude_m = mission.scene_center_altitude_m;
  // SAR 内部使用纯 ENU 轴序，frame_attitude 保持默认零。
  return reference;
}

bool TryMakePulseFromExternalKinematics(
    const SarExternalPulseInput& input,
    const oneq::coordinate::LocalFrameReference& reference,
    SarRawIqFrame::PulseState* pulse,
    SarCoordinateStatus* status) {
  if (status != nullptr) {
    *status = SarCoordinateStatus::kOk;
  }

  if (pulse == nullptr) {
    if (status != nullptr) {
      *status = SarCoordinateStatus::kNullOutput;
    }
    return false;
  }

  // 位置转换到 scene-center-relative ENU。
  oneq::coordinate::EnuPositionM enu_position;
  const auto& kin = input.kinematics;
  bool position_ok = false;
  if (kin.position_frame == oneq::coordinate::PositionFrame::kEcef) {
    if (!IsFiniteEcefPosition(kin.position_ecef_m)) {
      if (status != nullptr) {
        *status = SarCoordinateStatus::kCoordinateTransformFail;
      }
      return false;
    }
    position_ok = oneq::coordinate::TryEcefToEnu(kin.position_ecef_m, reference.origin_lla,
                                                 &enu_position);
  } else if (kin.position_frame == oneq::coordinate::PositionFrame::kLla) {
    position_ok = oneq::coordinate::TryLlaToEnu(kin.position_lla_deg_m, reference.origin_lla,
                                                &enu_position);
  } else {
    if (status != nullptr) {
      *status = SarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }

  if (!position_ok) {
    if (status != nullptr) {
      *status = SarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }

  // 速度转换（固定 ECEF）。
  if (!IsFiniteEcefVelocity(kin.velocity_mps)) {
    if (status != nullptr) {
      *status = SarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  oneq::coordinate::EnuVelocityMps enu_velocity;
  if (!oneq::coordinate::TryEcefToEnuVelocity(kin.velocity_mps, reference.origin_lla,
                                              &enu_velocity)) {
    if (status != nullptr) {
      *status = SarCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }

  // ENU → PulseState（x=East, y=North, z=Up）。
  pulse->pulse_id = input.pulse_id;
  pulse->time_s = input.time_s;
  pulse->position_x_m = enu_position.east_m;
  pulse->position_y_m = enu_position.north_m;
  pulse->position_z_m = enu_position.up_m;
  pulse->velocity_x_mps = enu_velocity.east_mps;
  pulse->velocity_y_mps = enu_velocity.north_mps;
  pulse->velocity_z_mps = enu_velocity.up_mps;

  return true;
}

}  // namespace session
}  // namespace sar
