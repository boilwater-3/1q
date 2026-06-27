#include "common/validation/ValidationUtils.h"
#include "1q/airborne_radar/session/RadarExternalOutputAdapter.h"

#include <cmath>

#include "common/validation/ValidationUtils.h"
#include "1q/coordinate/attitude_transform.h"
#include "common/validation/ValidationUtils.h"
#include "1q/coordinate/position_transform.h"
#include "common/validation/ValidationUtils.h"
#include "1q/coordinate/velocity_transform.h"

namespace airborne_radar {
namespace session {

namespace {


bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return oneq::internal::validation::IsFinite(value.x) && oneq::internal::validation::IsFinite(value.y) && oneq::internal::validation::IsFinite(value.z);
}

oneq::coordinate::EnuPositionM Vector3dToEnuPosition(const oneq::coordinate::Vector3d& enu) {
  oneq::coordinate::EnuPositionM pos;
  pos.east_m = enu.x;
  pos.north_m = enu.y;
  pos.up_m = enu.z;
  return pos;
}

oneq::coordinate::EnuVelocityMps Vector3dToEnuVelocity(const oneq::coordinate::Vector3d& enu) {
  oneq::coordinate::EnuVelocityMps vel;
  vel.east_mps = enu.x;
  vel.north_mps = enu.y;
  vel.up_mps = enu.z;
  return vel;
}

}  // namespace

bool TryMakeExternalTrackFromSnapshot(const session::TrackStateSnapshot& snapshot,
                                      const oneq::coordinate::LocalFrameReference& reference,
                                      oneq::foundation::Vector3f radar_local_velocity_mps,
                                      RadarExternalTrackKinematics* output) {
  if (output == nullptr || !IsFiniteVector3f(radar_local_velocity_mps) ||
      !oneq::internal::validation::IsFinite(snapshot.position_x) || !oneq::internal::validation::IsFinite(snapshot.position_y) ||
      !oneq::internal::validation::IsFinite(snapshot.position_z) || !oneq::internal::validation::IsFinite(snapshot.velocity_x) ||
      !oneq::internal::validation::IsFinite(snapshot.velocity_y) || !oneq::internal::validation::IsFinite(snapshot.velocity_z)) {
    return false;
  }

  const oneq::coordinate::Vector3d position_enu =
      oneq::coordinate::RotateLocalToEnu(
          static_cast<double>(snapshot.position_x), static_cast<double>(snapshot.position_y),
          static_cast<double>(snapshot.position_z), reference.frame_attitude_deg);
  oneq::coordinate::EcefPositionM position_ecef;
  if (!oneq::coordinate::TryEnuToEcef(Vector3dToEnuPosition(position_enu),
                                       reference.origin_lla, &position_ecef)) {
    return false;
  }

  const double absolute_velocity_local_x =
      static_cast<double>(snapshot.velocity_x) + static_cast<double>(radar_local_velocity_mps.x);
  const double absolute_velocity_local_y =
      static_cast<double>(snapshot.velocity_y) + static_cast<double>(radar_local_velocity_mps.y);
  const double absolute_velocity_local_z =
      static_cast<double>(snapshot.velocity_z) + static_cast<double>(radar_local_velocity_mps.z);
  const oneq::coordinate::Vector3d velocity_enu =
      oneq::coordinate::RotateLocalToEnu(
          absolute_velocity_local_x, absolute_velocity_local_y,
          absolute_velocity_local_z, reference.frame_attitude_deg);
  oneq::coordinate::EcefVelocityMps velocity_ecef;
  if (!oneq::coordinate::TryEnuToEcefVelocity(Vector3dToEnuVelocity(velocity_enu),
                                               reference.origin_lla, &velocity_ecef)) {
    return false;
  }

  output->association_key = snapshot.association_key;
  output->external_target_id = snapshot.external_target_id;
  output->target_name = snapshot.target_name;
  output->status = snapshot.status;
  output->target_position_ecef_m = position_ecef;
  output->target_velocity_mps = velocity_ecef;
  output->speed = static_cast<float>(std::sqrt(
      velocity_ecef.x_mps * velocity_ecef.x_mps +
      velocity_ecef.y_mps * velocity_ecef.y_mps +
      velocity_ecef.z_mps * velocity_ecef.z_mps));
  output->rcs = snapshot.rcs;
  output->jamming_detected = snapshot.jamming_detected;
  output->hit_count = snapshot.hit_count;
  output->miss_count = snapshot.miss_count;
  output->target_probability = snapshot.target_probability;
  return true;
}

}  // namespace session
}  // namespace airborne_radar
