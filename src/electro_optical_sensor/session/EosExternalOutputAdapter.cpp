#include "1q/electro_optical_sensor/session/EosExternalOutputAdapter.h"

#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "common/numerics/Constants.h"
#include "common/validation/ValidationUtils.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

namespace electro_optical_sensor {
namespace session {

namespace {

oneq::coordinate::Vector3d ToPlatformFrameVector(float range_m, float azimuth_deg,
                                                 float elevation_deg) {
  const double range = static_cast<double>(range_m);
  const double az_rad = oneq::common::numerics::DegToRad(static_cast<double>(azimuth_deg));
  const double el_rad = oneq::common::numerics::DegToRad(static_cast<double>(elevation_deg));
  const double horizontal = range * std::cos(el_rad);
  oneq::coordinate::Vector3d vector;
  vector.x = horizontal * std::cos(az_rad);
  vector.y = horizontal * std::sin(az_rad);
  vector.z = range * std::sin(el_rad);
  return vector;
}

oneq::coordinate::EulerAnglesDeg ToCoordinateEuler(
    const oneq::foundation::EulerAnglesDeg& attitude_deg) {
  oneq::coordinate::EulerAnglesDeg output;
  output.yaw_deg = static_cast<double>(attitude_deg.yaw_deg);
  output.pitch_deg = static_cast<double>(attitude_deg.pitch_deg);
  output.roll_deg = static_cast<double>(attitude_deg.roll_deg);
  return output;
}

oneq::coordinate::EnuPositionM Vector3dToEnuPosition(const oneq::coordinate::Vector3d& enu) {
  oneq::coordinate::EnuPositionM pos;
  pos.east_m = enu.x;
  pos.north_m = enu.y;
  pos.up_m = enu.z;
  return pos;
}

}  // namespace

bool TryMakeExternalDetectionFromRecord(const output::EosDetectionRecord& detection,
                                        const oneq::coordinate::LocalFrameReference& reference,
                                        const oneq::foundation::PoseState& platform_pose,
                                        EosExternalDetectionRecord* output) {
  if (output == nullptr || !oneq::common::validation::IsFinite(detection.range_m) ||
      !oneq::common::validation::IsFinite(detection.azimuth_deg) ||
      !oneq::common::validation::IsFinite(detection.elevation_deg) || detection.range_m <= 0.0f) {
    return false;
  }

  const oneq::coordinate::Vector3d platform_frame =
      ToPlatformFrameVector(detection.range_m, detection.azimuth_deg, detection.elevation_deg);
  const oneq::coordinate::Vector3d relative_local =
      oneq::coordinate::RotateLocalToEnu(platform_frame.x, platform_frame.y, platform_frame.z,
                                         ToCoordinateEuler(platform_pose.attitude_deg));
  oneq::coordinate::Vector3d target_local;
  target_local.x = static_cast<double>(platform_pose.position_m.x) + relative_local.x;
  target_local.y = static_cast<double>(platform_pose.position_m.y) + relative_local.y;
  target_local.z = static_cast<double>(platform_pose.position_m.z) + relative_local.z;
  const oneq::coordinate::Vector3d target_enu = oneq::coordinate::RotateLocalToEnu(
      target_local.x, target_local.y, target_local.z, reference.frame_attitude_deg);

  oneq::coordinate::EcefPositionM target_ecef;
  if (!oneq::coordinate::TryEnuToEcef(Vector3dToEnuPosition(target_enu), reference.origin_lla,
                                      &target_ecef)) {
    return false;
  }

  output->detection_id = detection.detection_id;
  output->target_position_ecef_m = target_ecef;
  output->range_m = detection.range_m;
  output->azimuth_deg = detection.azimuth_deg;
  output->elevation_deg = detection.elevation_deg;
  output->infrared_snr_linear = detection.infrared_snr_linear;
  output->visible_snr_linear = detection.visible_snr_linear;
  output->fused_snr_linear = detection.fused_snr_linear;
  output->fused_snr_db = detection.fused_snr_db;
  output->detected = detection.detected;
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
