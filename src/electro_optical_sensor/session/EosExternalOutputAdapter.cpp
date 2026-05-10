#include "1q/electro_optical_sensor/session/EosExternalOutputAdapter.h"

#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

namespace electro_optical_sensor {
namespace session {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool IsFinite(float value) { return std::isfinite(value) != 0; }
double DegToRad(double deg) { return deg * kPi / 180.0; }

oneq::coordinate::Vector3d RotateLocalToEnu(
    double local_x, double local_y, double local_z,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  const oneq::coordinate::RotationMatrix3d rotation =
      oneq::coordinate::BuildRotationMatrix(local_attitude_deg);
  oneq::coordinate::Vector3d enu;
  enu.x = rotation.m00 * local_x + rotation.m01 * local_y + rotation.m02 * local_z;
  enu.y = rotation.m10 * local_x + rotation.m11 * local_y + rotation.m12 * local_z;
  enu.z = rotation.m20 * local_x + rotation.m21 * local_y + rotation.m22 * local_z;
  return enu;
}

bool TryEnuPositionToEcef(const oneq::coordinate::Vector3d& enu,
                          const oneq::coordinate::LlaPositionDegM& origin_lla,
                          oneq::coordinate::EcefPositionM* ecef) {
  if (ecef == nullptr || !oneq::coordinate::IsValid(origin_lla)) {
    return false;
  }
  oneq::coordinate::EcefPositionM origin_ecef;
  if (!oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef)) {
    return false;
  }

  const double lat_rad = DegToRad(origin_lla.latitude_deg);
  const double lon_rad = DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  ecef->x_m =
      origin_ecef.x_m - sin_lon * enu.x - sin_lat * cos_lon * enu.y + cos_lat * cos_lon * enu.z;
  ecef->y_m =
      origin_ecef.y_m + cos_lon * enu.x - sin_lat * sin_lon * enu.y + cos_lat * sin_lon * enu.z;
  ecef->z_m = origin_ecef.z_m + cos_lat * enu.y + sin_lat * enu.z;
  return oneq::coordinate::IsFinite(*ecef);
}

oneq::coordinate::Vector3d ToPlatformFrameVector(float range_m, float azimuth_deg,
                                                 float elevation_deg) {
  const double range = static_cast<double>(range_m);
  const double az_rad = DegToRad(static_cast<double>(azimuth_deg));
  const double el_rad = DegToRad(static_cast<double>(elevation_deg));
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

}  // namespace

bool TryMakeExternalDetectionFromRecord(const output::EosDetectionRecord& detection,
                                        const EosCoordinateReference& reference,
                                        const oneq::foundation::PoseState& platform_pose,
                                        EosExternalDetectionRecord* output) {
  if (output == nullptr || !IsFinite(detection.range_m) || !IsFinite(detection.azimuth_deg) ||
      !IsFinite(detection.elevation_deg) || detection.range_m <= 0.0f) {
    return false;
  }

  const oneq::coordinate::Vector3d platform_frame =
      ToPlatformFrameVector(detection.range_m, detection.azimuth_deg, detection.elevation_deg);
  const oneq::coordinate::Vector3d relative_local =
      RotateLocalToEnu(platform_frame.x, platform_frame.y, platform_frame.z,
                       ToCoordinateEuler(platform_pose.attitude_deg));
  oneq::coordinate::Vector3d target_local;
  target_local.x = static_cast<double>(platform_pose.position_m.x) + relative_local.x;
  target_local.y = static_cast<double>(platform_pose.position_m.y) + relative_local.y;
  target_local.z = static_cast<double>(platform_pose.position_m.z) + relative_local.z;
  const oneq::coordinate::Vector3d target_enu = RotateLocalToEnu(
      target_local.x, target_local.y, target_local.z, reference.frame_attitude_deg);

  oneq::coordinate::EcefPositionM target_ecef;
  if (!TryEnuPositionToEcef(target_enu, reference.origin_lla, &target_ecef)) {
    return false;
  }

  output->target_id = detection.target_id;
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
