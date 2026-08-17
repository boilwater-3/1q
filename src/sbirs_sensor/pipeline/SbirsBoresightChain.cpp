#include "sbirs_sensor/pipeline/SbirsBoresightChain.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace pipeline {

namespace {

session::SbirsVector3M LosFromAzimuthElevationDeg(float azimuth_deg, float elevation_deg) {
  const double azimuth_rad = oneq::common::numerics::DegToRad(static_cast<double>(azimuth_deg));
  const double elevation_rad = oneq::common::numerics::DegToRad(static_cast<double>(elevation_deg));
  const double horizontal = std::cos(elevation_rad);
  session::SbirsVector3M los;
  los.x = horizontal * std::cos(azimuth_rad);
  los.y = horizontal * std::sin(azimuth_rad);
  los.z = std::sin(elevation_rad);
  return los;
}

session::SbirsVector3M Multiply(const oneq::coordinate::RotationMatrix3d& rotation,
                                const session::SbirsVector3M& vector) {
  session::SbirsVector3M out;
  out.x = rotation.m00 * vector.x + rotation.m01 * vector.y + rotation.m02 * vector.z;
  out.y = rotation.m10 * vector.x + rotation.m11 * vector.y + rotation.m12 * vector.z;
  out.z = rotation.m20 * vector.x + rotation.m21 * vector.y + rotation.m22 * vector.z;
  return out;
}

}  // namespace

SbirsBoresightChain::SbirsBoresightChain(const session::SbirsEulerAnglesDeg& attitude_eci_body_deg,
                                         const oneq::foundation::EulerAnglesDeg& mount_angles_deg) {
  const oneq::coordinate::EulerAnglesDeg attitude(attitude_eci_body_deg.yaw_deg,
                                                  attitude_eci_body_deg.pitch_deg,
                                                  attitude_eci_body_deg.roll_deg);
  const oneq::coordinate::EulerAnglesDeg mount(mount_angles_deg.yaw_deg,
                                               mount_angles_deg.pitch_deg,
                                               mount_angles_deg.roll_deg);
  // 姿态（Body->ECI）与安装角（Body->Sensor）复合：R_sensor_to_eci = R_body_to_eci *
  // R_sensor_to_body。行主序约定：v_out = R · v。
  sensor_to_eci_ =
      oneq::coordinate::Compose(oneq::coordinate::BuildRotationMatrix(attitude),
                                oneq::coordinate::Inverse(oneq::coordinate::BuildRotationMatrix(mount)));
  eci_to_sensor_ = oneq::coordinate::Inverse(sensor_to_eci_);
  identity_ = attitude_eci_body_deg.yaw_deg == 0.0 && attitude_eci_body_deg.pitch_deg == 0.0 &&
              attitude_eci_body_deg.roll_deg == 0.0 && mount_angles_deg.yaw_deg == 0.0 &&
              mount_angles_deg.pitch_deg == 0.0 && mount_angles_deg.roll_deg == 0.0;
}

bool SbirsBoresightChain::IsIdentity() const { return identity_; }

session::SbirsVector3M SbirsBoresightChain::RotateSensorToEci(
    const session::SbirsVector3M& sensor_los) const {
  return Multiply(sensor_to_eci_, sensor_los);
}

session::SbirsVector3M SbirsBoresightChain::RotateEciToSensor(
    const session::SbirsVector3M& eci_vector) const {
  return Multiply(eci_to_sensor_, eci_vector);
}

session::SbirsVector3M SbirsBoresightChain::EciLosOfSensorPointing(float sensor_azimuth_deg,
                                                                   float sensor_elevation_deg) const {
  return RotateSensorToEci(LosFromAzimuthElevationDeg(sensor_azimuth_deg, sensor_elevation_deg));
}

void SbirsBoresightChain::SensorAzElOfEciVector(const session::SbirsVector3M& eci_los,
                                                float* azimuth_deg,
                                                float* elevation_deg) const {
  const session::SbirsVector3M sensor_los = RotateEciToSensor(eci_los);
  const double range = std::sqrt(sensor_los.x * sensor_los.x + sensor_los.y * sensor_los.y +
                                 sensor_los.z * sensor_los.z);
  const double horizontal = std::sqrt(sensor_los.x * sensor_los.x + sensor_los.y * sensor_los.y);
  if (azimuth_deg != nullptr) {
    *azimuth_deg = static_cast<float>(
        oneq::common::numerics::RadToDeg(std::atan2(sensor_los.y, sensor_los.x)));
  }
  if (elevation_deg != nullptr) {
    const double vertical = range > 0.0 ? sensor_los.z / range : 0.0;
    *elevation_deg = static_cast<float>(oneq::common::numerics::RadToDeg(
        std::asin(std::max(-1.0, std::min(1.0, vertical)))));
  }
}

void SbirsBoresightChain::SensorPointingForDesiredEciLos(const session::SbirsVector3M& desired_eci_los,
                                                         float* azimuth_deg,
                                                         float* elevation_deg) const {
  SensorAzElOfEciVector(desired_eci_los, azimuth_deg, elevation_deg);
}

void SbirsBoresightChain::ClampToScanLimits(const config::SbirsScanLimitsDeg& limits,
                                            float* azimuth_deg, float* elevation_deg) {
  if (azimuth_deg != nullptr) {
    *azimuth_deg = std::max(limits.az_min_deg, std::min(limits.az_max_deg, *azimuth_deg));
  }
  if (elevation_deg != nullptr) {
    *elevation_deg = std::max(limits.el_min_deg, std::min(limits.el_max_deg, *elevation_deg));
  }
}

}  // namespace pipeline
}  // namespace sbirs_sensor
