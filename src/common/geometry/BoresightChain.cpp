#include "common/geometry/BoresightChain.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/Constants.h"

namespace oneq {
namespace common {
namespace geometry {

namespace {

oneq::coordinate::Vector3d LosFromAzimuthElevationDeg(float azimuth_deg, float elevation_deg) {
  const double azimuth_rad = oneq::common::numerics::DegToRad(static_cast<double>(azimuth_deg));
  const double elevation_rad = oneq::common::numerics::DegToRad(static_cast<double>(elevation_deg));
  const double horizontal = std::cos(elevation_rad);
  return oneq::coordinate::Vector3d(horizontal * std::cos(azimuth_rad),
                                    horizontal * std::sin(azimuth_rad), std::sin(elevation_rad));
}

oneq::coordinate::Vector3d Multiply(const oneq::coordinate::RotationMatrix3d& rotation,
                                    const oneq::coordinate::Vector3d& vector) {
  return oneq::coordinate::Vector3d(
      rotation.m00 * vector.x + rotation.m01 * vector.y + rotation.m02 * vector.z,
      rotation.m10 * vector.x + rotation.m11 * vector.y + rotation.m12 * vector.z,
      rotation.m20 * vector.x + rotation.m21 * vector.y + rotation.m22 * vector.z);
}

}  // namespace

BoresightChain::BoresightChain(const EulerAnglesDeg& attitude_reference_body_deg,
                               const EulerAnglesDeg& mount_body_sensor_deg)
    : BoresightChain(attitude_reference_body_deg, mount_body_sensor_deg, EulerAnglesDeg()) {}

BoresightChain::BoresightChain(const EulerAnglesDeg& attitude_reference_body_deg,
                               const EulerAnglesDeg& mount_body_sensor_deg,
                               const EulerAnglesDeg& misalignment_deg) {
  // 姿态（Body->Reference）与安装角（Body->Sensor）及安装失准复合：
  // R_sensor_to_reference = R_reference_body · R_body_sensor⁻¹ · R_sensor_misalign⁻¹
  // （失准作用于传感器系内，等效安装偏置微扰）。行主序约定：v_out = R · v。
  const oneq::coordinate::EulerAnglesDeg attitude(attitude_reference_body_deg.yaw_deg,
                                                  attitude_reference_body_deg.pitch_deg,
                                                  attitude_reference_body_deg.roll_deg);
  const oneq::coordinate::EulerAnglesDeg mount(mount_body_sensor_deg.yaw_deg,
                                               mount_body_sensor_deg.pitch_deg,
                                               mount_body_sensor_deg.roll_deg);
  const oneq::coordinate::EulerAnglesDeg misalignment(misalignment_deg.yaw_deg,
                                                      misalignment_deg.pitch_deg,
                                                      misalignment_deg.roll_deg);
  sensor_to_reference_ = oneq::coordinate::Compose(
      oneq::coordinate::Compose(oneq::coordinate::BuildRotationMatrix(attitude),
                                oneq::coordinate::Inverse(
                                    oneq::coordinate::BuildRotationMatrix(mount))),
      oneq::coordinate::Inverse(oneq::coordinate::BuildRotationMatrix(misalignment)));
  reference_to_sensor_ = oneq::coordinate::Inverse(sensor_to_reference_);
  identity_ = attitude_reference_body_deg.yaw_deg == 0.0 &&
              attitude_reference_body_deg.pitch_deg == 0.0 &&
              attitude_reference_body_deg.roll_deg == 0.0 &&
              mount_body_sensor_deg.yaw_deg == 0.0 && mount_body_sensor_deg.pitch_deg == 0.0 &&
              mount_body_sensor_deg.roll_deg == 0.0 && misalignment_deg.yaw_deg == 0.0 &&
              misalignment_deg.pitch_deg == 0.0 && misalignment_deg.roll_deg == 0.0;
}

bool BoresightChain::IsIdentity() const { return identity_; }

oneq::coordinate::Vector3d BoresightChain::RotateSensorToReference(
    const oneq::coordinate::Vector3d& sensor_vector) const {
  return Multiply(sensor_to_reference_, sensor_vector);
}

oneq::coordinate::Vector3d BoresightChain::RotateReferenceToSensor(
    const oneq::coordinate::Vector3d& reference_vector) const {
  return Multiply(reference_to_sensor_, reference_vector);
}

oneq::coordinate::Vector3d BoresightChain::ReferenceLosOfSensorPointing(
    float sensor_azimuth_deg, float sensor_elevation_deg) const {
  return RotateSensorToReference(
      LosFromAzimuthElevationDeg(sensor_azimuth_deg, sensor_elevation_deg));
}

void BoresightChain::SensorAzElOfReferenceVector(
    const oneq::coordinate::Vector3d& reference_los, float* azimuth_deg,
    float* elevation_deg) const {
  const oneq::coordinate::Vector3d sensor_los = RotateReferenceToSensor(reference_los);
  const double range = std::sqrt(sensor_los.x * sensor_los.x + sensor_los.y * sensor_los.y +
                                 sensor_los.z * sensor_los.z);
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

void BoresightChain::SensorPointingForDesiredReferenceLos(
    const oneq::coordinate::Vector3d& desired_reference_los, float* azimuth_deg,
    float* elevation_deg) const {
  SensorAzElOfReferenceVector(desired_reference_los, azimuth_deg, elevation_deg);
}

void BoresightChain::ClampToScanLimits(const AzimuthElevationLimitsDeg& limits,
                                       float* azimuth_deg, float* elevation_deg) {
  if (azimuth_deg != nullptr) {
    *azimuth_deg = std::max(limits.az_min_deg, std::min(limits.az_max_deg, *azimuth_deg));
  }
  if (elevation_deg != nullptr) {
    *elevation_deg = std::max(limits.el_min_deg, std::min(limits.el_max_deg, *elevation_deg));
  }
}

}  // namespace geometry
}  // namespace common
}  // namespace oneq
