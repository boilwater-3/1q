#include "electro_optical_sensor/foundation/EosLookAngles.h"

#include <cmath>

#include "common/geometry/BoresightChain.h"

namespace electro_optical_sensor {
namespace foundation {

namespace {

constexpr float kNormFloor = 1.0e-6f;

}  // namespace

float EosLookAngleNormFloorM() { return kNormFloor; }

bool TryResolveEosLookAngles(double position_x, double position_y, double position_z,
                             const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
                             float* range_m, float* azimuth_deg, float* elevation_deg) {
  if (range_m == nullptr || azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }

  const double range =
      std::sqrt(position_x * position_x + position_y * position_y + position_z * position_z);
  if (range <= static_cast<double>(kNormFloor)) {
    return false;
  }

  // ENU 旋入体系取角委托公共安装链（common/geometry/BoresightChain 薄使用）：EOS 无
  // 安装角/失准配置，链路仅含平台姿态（Body->ENU），方位/仰角语义与直接 atan2 提取
  // 一致；退化判定（模长下限）与斜距输出留在本层（模块策略）。
  const oneq::common::geometry::BoresightChain chain(
      oneq::coordinate::EulerAnglesDeg(platform_attitude_deg.yaw_deg,
                                       platform_attitude_deg.pitch_deg,
                                       platform_attitude_deg.roll_deg),
      oneq::coordinate::EulerAnglesDeg());
  const oneq::coordinate::Vector3d enu_position(position_x, position_y, position_z);
  *range_m = static_cast<float>(range);
  chain.SensorAzElOfReferenceVector(enu_position, azimuth_deg, elevation_deg);
  return true;
}

}  // namespace foundation
}  // namespace electro_optical_sensor
