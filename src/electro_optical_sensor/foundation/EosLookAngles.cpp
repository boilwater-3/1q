#include "electro_optical_sensor/foundation/EosLookAngles.h"

#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

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

  // ENU 旋入平台体系后取角：方位 = atan2(体系 y, 体系 x)，仰角出水平面为正。
  const oneq::coordinate::Vector3d body_vector =
      oneq::coordinate::RotateEnuToLocal(position_x, position_y, position_z,
                                         platform_attitude_deg);
  const double horizontal_norm =
      std::sqrt(body_vector.x * body_vector.x + body_vector.y * body_vector.y);
  const double kPi = static_cast<double>(constants::kPi);
  *range_m = static_cast<float>(range);
  *azimuth_deg = static_cast<float>(std::atan2(body_vector.y, body_vector.x) * 180.0 / kPi);
  *elevation_deg =
      static_cast<float>(std::atan2(body_vector.z, horizontal_norm) * 180.0 / kPi);
  return true;
}

}  // namespace foundation
}  // namespace electro_optical_sensor
