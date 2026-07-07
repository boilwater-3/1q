/**
 * @file CoordinateUtils.h
 * @brief 定义 coordinate 域与 foundation 域之间的轻量类型适配工具。
 */

#ifndef COMMON_COORDINATE_COORDINATE_UTILS_H_
#define COMMON_COORDINATE_COORDINATE_UTILS_H_

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/foundation/pose_types.h"

namespace oneq {
namespace common {
namespace coordinate_utils {

/**
 * @brief 将 coordinate 域的 EulerAnglesDeg 转换为 foundation 域的同名类型。
 * @param[in] attitude_deg coordinate 域欧拉角。
 * @return foundation 域欧拉角。
 */
inline oneq::foundation::EulerAnglesDeg ToFoundationEuler(
    const oneq::coordinate::EulerAnglesDeg& attitude_deg) {
  oneq::foundation::EulerAnglesDeg output;
  output.yaw_deg = attitude_deg.yaw_deg;
  output.pitch_deg = attitude_deg.pitch_deg;
  output.roll_deg = attitude_deg.roll_deg;
  return output;
}

/**
 * @brief 将 coordinate::Vector3d 转换为 foundation::Vector3f。
 * @param[in] v coordinate 域三维向量。
 * @return foundation 域三维向量。
 */
inline oneq::foundation::Vector3f ToFoundationVector(const oneq::coordinate::Vector3d& v) {
  oneq::foundation::Vector3f out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

/**
 * @brief 将 ENU 位置按局部姿态旋转到局部坐标系，并转换为 foundation::Vector3f。
 * @param[in] enu ENU 位置。
 * @param[in] local_attitude_deg 局部坐标系相对 ENU 的姿态角。
 * @return 局部坐标系下的位置向量（foundation 域）。
 */
inline oneq::foundation::Vector3f RotateEnuPositionToLocal(
    const oneq::coordinate::EnuPositionM& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToFoundationVector(
      oneq::coordinate::RotateEnuToLocal(enu.east_m, enu.north_m, enu.up_m, local_attitude_deg));
}

/**
 * @brief 将 ENU 速度按局部姿态旋转到局部坐标系，并转换为 foundation::Vector3f。
 * @param[in] enu ENU 速度。
 * @param[in] local_attitude_deg 局部坐标系相对 ENU 的姿态角。
 * @return 局部坐标系下的速度向量（foundation 域）。
 */
inline oneq::foundation::Vector3f RotateEnuVelocityToLocal(
    const oneq::coordinate::EnuVelocityMps& enu,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  return ToFoundationVector(oneq::coordinate::RotateEnuToLocal(enu.east_mps, enu.north_mps,
                                                               enu.up_mps, local_attitude_deg));
}

}  // namespace coordinate_utils
}  // namespace common
}  // namespace oneq

#endif  // COMMON_COORDINATE_COORDINATE_UTILS_H_
