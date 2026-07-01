/**
 * @file ArOrientationUtils.h
 * @brief 定义机载雷达方向配置的组合、限幅与校验工具函数。
 */

#ifndef AIRBORNE_RADAR_UTILS_AR_ORIENTATION_UTILS_H_
#define AIRBORNE_RADAR_UTILS_AR_ORIENTATION_UTILS_H_

#include <algorithm>
#include <cmath>

#include "airborne_radar/utils/MathUtils.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "common/numerics/Constants.h"

namespace airborne_radar {
namespace utils {




struct Matrix3f {
  float m[3][3]{};
};

inline float At(const Matrix3f& matrix, int row, int col) { return matrix.m[row][col]; }

inline Matrix3f Multiply(const Matrix3f& lhs, const Matrix3f& rhs) {
  Matrix3f result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result.m[row][col] = 0.0f;
      for (int k = 0; k < 3; ++k) {
        result.m[row][col] += lhs.m[row][k] * rhs.m[k][col];
      }
    }
  }
  return result;
}

inline Matrix3f BuildRotationMatrix(const config::EulerAnglesDeg& euler_deg) {
  const float yaw_rad = static_cast<float>(oneq::internal::numerics::DegToRad(euler_deg.yaw_deg));
  // Keep the same pitch sign convention as the internal geometry module.
  const float pitch_rad = static_cast<float>(oneq::internal::numerics::DegToRad(-euler_deg.pitch_deg));
  const float roll_rad = static_cast<float>(oneq::internal::numerics::DegToRad(euler_deg.roll_deg));

  const float cy = std::cos(yaw_rad);
  const float sy = std::sin(yaw_rad);
  const float cp = std::cos(pitch_rad);
  const float sp = std::sin(pitch_rad);
  const float cr = std::cos(roll_rad);
  const float sr = std::sin(roll_rad);

  Matrix3f rotation;
  rotation.m[0][0] = cy * cp;
  rotation.m[0][1] = cy * sp * sr - sy * cr;
  rotation.m[0][2] = cy * sp * cr + sy * sr;
  rotation.m[1][0] = sy * cp;
  rotation.m[1][1] = sy * sp * sr + cy * cr;
  rotation.m[1][2] = sy * sp * cr - cy * sr;
  rotation.m[2][0] = -sp;
  rotation.m[2][1] = cp * sr;
  rotation.m[2][2] = cp * cr;
  return rotation;
}

inline config::EulerAnglesDeg FromRotationMatrix(const Matrix3f& rotation) {
  config::EulerAnglesDeg euler_deg;
  const float r20 = std::max(-1.0f, std::min(1.0f, At(rotation, 2, 0)));
  euler_deg.pitch_deg = std::asin(r20) * 180.0f / 3.14159265358979f;
  euler_deg.yaw_deg =
      std::atan2(At(rotation, 1, 0), At(rotation, 0, 0)) * 180.0f / 3.14159265358979f;
  euler_deg.roll_deg =
      std::atan2(At(rotation, 2, 1), At(rotation, 2, 2)) * 180.0f / 3.14159265358979f;
  return euler_deg;
}


/**
 * @brief 判断扫描限位是否合法。
 * @param[in] limits 待校验的方位/俯仰限位。
 * @return 若方位和俯仰最小值均不大于最大值，则返回 true。
 */
inline bool IsValidScanLimits(const config::AzimuthElevationLimitsDeg& limits) {
  return limits.az_min_deg <= limits.az_max_deg && limits.el_min_deg <= limits.el_max_deg;
}

/**
 * @brief 对方位/俯仰角执行扫描窗口限幅。
 * @param[in] angle 待限幅角度。
 * @param[in] limits 扫描窗口。
 * @return 限幅后的方位/俯仰角。
 */
inline config::AzimuthElevationDeg ClampAzimuthElevation(
    const config::AzimuthElevationDeg& angle, const config::AzimuthElevationLimitsDeg& limits) {
  config::AzimuthElevationDeg clamped;
  clamped.az_deg = ClampFloat(angle.az_deg, limits.az_min_deg, limits.az_max_deg);
  clamped.el_deg = ClampFloat(angle.el_deg, limits.el_min_deg, limits.el_max_deg);
  return clamped;
}

/**
 * @brief 计算机械扫描窗口与电子扫描窗口的交集。
 * @param[in] mechanical_limits 机械扫描限位。
 * @param[in] electronic_limits 电子扫描限位。
 * @return 两者交集；若无交集，则退化为零宽限位并由调用方继续处理。
 */
inline config::AzimuthElevationLimitsDeg IntersectScanLimits(
    const config::AzimuthElevationLimitsDeg& mechanical_limits,
    const config::AzimuthElevationLimitsDeg& electronic_limits) {
  config::AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = mechanical_limits.az_min_deg > electronic_limits.az_min_deg
                          ? mechanical_limits.az_min_deg
                          : electronic_limits.az_min_deg;
  limits.az_max_deg = mechanical_limits.az_max_deg < electronic_limits.az_max_deg
                          ? mechanical_limits.az_max_deg
                          : electronic_limits.az_max_deg;
  limits.el_min_deg = mechanical_limits.el_min_deg > electronic_limits.el_min_deg
                          ? mechanical_limits.el_min_deg
                          : electronic_limits.el_min_deg;
  limits.el_max_deg = mechanical_limits.el_max_deg < electronic_limits.el_max_deg
                          ? mechanical_limits.el_max_deg
                          : electronic_limits.el_max_deg;

  if (limits.az_min_deg > limits.az_max_deg) {
    const float center = 0.5f * (limits.az_min_deg + limits.az_max_deg);
    limits.az_min_deg = center;
    limits.az_max_deg = center;
  }
  if (limits.el_min_deg > limits.el_max_deg) {
    const float center = 0.5f * (limits.el_min_deg + limits.el_max_deg);
    limits.el_min_deg = center;
    limits.el_max_deg = center;
  }
  return limits;
}

/**
 * @brief 计算挂架坐标系下的实际波束指向，并按扫描窗口限幅。
 * @param[in] config 雷达方向配置。
 * @param[in] dwell_center_deg 运行期驻留偏移；该偏移不属于 ArOrientationConfig 的静态字段。
 * @return 相对雷达安装基准轴的方位/俯仰指向。
 * @note 当 dwell_center_deg 为零时，该函数对应静态基准关系；
 *       非零时表示在静态基准上叠加运行期偏移。
 */
inline config::AzimuthElevationDeg ComputeMountFrameBeamPointing(
    const config::ArOrientationConfig& config,
    const config::AzimuthElevationDeg& dwell_center_deg) {
  config::AzimuthElevationDeg unclamped;
  unclamped.az_deg = config.scan_center_deg.az_deg + dwell_center_deg.az_deg;
  unclamped.el_deg = config.scan_center_deg.el_deg + dwell_center_deg.el_deg;
  const config::AzimuthElevationLimitsDeg effective_limits =
      IntersectScanLimits(config.mechanical_scan_limits_deg, config.electronic_scan_limits_deg);
  return ClampAzimuthElevation(unclamped, effective_limits);
}

/**
 * @brief 计算未显式给出驻留偏移时的挂架坐标系波束指向。
 * @param[in] config 雷达方向配置。
 * @return 相对雷达安装基准轴的方位/俯仰指向。
 */
inline config::AzimuthElevationDeg ComputeMountFrameBeamPointing(
    const config::ArOrientationConfig& config) {
  return ComputeMountFrameBeamPointing(config, config::AzimuthElevationDeg());
}

/**
 * @brief 计算机体系下的实际波束指向。
 * @param[in] config 雷达方向配置。
 * @param[in] dwell_center_deg 运行期驻留偏移。
 * @return 机体系下的欧拉角；由安装姿态与挂架波束指向做旋转合成得到。
 */
inline config::EulerAnglesDeg ComputeBodyFrameBeamPointing(
    const config::ArOrientationConfig& config,
    const config::AzimuthElevationDeg& dwell_center_deg) {
  const config::AzimuthElevationDeg mount_frame_pointing =
      ComputeMountFrameBeamPointing(config, dwell_center_deg);
  config::EulerAnglesDeg mount_frame_euler;
  mount_frame_euler.yaw_deg = mount_frame_pointing.az_deg;
  mount_frame_euler.pitch_deg = mount_frame_pointing.el_deg;
  const Matrix3f body_rotation = Multiply(
      BuildRotationMatrix(config.mount_angles_deg),
      BuildRotationMatrix(mount_frame_euler));
  return FromRotationMatrix(body_rotation);
}

/**
 * @brief 计算未显式给出驻留偏移时的机体系波束指向。
 * @param[in] config 雷达方向配置。
 * @return 机体系下的欧拉角；由安装姿态与挂架波束指向做旋转合成得到。
 */
inline config::EulerAnglesDeg ComputeBodyFrameBeamPointing(
    const config::ArOrientationConfig& config) {
  return ComputeBodyFrameBeamPointing(config, config::AzimuthElevationDeg());
}

/**
 * @brief 计算平台姿态叠加后的波束指向。
 * @param[in] platform_attitude_deg 平台姿态角。
 * @param[in] config 雷达方向配置。
 * @param[in] dwell_center_deg 运行期驻留偏移。
 * @return 平台姿态叠加后的欧拉角结果。
 * @note 该函数仅执行几何叠加，适用于机体稳定模式；
 *       若采用惯性稳定或对地稳定，调用方应先求得等效平台姿态后再使用。
 */
inline config::EulerAnglesDeg ComputePlatformFrameBeamPointing(
    const config::EulerAnglesDeg& platform_attitude_deg,
    const config::ArOrientationConfig& config,
    const config::AzimuthElevationDeg& dwell_center_deg) {
  const config::AzimuthElevationDeg mount_frame_pointing =
      ComputeMountFrameBeamPointing(config, dwell_center_deg);
  config::EulerAnglesDeg mount_frame_euler;
  mount_frame_euler.yaw_deg = mount_frame_pointing.az_deg;
  mount_frame_euler.pitch_deg = mount_frame_pointing.el_deg;
  const Matrix3f platform_mount_rotation = Multiply(
      BuildRotationMatrix(platform_attitude_deg),
      BuildRotationMatrix(config.mount_angles_deg));
  const Matrix3f platform_rotation = Multiply(
      platform_mount_rotation, BuildRotationMatrix(mount_frame_euler));
  return FromRotationMatrix(platform_rotation);
}

/**
 * @brief 计算未显式给出驻留偏移时的平台姿态叠加波束指向。
 * @param[in] platform_attitude_deg 平台姿态角。
 * @param[in] config 雷达方向配置。
 * @return 平台姿态叠加后的欧拉角结果。
 */
inline config::EulerAnglesDeg ComputePlatformFrameBeamPointing(
    const config::EulerAnglesDeg& platform_attitude_deg,
    const config::ArOrientationConfig& config) {
  return ComputePlatformFrameBeamPointing(platform_attitude_deg, config,
                                          config::AzimuthElevationDeg());
}

}  // namespace utils
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_UTILS_AR_ORIENTATION_UTILS_H_
