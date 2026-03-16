// Copyright 2026. All Rights Reserved.
//
// Description: 定义机载雷达方向配置的组合、限幅与校验工具函数。

#ifndef AIRBORNE_RADAR_COMMON_RADAR_ORIENTATION_UTILS_H_
#define AIRBORNE_RADAR_COMMON_RADAR_ORIENTATION_UTILS_H_

#include "1q/airborne_radar/common/RadarOrientationConfig.h"

namespace airborne_radar {
namespace common {

/// @brief 判断扫描限位是否合法。
/// @param limits 待校验的方位/俯仰限位。
/// @return 若方位和俯仰最小值均不大于最大值，则返回 true。
inline bool IsValidScanLimits(const AzimuthElevationLimitsDeg &limits) {
  return limits.az_min_deg <= limits.az_max_deg &&
         limits.el_min_deg <= limits.el_max_deg;
}

/// @brief 对浮点值执行闭区间限幅。
/// @param value 原始值。
/// @param min_value 下界。
/// @param max_value 上界。
/// @return 限幅后的结果。
inline float ClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

/// @brief 对方位/俯仰角执行扫描窗口限幅。
/// @param angle 待限幅角度。
/// @param limits 扫描窗口。
/// @return 限幅后的方位/俯仰角。
inline AzimuthElevationDeg ClampAzimuthElevation(
    const AzimuthElevationDeg &angle,
    const AzimuthElevationLimitsDeg &limits) {
  AzimuthElevationDeg clamped;
  clamped.az_deg =
      ClampFloat(angle.az_deg, limits.az_min_deg, limits.az_max_deg);
  clamped.el_deg =
      ClampFloat(angle.el_deg, limits.el_min_deg, limits.el_max_deg);
  return clamped;
}

/// @brief 计算机械扫描窗口与电子扫描窗口的交集。
/// @param mechanical_limits 机械扫描限位。
/// @param electronic_limits 电子扫描限位。
/// @return 两者交集；若无交集，则退化为零宽限位并由调用方继续处理。
inline AzimuthElevationLimitsDeg IntersectScanLimits(
    const AzimuthElevationLimitsDeg &mechanical_limits,
    const AzimuthElevationLimitsDeg &electronic_limits) {
  AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = mechanical_limits.az_min_deg >
                              electronic_limits.az_min_deg
                          ? mechanical_limits.az_min_deg
                          : electronic_limits.az_min_deg;
  limits.az_max_deg = mechanical_limits.az_max_deg <
                              electronic_limits.az_max_deg
                          ? mechanical_limits.az_max_deg
                          : electronic_limits.az_max_deg;
  limits.el_min_deg = mechanical_limits.el_min_deg >
                              electronic_limits.el_min_deg
                          ? mechanical_limits.el_min_deg
                          : electronic_limits.el_min_deg;
  limits.el_max_deg = mechanical_limits.el_max_deg <
                              electronic_limits.el_max_deg
                          ? mechanical_limits.el_max_deg
                          : electronic_limits.el_max_deg;

  if (limits.az_min_deg > limits.az_max_deg) {
    const float center =
        0.5f * (limits.az_min_deg + limits.az_max_deg);
    limits.az_min_deg = center;
    limits.az_max_deg = center;
  }
  if (limits.el_min_deg > limits.el_max_deg) {
    const float center =
        0.5f * (limits.el_min_deg + limits.el_max_deg);
    limits.el_min_deg = center;
    limits.el_max_deg = center;
  }
  return limits;
}

/// @brief 计算挂架坐标系下的实际波束指向，并按扫描窗口限幅。
/// @param config 雷达方向配置。
/// @return 相对雷达安装基准轴的方位/俯仰指向。
inline AzimuthElevationDeg ComputeMountFrameBeamPointing(
    const RadarOrientationConfig &config) {
  AzimuthElevationDeg unclamped;
  unclamped.az_deg =
      config.scan_center_deg.az_deg + config.dwell_center_deg.az_deg;
  unclamped.el_deg =
      config.scan_center_deg.el_deg + config.dwell_center_deg.el_deg;
  const AzimuthElevationLimitsDeg effective_limits = IntersectScanLimits(
      config.mechanical_scan_limits_deg, config.electronic_scan_limits_deg);
  return ClampAzimuthElevation(unclamped, effective_limits);
}

/// @brief 计算机体系下的实际波束指向。
/// @param config 雷达方向配置。
/// @return 机体系下的欧拉角；roll 继承安装滚转角。
inline EulerAnglesDeg ComputeBodyFrameBeamPointing(
    const RadarOrientationConfig &config) {
  const AzimuthElevationDeg mount_frame_pointing =
      ComputeMountFrameBeamPointing(config);
  EulerAnglesDeg body_frame_pointing;
  body_frame_pointing.yaw_deg =
      config.mount_angles_deg.yaw_deg + mount_frame_pointing.az_deg;
  body_frame_pointing.pitch_deg =
      config.mount_angles_deg.pitch_deg + mount_frame_pointing.el_deg;
  body_frame_pointing.roll_deg = config.mount_angles_deg.roll_deg;
  return body_frame_pointing;
}

/// @brief 计算平台姿态叠加后的波束指向。
/// @param platform_attitude_deg 平台姿态角。
/// @param config 雷达方向配置。
/// @return 平台姿态叠加后的欧拉角结果。
/// @note 该函数仅执行几何叠加，适用于机体稳定模式；
///       若采用惯性稳定或对地稳定，调用方应先求得等效平台姿态后再使用。
inline EulerAnglesDeg ComputePlatformFrameBeamPointing(
    const EulerAnglesDeg &platform_attitude_deg,
    const RadarOrientationConfig &config) {
  const EulerAnglesDeg body_frame_pointing =
      ComputeBodyFrameBeamPointing(config);
  EulerAnglesDeg platform_frame_pointing;
  platform_frame_pointing.yaw_deg =
      platform_attitude_deg.yaw_deg + body_frame_pointing.yaw_deg;
  platform_frame_pointing.pitch_deg =
      platform_attitude_deg.pitch_deg + body_frame_pointing.pitch_deg;
  platform_frame_pointing.roll_deg =
      platform_attitude_deg.roll_deg + body_frame_pointing.roll_deg;
  return platform_frame_pointing;
}

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_RADAR_ORIENTATION_UTILS_H_
