// Copyright 2026. All Rights Reserved.
//
// Description: 定义波束控制与方向图增益解析的私有工具。

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_

#include <cmath>

#include <Eigen/Core>

#include "1q/airborne_radar/common/AntennaPatternUtils.h"
#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/RadarOrientationUtils.h"
#include "1q/airborne_radar/signal/detection/BeamwidthResolution.h"
#include "1q/airborne_radar/signal/detection/DetectionTypes.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/// @brief ResolvedBeamState 表示当前探测使用的波束状态。
struct ResolvedBeamState {
  /// @brief 生效方位/俯仰波束宽度。
  EffectiveBeamwidthDeg effective_beamwidth_deg;

  /// @brief 挂架坐标系下的当前波束中心。
  common::AzimuthElevationDeg beam_pointing_deg;

  /// @brief 当前目标方向上的单程天线增益（dB）。
  float one_way_antenna_gain_db{0.0f};
};

/// @brief BeamControlResolver 负责组合波束宽度、指向与方向图增益。
class BeamControlResolver {
 public:
  /// @brief 解析当前探测使用的波束状态。
  /// @param antenna_config 天线配置。
  /// @param orientation_config 雷达方向与控制配置。
  /// @param target_look_angles 目标在雷达局部坐标系中的 look angle。
  /// @return 当前探测使用的波束状态。
  static ResolvedBeamState Resolve(
      const AntennaConfig& antenna_config,
      const common::RadarOrientationConfig& orientation_config,
      const common::PlatformAttitudeDeg& platform_attitude_deg,
      const TargetLookAnglesDeg& target_look_angles) {
    ResolvedBeamState state;
    state.effective_beamwidth_deg =
        ResolveEffectiveBeamwidth(antenna_config, orientation_config);
    state.beam_pointing_deg = ResolveMountFrameBeamPointing(
        orientation_config, platform_attitude_deg);
    state.one_way_antenna_gain_db = antenna_config.main_beam_gain_db;

    if (!antenna_config.enable_directional_pattern ||
        !target_look_angles.has_look_angles) {
      return state;
    }

    common::AntennaPatternBeamwidthDeg pattern_beamwidth;
    pattern_beamwidth.az_beamwidth_deg =
        state.effective_beamwidth_deg.az_beamwidth_deg;
    pattern_beamwidth.el_beamwidth_deg =
        state.effective_beamwidth_deg.el_beamwidth_deg;

    common::AntennaLookOffsetDeg offset_deg;
    offset_deg.delta_az_deg =
        target_look_angles.look_az_deg - state.beam_pointing_deg.az_deg;
    offset_deg.delta_el_deg =
        target_look_angles.look_el_deg - state.beam_pointing_deg.el_deg;

    const common::AntennaPatternSample sample =
        common::EvaluateAntennaPattern(
            antenna_config.main_beam_gain_db, antenna_config.pattern,
            pattern_beamwidth, offset_deg,
            orientation_config.scan_center_deg);
    state.one_way_antenna_gain_db = sample.gain_dbi;
    return state;
  }

 private:
  /// @brief 将角度从度转换为弧度。
  /// @param angle_deg 角度（度）。
  /// @return 弧度值。
  static float DegToRad(float angle_deg) {
    return angle_deg * 3.14159265358979f / 180.0f;
  }

  /// @brief 将角度从弧度转换为度。
  /// @param angle_rad 角度（弧度）。
  /// @return 度值。
  static float RadToDeg(float angle_rad) {
    return angle_rad * 180.0f / 3.14159265358979f;
  }

  /// @brief 将方位/俯仰角转换为单位方向向量。
  /// @param pointing_deg 方位/俯仰角（度）。
  /// @return 单位方向向量。
  static Eigen::Vector3f AzimuthElevationToUnitVector(
      const common::AzimuthElevationDeg& pointing_deg) {
    const float az_rad = DegToRad(pointing_deg.az_deg);
    const float el_rad = DegToRad(pointing_deg.el_deg);
    const float cos_el = std::cos(el_rad);
    return Eigen::Vector3f(cos_el * std::cos(az_rad),
                           cos_el * std::sin(az_rad),
                           std::sin(el_rad));
  }

  /// @brief 将单位方向向量转换为方位/俯仰角。
  /// @param unit_vector 单位方向向量。
  /// @return 方位/俯仰角（度）。
  static common::AzimuthElevationDeg UnitVectorToAzimuthElevation(
      const Eigen::Vector3f& unit_vector) {
    common::AzimuthElevationDeg pointing_deg;
    const float horizontal_norm = std::sqrt(
        unit_vector.x() * unit_vector.x() + unit_vector.y() * unit_vector.y());
    pointing_deg.az_deg = RadToDeg(std::atan2(unit_vector.y(), unit_vector.x()));
    pointing_deg.el_deg = RadToDeg(std::atan2(unit_vector.z(), horizontal_norm));
    return pointing_deg;
  }

  /// @brief 构建 Z-Y-X 顺序的欧拉角旋转矩阵。
  /// @param euler_deg 欧拉角（yaw/pitch/roll，单位：度）。
  /// @return 旋转矩阵。
  static Eigen::Matrix3f BuildRotationMatrix(
      const common::EulerAnglesDeg& euler_deg) {
    const float yaw_rad = DegToRad(euler_deg.yaw_deg);
    // 仓库内约定正 pitch 表示正仰角；在当前 z-up 右手系下，
    // 旋转矩阵需使用绕 y 轴的负角，才能与 az/el 语义保持一致。
    const float pitch_rad = DegToRad(-euler_deg.pitch_deg);
    const float roll_rad = DegToRad(euler_deg.roll_deg);

    const float cy = std::cos(yaw_rad);
    const float sy = std::sin(yaw_rad);
    const float cp = std::cos(pitch_rad);
    const float sp = std::sin(pitch_rad);
    const float cr = std::cos(roll_rad);
    const float sr = std::sin(roll_rad);

    Eigen::Matrix3f rotation;
    rotation << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr;
    return rotation;
  }

  /// @brief 将稳定参考系下的期望波束指向逆变换到挂架坐标系。
  /// @param desired_platform_pointing_deg 稳定参考系下的期望方位/俯仰角。
  /// @param platform_attitude_deg 当前平台姿态角。
  /// @param mount_angles_deg 雷达安装角。
  /// @return 挂架坐标系下的波束指向。
  static common::AzimuthElevationDeg ResolveStabilizedMountFramePointing(
      const common::AzimuthElevationDeg& desired_platform_pointing_deg,
      const common::PlatformAttitudeDeg& platform_attitude_deg,
      const common::EulerAnglesDeg& mount_angles_deg) {
    const Eigen::Vector3f platform_vector =
        AzimuthElevationToUnitVector(desired_platform_pointing_deg);
    const Eigen::Matrix3f platform_rotation =
        BuildRotationMatrix(platform_attitude_deg);
    const Eigen::Matrix3f mount_rotation =
        BuildRotationMatrix(mount_angles_deg);
    const Eigen::Vector3f body_vector =
        platform_rotation.transpose() * platform_vector;
    const Eigen::Vector3f mount_vector =
        mount_rotation.transpose() * body_vector;
    return UnitVectorToAzimuthElevation(mount_vector);
  }

  /// @brief 解析当前稳定模式下的挂架坐标系波束指向。
  /// @param orientation_config 雷达方向与控制配置。
  /// @param platform_attitude_deg 当前平台姿态角。
  /// @return 挂架坐标系下的波束指向。
  /// @note 对地稳定当前无地理参考输入，代码上显式等同于对惯性空间稳定。
  static common::AzimuthElevationDeg ResolveMountFrameBeamPointing(
      const common::RadarOrientationConfig& orientation_config,
      const common::PlatformAttitudeDeg& platform_attitude_deg) {
    const common::AzimuthElevationLimitsDeg effective_limits =
        common::IntersectScanLimits(
            orientation_config.mechanical_scan_limits_deg,
            orientation_config.electronic_scan_limits_deg);
    if (orientation_config.stabilization_mode ==
        common::StabilizationMode::kBodyStabilized) {
      return common::ComputeMountFrameBeamPointing(orientation_config);
    }

    common::AzimuthElevationDeg desired_platform_pointing_deg;
    desired_platform_pointing_deg.az_deg =
        orientation_config.scan_center_deg.az_deg +
        orientation_config.dwell_center_deg.az_deg;
    desired_platform_pointing_deg.el_deg =
        orientation_config.scan_center_deg.el_deg +
        orientation_config.dwell_center_deg.el_deg;

    const common::AzimuthElevationDeg stabilized_mount_frame_pointing =
        ResolveStabilizedMountFramePointing(desired_platform_pointing_deg,
                                            platform_attitude_deg,
                                            orientation_config.mount_angles_deg);
    return common::ClampAzimuthElevation(
        stabilized_mount_frame_pointing, effective_limits);
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_
