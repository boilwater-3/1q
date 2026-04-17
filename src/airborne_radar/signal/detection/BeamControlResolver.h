/**
 * @file BeamControlResolver.h
 * @brief 定义波束控制与方向图增益解析的私有工具。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_

#include "airborne_radar/config/engineering/SignalEngineeringConfig.h"
#include "airborne_radar/utils/RadarOrientationUtils.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/config/SignalDetectionConfig.h"
#include "airborne_radar/signal/detection/AntennaPatternRuntime.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"
#include "common/geometry/GeometryTransform.h"

namespace airborne_radar {
namespace signal {
namespace detection {
/**
 * @brief ResolvedBeamState 表示当前探测使用的波束状态。
 */
struct ResolvedBeamState {
  EffectiveBeamwidthDeg effective_beamwidth_deg;         /**< 生效方位/俯仰波束宽度。 */
  model::AzimuthElevationDeg beam_pointing_deg; /**< 挂架坐标系下的当前波束中心。 */
  float one_way_antenna_gain_db{0.0f}; /**< 当前目标方向上的单程天线增益（dB）。 */
};
/**
 * @brief BeamControlResolver 负责组合波束宽度、指向与方向图增益。
 */
class BeamControlResolver {
 public:
  /**
   * @brief 解析当前探测使用的波束状态。
   * @param antenna_config 天线配置。
   * @param orientation_config 雷达方向与控制配置。
   * @param target_look_angles 目标在雷达局部坐标系中的 look angle。
   * @return 当前探测使用的波束状态。
   */
  static ResolvedBeamState Resolve(const config::engineering::AntennaConfig& antenna_config,
                                   const model::RadarOrientationConfig& orientation_config,
                                   const model::PlatformAttitudeDeg& platform_attitude_deg,
                                   const TargetLookAnglesDeg& target_look_angles) {
    ResolvedBeamState state;
    state.effective_beamwidth_deg = ResolveEffectiveBeamwidth(antenna_config, orientation_config);
    state.beam_pointing_deg =
        ResolveMountFrameBeamPointing(orientation_config, platform_attitude_deg);
    state.one_way_antenna_gain_db = antenna_config.main_beam_gain_db;

    if (!antenna_config.enable_directional_pattern || !target_look_angles.has_look_angles) {
      return state;
    }

    AntennaPatternBeamwidthDeg pattern_beamwidth;
    pattern_beamwidth.az_beamwidth_deg = state.effective_beamwidth_deg.az_beamwidth_deg;
    pattern_beamwidth.el_beamwidth_deg = state.effective_beamwidth_deg.el_beamwidth_deg;

    AntennaLookOffsetDeg offset_deg;
    offset_deg.delta_az_deg = target_look_angles.look_az_deg - state.beam_pointing_deg.az_deg;
    offset_deg.delta_el_deg = target_look_angles.look_el_deg - state.beam_pointing_deg.el_deg;

    const AntennaPatternSample sample =
        EvaluateAntennaPattern(antenna_config.main_beam_gain_db, antenna_config.pattern,
                               pattern_beamwidth, offset_deg, state.beam_pointing_deg);
    state.one_way_antenna_gain_db = sample.gain_dbi;
    return state;
  }

 private:
  /**
   * @brief 解析当前稳定模式下的挂架坐标系波束指向。
   * @param orientation_config 雷达方向与控制配置。
   * @param platform_attitude_deg 当前平台姿态角。
   * @return 挂架坐标系下的波束指向。
   * @note 对地稳定当前无地理参考输入，代码上显式等同于对惯性空间稳定。
   */
  static model::AzimuthElevationDeg ResolveMountFrameBeamPointing(
      const model::RadarOrientationConfig& orientation_config,
      const model::PlatformAttitudeDeg& platform_attitude_deg) {
    const model::AzimuthElevationLimitsDeg effective_limits =
        utils::IntersectScanLimits(orientation_config.mechanical_scan_limits_deg,
                                           orientation_config.electronic_scan_limits_deg);
    if (orientation_config.stabilization_mode ==
        model::StabilizationMode::kBodyStabilized) {
      return utils::ComputeMountFrameBeamPointing(orientation_config);
    }

    model::AzimuthElevationDeg desired_platform_pointing_deg;
    desired_platform_pointing_deg.az_deg =
        orientation_config.scan_center_deg.az_deg + orientation_config.dwell_center_deg.az_deg;
    desired_platform_pointing_deg.el_deg =
        orientation_config.scan_center_deg.el_deg + orientation_config.dwell_center_deg.el_deg;

    oneq::internal::geometry::AzimuthElevationDeg desired_platform_pointing;
    desired_platform_pointing.az_deg = desired_platform_pointing_deg.az_deg;
    desired_platform_pointing.el_deg = desired_platform_pointing_deg.el_deg;
    oneq::internal::geometry::EulerAnglesDeg platform_attitude;
    platform_attitude.yaw_deg = platform_attitude_deg.yaw_deg;
    platform_attitude.pitch_deg = platform_attitude_deg.pitch_deg;
    platform_attitude.roll_deg = platform_attitude_deg.roll_deg;
    oneq::internal::geometry::EulerAnglesDeg mount_angles;
    mount_angles.yaw_deg = orientation_config.mount_angles_deg.yaw_deg;
    mount_angles.pitch_deg = orientation_config.mount_angles_deg.pitch_deg;
    mount_angles.roll_deg = orientation_config.mount_angles_deg.roll_deg;
    const oneq::internal::geometry::AzimuthElevationDeg stabilized_mount_pointing =
        oneq::internal::geometry::ResolveStabilizedMountFramePointing(
            desired_platform_pointing, platform_attitude, mount_angles);
    model::AzimuthElevationDeg stabilized_mount_frame_pointing;
    stabilized_mount_frame_pointing.az_deg = stabilized_mount_pointing.az_deg;
    stabilized_mount_frame_pointing.el_deg = stabilized_mount_pointing.el_deg;
    return utils::ClampAzimuthElevation(stabilized_mount_frame_pointing, effective_limits);
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_
