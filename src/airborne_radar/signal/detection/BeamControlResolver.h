// Copyright 2026. All Rights Reserved.
//
// Description: 定义波束控制与方向图增益解析的私有工具。

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_

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
      const TargetLookAnglesDeg& target_look_angles) {
    ResolvedBeamState state;
    state.effective_beamwidth_deg =
        ResolveEffectiveBeamwidth(antenna_config, orientation_config);
    state.beam_pointing_deg =
        common::ComputeMountFrameBeamPointing(orientation_config);
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
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_BEAM_CONTROL_RESOLVER_H_
