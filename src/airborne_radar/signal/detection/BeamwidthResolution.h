/**
 * @file BeamwidthResolution.h
 * @brief 定义雷达名义波束宽度与指令态波束宽度的统一解析规则。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_BEAMWIDTH_RESOLUTION_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_BEAMWIDTH_RESOLUTION_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/config/SignalDetectionConfig.h"

namespace airborne_radar {
namespace signal {
namespace detection {
/**
 * @brief EffectiveBeamwidthDeg 表示解析后的有效波束宽度（单位：度）。
 */
struct EffectiveBeamwidthDeg {
  float az_beamwidth_deg{0.0f}; /**< 有效方位波束宽度（单位：度）。 */
  float el_beamwidth_deg{0.0f}; /**< 有效俯仰波束宽度（单位：度）。 */
};
/**
 * @brief 解析有效波束宽度。
 * @param antenna_config 雷达体制名义天线配置。
 * @param orientation_config 雷达方向与控制配置。
 * @return 若启用指令态覆盖，则返回 commanded_*；
 *         否则回退到 nominal_*。
 * @note 该返回值是“探测/控制共享的统一入口”。
 *       SignalDetector 会基于解析后的有效波束宽度计算等效角度测量标准差，
 *       后续由 SignalPipeline 将其传播到量测协方差建模。
 */
inline EffectiveBeamwidthDeg ResolveEffectiveBeamwidth(
    const config::AntennaConfig& antenna_config,
    const model::RadarOrientationConfig& orientation_config) {
  EffectiveBeamwidthDeg beamwidth;
  if (orientation_config.commanded_beamwidth_enabled) {
    beamwidth.az_beamwidth_deg =
        orientation_config.commanded_beamwidth_deg.commanded_az_beamwidth_deg;
    beamwidth.el_beamwidth_deg =
        orientation_config.commanded_beamwidth_deg.commanded_el_beamwidth_deg;
    return beamwidth;
  }

  beamwidth.az_beamwidth_deg = antenna_config.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = antenna_config.nominal_el_beamwidth_deg;
  return beamwidth;
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_BEAMWIDTH_RESOLUTION_H_
