/**
 * @file BeamwidthResolution.h
 * @brief AR 波束宽度解析薄适配层（common 单源）。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_BEAMWIDTH_RESOLUTION_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_BEAMWIDTH_RESOLUTION_H_

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "common/radar/BeamwidthResolution.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief EffectiveBeamwidthDeg 表示解析后的有效波束宽度（单位：度）。
 */
using EffectiveBeamwidthDeg = ::oneq::common::radar::EffectiveBeamwidthDeg;

/**
 * @brief 从物理孔径尺寸推导半功率波束宽度（单位：rad）。
 * @param[in] aperture_length_m 孔径尺寸（单位：m）。
 * @param[in] wavelength_m 载波波长（单位：m）。
 * @return 半功率波束宽度（单位：rad），输入无效时返回 0。
 * @note 均匀孔径近似：θ_bw ≈ λ / L。
 */
inline float DeriveBeamwidthFromApertureRad(float aperture_length_m, float wavelength_m) {
  return ::oneq::common::radar::DeriveBeamwidthFromApertureRad(aperture_length_m, wavelength_m);
}

/**
 * @brief 解析有效波束宽度。
 * @param antenna_config 雷达体制名义天线配置。
 * @param orientation_config 雷达方向与控制配置。
 * @param wavelength_m 载波波长（单位：m），用于物理尺寸推导波束宽度（0=不使用物理推导）。
 * @return 若启用指令态覆盖，则返回 commanded_*；
 *         否则回退到 nominal_*；
 *         若 nominal_* 为 0 且 antenna_*_m>0，则从 λ/L 物理推导。
 * @note 该返回值是”探测/控制共享的统一入口”。
 *       SignalDetector 会基于解析后的有效波束宽度计算等效角度测量标准差，
 *       后续由 SignalPipeline 将其传播到量测协方差建模。
 */
inline EffectiveBeamwidthDeg ResolveEffectiveBeamwidth(
    const config::engineering::AntennaConfig& antenna_config,
    const config::ArOrientationConfig& orientation_config,
    float wavelength_m = 0.0f) {
  ::oneq::common::radar::CommandedBeamwidthOverride override;
  if (orientation_config.commanded_beamwidth_enabled) {
    override.enabled = true;
    override.az_beamwidth_deg =
        orientation_config.commanded_beamwidth_deg.commanded_az_beamwidth_deg;
    override.el_beamwidth_deg =
        orientation_config.commanded_beamwidth_deg.commanded_el_beamwidth_deg;
  }
  return ::oneq::common::radar::ResolveEffectiveBeamwidth(
      antenna_config.nominal_az_beamwidth_deg, antenna_config.nominal_el_beamwidth_deg,
      antenna_config.antenna_length_m, antenna_config.antenna_width_m, wavelength_m, override);
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_BEAMWIDTH_RESOLUTION_H_
