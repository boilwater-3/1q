/**
 * @file BeamwidthResolution.h
 * @brief 定义名义波束宽度、指令态覆盖与孔径物理推导的统一解析规则（common 单源）。
 */

#ifndef COMMON_RADAR_BEAMWIDTH_RESOLUTION_H_
#define COMMON_RADAR_BEAMWIDTH_RESOLUTION_H_

#include "common/numerics/Constants.h"

namespace oneq {
namespace common {
namespace radar {

using oneq::common::numerics::RadToDeg;

/**
 * @brief EffectiveBeamwidthDeg 表示解析后的有效波束宽度（单位：度）。
 */
struct EffectiveBeamwidthDeg {
  float az_beamwidth_deg{0.0f}; /**< 有效方位波束宽度（单位：deg） */
  float el_beamwidth_deg{0.0f}; /**< 有效俯仰波束宽度（单位：deg） */
};

/**
 * @brief CommandedBeamwidthOverride 描述模块注入的指令态波束宽度覆盖。
 */
struct CommandedBeamwidthOverride {
  bool enabled{false};
  float az_beamwidth_deg{0.0f};
  float el_beamwidth_deg{0.0f};
};

/**
 * @brief 从物理孔径尺寸推导半功率波束宽度（单位：rad）。
 * @param[in] aperture_length_m 孔径尺寸（单位：m）。
 * @param[in] wavelength_m 载波波长（单位：m）。
 * @return 半功率波束宽度（单位：rad），输入无效时返回 0。
 * @note 均匀孔径近似：θ_bw ≈ λ / L。
 */
inline float DeriveBeamwidthFromApertureRad(float aperture_length_m, float wavelength_m) {
  if (aperture_length_m <= 0.0f || wavelength_m <= 0.0f) {
    return 0.0f;
  }
  return wavelength_m / aperture_length_m;
}

/**
 * @brief 解析有效波束宽度。
 * @param[in] nominal_az_deg 名义方位波束宽度（单位：deg）。
 * @param[in] nominal_el_deg 名义俯仰波束宽度（单位：deg）。
 * @param[in] aperture_az_m 方位孔径尺寸（单位：m），用于 λ/L 物理推导。
 * @param[in] aperture_el_m 俯仰孔径尺寸（单位：m），用于 λ/L 物理推导。
 * @param[in] wavelength_m 载波波长（单位：m）；0 表示不使用物理推导。
 * @param[in] override 指令态波束宽度覆盖；enabled 时直接返回覆盖值。
 * @return 若启用指令态覆盖，则返回覆盖值；否则 nominal 为正时直接生效，
 *         nominal 为 0 且孔径/波长有效时从 λ/L 物理推导。
 */
inline EffectiveBeamwidthDeg ResolveEffectiveBeamwidth(
    float nominal_az_deg, float nominal_el_deg, float aperture_az_m, float aperture_el_m,
    float wavelength_m, const CommandedBeamwidthOverride& override = {}) {
  EffectiveBeamwidthDeg beamwidth;
  if (override.enabled) {
    beamwidth.az_beamwidth_deg = override.az_beamwidth_deg;
    beamwidth.el_beamwidth_deg = override.el_beamwidth_deg;
    return beamwidth;
  }

  beamwidth.az_beamwidth_deg = nominal_az_deg;
  beamwidth.el_beamwidth_deg = nominal_el_deg;

  if (beamwidth.az_beamwidth_deg <= 0.0f && aperture_az_m > 0.0f && wavelength_m > 0.0f) {
    beamwidth.az_beamwidth_deg =
        RadToDeg(DeriveBeamwidthFromApertureRad(aperture_az_m, wavelength_m));
  }
  if (beamwidth.el_beamwidth_deg <= 0.0f && aperture_el_m > 0.0f && wavelength_m > 0.0f) {
    beamwidth.el_beamwidth_deg =
        RadToDeg(DeriveBeamwidthFromApertureRad(aperture_el_m, wavelength_m));
  }
  return beamwidth;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_BEAMWIDTH_RESOLUTION_H_
