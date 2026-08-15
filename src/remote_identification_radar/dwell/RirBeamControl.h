/**
 * @file RirBeamControl.h
 * @brief 定义 RIR 波束宽度解析与给定指向下的方向图增益求解（私有实现头）。
 *
 * 副本来源：`src/airborne_radar/signal/detection/BeamwidthResolution.h` 与
 * `BeamControlResolver.h` 的 ResolveFrozen 路径（审计基线 96de367c，阶段 2-M M2）。
 * 与 AR 版的刻意差异（副本头注纪律）：
 *   - 无 commanded 波束宽度覆盖分支（AR orientation 域机制，RIR 不随迁）；
 *   - 无平台姿态/稳定模式安装系指向解算（RIR 驻留指向由驻留调度显式给定，
 *     阶段 2-S 接线，见《阶段 2 计划》D-A5）；
 *   - 保留：有效波束宽度三级回退（commanded 删除后两级：nominal → λ/L 物理推导）、
 *     离轴角 → 方向图增益链路、enable_directional_pattern=false 回退主瓣峰值。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief RirEffectiveBeamwidthDeg 表示解析后的有效波束宽度（单位：度）。
 */
struct RirEffectiveBeamwidthDeg {
  float az_beamwidth_deg{0.0f}; /**< 有效方位波束宽度（单位：deg） */
  float el_beamwidth_deg{0.0f}; /**< 有效俯仰波束宽度（单位：deg） */
};

/**
 * @brief 从物理孔径尺寸推导半功率波束宽度（单位：rad）。
 * @param[in] aperture_length_m 孔径尺寸（单位：m）。
 * @param[in] wavelength_m 载波波长（单位：m）。
 * @return 半功率波束宽度（单位：rad），输入无效时返回 0。
 * @note 均匀孔径近似：θ_bw ≈ λ / L。
 */
inline float RirDeriveBeamwidthFromApertureRad(float aperture_length_m, float wavelength_m) {
  if (aperture_length_m <= 0.0f || wavelength_m <= 0.0f) {
    return 0.0f;
  }
  return wavelength_m / aperture_length_m;
}

/**
 * @brief 解析有效波束宽度（两级回退）。
 * @param antenna_config 天线配置。
 * @param wavelength_m 载波波长（单位：m），用于物理尺寸推导波束宽度（0=不使用物理推导）。
 * @return nominal_* 为正时直接生效；nominal_* 为 0 且 antenna_*_m>0 时从 λ/L 物理推导。
 */
inline RirEffectiveBeamwidthDeg RirResolveEffectiveBeamwidth(
    const config::hardware::RirAntennaConfig& antenna_config, float wavelength_m = 0.0f) {
  RirEffectiveBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = antenna_config.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = antenna_config.nominal_el_beamwidth_deg;

  constexpr float kRad2Deg = 180.0f / 3.14159265358979f;
  if (beamwidth.az_beamwidth_deg <= 0.0f && antenna_config.antenna_length_m > 0.0f &&
      wavelength_m > 0.0f) {
    beamwidth.az_beamwidth_deg =
        RirDeriveBeamwidthFromApertureRad(antenna_config.antenna_length_m, wavelength_m) * kRad2Deg;
  }
  if (beamwidth.el_beamwidth_deg <= 0.0f && antenna_config.antenna_width_m > 0.0f &&
      wavelength_m > 0.0f) {
    beamwidth.el_beamwidth_deg =
        RirDeriveBeamwidthFromApertureRad(antenna_config.antenna_width_m, wavelength_m) * kRad2Deg;
  }

  return beamwidth;
}

/**
 * @brief RirResolvedBeamState 表示 RIR 当前驻留使用的波束状态。
 */
struct RirResolvedBeamState {
  RirEffectiveBeamwidthDeg effective_beamwidth_deg{}; /**< 生效方位/俯仰波束宽度。 */
  config::RirAzimuthElevationDeg beam_pointing_deg{}; /**< 当前波束中心指向。 */
  float one_way_antenna_gain_db{0.0f}; /**< 当前目标方向上的单程天线增益（dB）。 */
};

/**
 * @brief 解析给定驻留指向下的目标方向波束状态。
 *
 * 对齐 AR `BeamControlResolver::ResolveFrozen`（冻结指向变体）：波束指向由
 * 驻留调度显式给定；`enable_directional_pattern=false` 或无有效视线角时
 * 回退主瓣峰值增益（阶段 1 旧行为，缺省兼容）。
 * @param antenna_config 天线配置。
 * @param beam_pointing_deg 驻留波束指向（雷达局部方位/俯仰，单位：度）。
 * @param target_look_az_deg 目标视线方位角（单位：度）。
 * @param target_look_el_deg 目标视线俯仰角（单位：度）。
 * @param has_look_angles 目标视线角是否有效。
 * @param wavelength_m 载波波长（单位：m），用于波束宽度物理推导与 sinc² 模式。
 * @return 波束状态（有效波束宽度 + 指向 + 单程增益）。
 */
inline RirResolvedBeamState RirResolveBeamStateForPointing(
    const config::hardware::RirAntennaConfig& antenna_config,
    const config::RirAzimuthElevationDeg& beam_pointing_deg, float target_look_az_deg,
    float target_look_el_deg, bool has_look_angles, float wavelength_m = 0.0f) {
  RirResolvedBeamState state;
  state.effective_beamwidth_deg = RirResolveEffectiveBeamwidth(antenna_config, wavelength_m);
  state.beam_pointing_deg = beam_pointing_deg;
  state.one_way_antenna_gain_db = antenna_config.main_beam_gain_db;
  if (!antenna_config.enable_directional_pattern || !has_look_angles) {
    return state;
  }
  RirAntennaPatternBeamwidthDeg pattern_beamwidth;
  pattern_beamwidth.az_beamwidth_deg = state.effective_beamwidth_deg.az_beamwidth_deg;
  pattern_beamwidth.el_beamwidth_deg = state.effective_beamwidth_deg.el_beamwidth_deg;
  RirAntennaLookOffsetDeg offset_deg;
  offset_deg.delta_az_deg = target_look_az_deg - state.beam_pointing_deg.az_deg;
  offset_deg.delta_el_deg = target_look_el_deg - state.beam_pointing_deg.el_deg;
  state.one_way_antenna_gain_db =
      RirEvaluateAntennaPattern(antenna_config.main_beam_gain_db, antenna_config.pattern,
                                pattern_beamwidth, offset_deg, state.beam_pointing_deg,
                                antenna_config.antenna_length_m, antenna_config.antenna_width_m,
                                wavelength_m)
          .gain_dbi;
  return state;
}

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_
