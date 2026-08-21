/**
 * @file RirBeamControl.h
 * @brief 定义 RIR 波束宽度解析与给定指向下的方向图增益求解（私有实现头）。
 *
 * 波束宽度解析数值内核为 common 单源；方向图增益求解保留在本文件。
 * 与 AR 版的刻意差异：
 *   - 无 commanded 波束宽度覆盖分支（AR orientation 域机制，RIR 不随迁）；
 *   - 无平台姿态/稳定模式安装系指向解算：RIR 驻留指向由驻留调度显式给定，
 *     本文件只消费给定值（调度器给指向、RIR 信指向），见下方指向契约；
 *   - 保留：有效波束宽度两级回退（nominal → λ/L 物理推导）、
 *     离轴角 → 方向图增益链路、enable_directional_pattern=false 回退主瓣峰值。
 *
 * 驻留指向契约（冻结）：
 *   - 来源：调用方驻留调度器显式给定波束中心；RIR 不生成指向、不按目标位置
 *     重算或吸附指向。
 *   - 角度：`RirAzimuthElevationDeg`（deg），雷达局部 ENU 右手系；
 *     az ∈ [-180, 180]，el ∈ [-90, 90]，方位角须由调用方预先折算到该区间。
 *   - 含义：az=0/el=0 指向 +x（东向水平），az 正向转向 +y（北），
 *     el 正向指向 +z（天）；单位向量
 *     (cos(el)·cos(az), cos(el)·sin(az), sin(el))。
 *   - 离轴：目标视线角以同一坐标系公式计算；方位差经
 *     `NormalizeAzimuthDeltaDeg` 折算到 [-180, 180]，俯仰差直接相减。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_BEAM_CONTROL_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "common/radar/BeamwidthResolution.h"
#include "common/radar/ScanScheduleRuntime.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief RirEffectiveBeamwidthDeg 表示解析后的有效波束宽度（单位：度）。
 */
using RirEffectiveBeamwidthDeg = ::oneq::common::radar::EffectiveBeamwidthDeg;

/**
 * @brief 从物理孔径尺寸推导半功率波束宽度（单位：rad）。
 * @param[in] aperture_length_m 孔径尺寸（单位：m）。
 * @param[in] wavelength_m 载波波长（单位：m）。
 * @return 半功率波束宽度（单位：rad），输入无效时返回 0。
 * @note 均匀孔径近似：θ_bw ≈ λ / L。
 */
inline float RirDeriveBeamwidthFromApertureRad(float aperture_length_m, float wavelength_m) {
  return ::oneq::common::radar::DeriveBeamwidthFromApertureRad(aperture_length_m, wavelength_m);
}

/**
 * @brief 解析有效波束宽度（两级回退，无 commanded 覆盖）。
 * @param antenna_config 天线配置。
 * @param wavelength_m 载波波长（单位：m），用于物理尺寸推导波束宽度（0=不使用物理推导）。
 * @return nominal_* 为正时直接生效；nominal_* 为 0 且 antenna_*_m>0 时从 λ/L 物理推导。
 */
inline RirEffectiveBeamwidthDeg RirResolveEffectiveBeamwidth(
    const config::hardware::RirAntennaConfig& antenna_config, float wavelength_m = 0.0f) {
  return ::oneq::common::radar::ResolveEffectiveBeamwidth(
      antenna_config.nominal_az_beamwidth_deg, antenna_config.nominal_el_beamwidth_deg,
      antenna_config.antenna_length_m, antenna_config.antenna_width_m, wavelength_m);
}

/**
 * @brief RirResolvedBeamState 表示 RIR 当前驻留使用的波束状态。
 */
struct RirResolvedBeamState {
  RirEffectiveBeamwidthDeg effective_beamwidth_deg{}; /**< 生效方位/俯仰波束宽度。 */
  config::RirAzimuthElevationDeg beam_pointing_deg{}; /**< 当前波束中心指向（驻留调度显式给定，雷达局部 ENU 系，deg）。 */
  float one_way_antenna_gain_db{0.0f}; /**< 当前目标方向上的单程天线增益（dB）。 */
};

/**
 * @brief 解析给定驻留指向下的目标方向波束状态。
 *
 * 对齐 AR `BeamControlResolver::ResolveFrozen`（冻结指向变体）：波束指向由
 * 驻留调度显式给定，RIR 原样信任并消费（调度器给指向、RIR 信指向）。
 * `enable_directional_pattern=false` 或无有效视线角时回退主瓣峰值增益
 * （阶段 1 旧行为，缺省兼容）。
 *
 * @note 给定指向与目标视线角必须同属雷达局部 ENU 右手系：
 *       az ∈ [-180, 180]、el ∈ [-90, 90]；az=0/el=0 指向 +x，
 *       az 正向转向 +y，el 正向指向 +z。给定指向须预先折算到合法域；
 *       离轴方位差由本函数归一化；指向不朝向目标时按实际离轴角衰减。
 * @param antenna_config 天线配置。
 * @param beam_pointing_deg 驻留波束指向（调度器显式给定，雷达局部 ENU 系，单位：度）。
 * @param target_look_az_deg 目标视线方位角（同一雷达局部 ENU 系，单位：度）。
 * @param target_look_el_deg 目标视线俯仰角（同一雷达局部 ENU 系，单位：度）。
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
  offset_deg.delta_az_deg = ::oneq::common::radar::NormalizeAzimuthDeltaDeg(
      target_look_az_deg - state.beam_pointing_deg.az_deg);
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
