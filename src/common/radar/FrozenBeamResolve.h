/**
 * @file FrozenBeamResolve.h
 * @brief 冻结指向下的波束宽度 + 方向图增益求解（header-only 单源）。
 *
 * 不包含 AR 挂架稳定 / 驻留叠加；调用方先解析有效波束宽度与指向后再调用。
 */

#ifndef COMMON_RADAR_FROZEN_BEAM_RESOLVE_H_
#define COMMON_RADAR_FROZEN_BEAM_RESOLVE_H_

#include "common/radar/AntennaPatternRuntime.h"
#include "common/radar/BeamwidthResolution.h"
#include "common/radar/ScanScheduleRuntime.h"

namespace oneq {
namespace common {
namespace radar {

/** @brief 冻结波束求解输入（标量 + 已解析波束宽度）。 */
struct FrozenBeamResolveInputs {
  float main_beam_gain_db{0.0f};
  bool enable_directional_pattern{false};
  AntennaPatternConfig pattern{};
  EffectiveBeamwidthDeg effective_beamwidth_deg{};
  float beam_pointing_az_deg{0.0f};
  float beam_pointing_el_deg{0.0f};
  float look_az_deg{0.0f};
  float look_el_deg{0.0f};
  bool has_look_angles{false};
  float antenna_length_m{0.0f};
  float antenna_width_m{0.0f};
  float wavelength_m{0.0f};
  /** RIR 在构造离轴角前归一化方位差；AR ResolveFrozen 不预归一化。 */
  bool normalize_azimuth_delta{false};
};

/** @brief 冻结波束求解输出。 */
struct FrozenBeamResolveResult {
  EffectiveBeamwidthDeg effective_beamwidth_deg{};
  float one_way_antenna_gain_db{0.0f};
};

/**
 * @brief 给定冻结指向与目标视线，求解有效波束宽度与单程增益。
 */
inline FrozenBeamResolveResult ResolveFrozenBeamState(const FrozenBeamResolveInputs& inputs) {
  FrozenBeamResolveResult result;
  result.effective_beamwidth_deg = inputs.effective_beamwidth_deg;
  result.one_way_antenna_gain_db = inputs.main_beam_gain_db;
  if (!inputs.enable_directional_pattern || !inputs.has_look_angles) {
    return result;
  }

  AntennaPatternBeamwidthDeg pattern_beamwidth;
  pattern_beamwidth.az_beamwidth_deg = inputs.effective_beamwidth_deg.az_beamwidth_deg;
  pattern_beamwidth.el_beamwidth_deg = inputs.effective_beamwidth_deg.el_beamwidth_deg;

  float delta_az_deg = inputs.look_az_deg - inputs.beam_pointing_az_deg;
  if (inputs.normalize_azimuth_delta) {
    delta_az_deg = NormalizeAzimuthDeltaDeg(delta_az_deg);
  }
  AntennaLookOffsetDeg offset_deg;
  offset_deg.delta_az_deg = delta_az_deg;
  offset_deg.delta_el_deg = inputs.look_el_deg - inputs.beam_pointing_el_deg;

  const AzimuthElevationDeg pointing(inputs.beam_pointing_az_deg, inputs.beam_pointing_el_deg);
  result.one_way_antenna_gain_db =
      EvaluateAntennaPattern(inputs.main_beam_gain_db, inputs.pattern, pattern_beamwidth,
                             offset_deg, pointing, inputs.antenna_length_m,
                             inputs.antenna_width_m, inputs.wavelength_m)
          .gain_dbi;
  return result;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_FROZEN_BEAM_RESOLVE_H_
