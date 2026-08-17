/**
 * @file AntennaPatternRuntime.h
 * @brief 定义方向图运行期中间量与评估函数（AR 薄适配层，common 单源）。
 * @note 本文件仅供 `signal::detection` 链路内部使用，不作为公开 API。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_ANTENNA_PATTERN_RUNTIME_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_ANTENNA_PATTERN_RUNTIME_H_

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "common/radar/AntennaPatternRuntime.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief AntennaPatternBeamwidthDeg 表示方向图评估使用的有效波束宽度。
 */
struct AntennaPatternBeamwidthDeg {
  float az_beamwidth_deg{3.0f};
  float el_beamwidth_deg{3.0f};
};

/**
 * @brief AntennaLookOffsetDeg 表示目标相对当前波束中心的离轴角。
 */
struct AntennaLookOffsetDeg {
  float delta_az_deg{0.0f};
  float delta_el_deg{0.0f};
};

/**
 * @brief AntennaPatternSample 表示方向图采样结果。
 */
struct AntennaPatternSample {
  float gain_dbi{0.0f};
  float main_lobe_attenuation_db{0.0f};
  float scan_loss_db{0.0f};
  bool inside_main_lobe{false};
  bool inside_back_lobe{false};
};

namespace antenna_pattern_adapter {

inline oneq::common::radar::AntennaPatternBeamwidthDeg ToCommonBeamwidth(
    const AntennaPatternBeamwidthDeg& beamwidth_deg) {
  return oneq::common::radar::AntennaPatternBeamwidthDeg{beamwidth_deg.az_beamwidth_deg,
                                                         beamwidth_deg.el_beamwidth_deg};
}

inline oneq::common::radar::AntennaLookOffsetDeg ToCommonLookOffset(
    const AntennaLookOffsetDeg& offset_deg) {
  return oneq::common::radar::AntennaLookOffsetDeg{offset_deg.delta_az_deg,
                                                   offset_deg.delta_el_deg};
}

inline oneq::common::radar::AntennaPatternConfig ToCommonPatternConfig(
    const config::engineering::AntennaPatternConfig& config) {
  oneq::common::radar::AntennaPatternConfig common_config;
  common_config.model_type = static_cast<oneq::common::radar::AntennaPatternModelType>(
      static_cast<int>(config.model_type));
  common_config.max_sidelobe_level_db = config.max_sidelobe_level_db;
  common_config.backlobe_level_db = config.backlobe_level_db;
  common_config.scan_loss_coeff_db_per_deg2 = config.scan_loss_coeff_db_per_deg2;
  common_config.max_scan_loss_db = config.max_scan_loss_db;
  common_config.boresight_offset_deg.az_deg = config.boresight_offset_deg.az_deg;
  common_config.boresight_offset_deg.el_deg = config.boresight_offset_deg.el_deg;
  return common_config;
}

inline oneq::common::radar::AzimuthElevationDeg ToCommonPointing(
    const config::AzimuthElevationDeg& beam_pointing_deg) {
  return oneq::common::radar::AzimuthElevationDeg{beam_pointing_deg.az_deg,
                                                  beam_pointing_deg.el_deg};
}

inline AntennaPatternSample FromCommonSample(
    const oneq::common::radar::AntennaPatternSample& sample) {
  AntennaPatternSample result;
  result.gain_dbi = sample.gain_dbi;
  result.main_lobe_attenuation_db = sample.main_lobe_attenuation_db;
  result.scan_loss_db = sample.scan_loss_db;
  result.inside_main_lobe = sample.inside_main_lobe;
  result.inside_back_lobe = sample.inside_back_lobe;
  return result;
}

}  // namespace antenna_pattern_adapter

inline bool IsInsideMainLobe(const AntennaPatternBeamwidthDeg& beamwidth_deg,
                             const AntennaLookOffsetDeg& offset_deg) {
  return oneq::common::radar::IsInsideMainLobe(
      antenna_pattern_adapter::ToCommonBeamwidth(beamwidth_deg),
      antenna_pattern_adapter::ToCommonLookOffset(offset_deg));
}

inline float ComputeMainLobeAttenuationDb(
    const config::engineering::AntennaPatternConfig& config,
    const AntennaPatternBeamwidthDeg& beamwidth_deg, const AntennaLookOffsetDeg& offset_deg,
    float antenna_az_length_m = 0.0f, float antenna_el_width_m = 0.0f,
    float wavelength_m = 0.0f) {
  return oneq::common::radar::ComputeMainLobeAttenuationDb(
      antenna_pattern_adapter::ToCommonPatternConfig(config),
      antenna_pattern_adapter::ToCommonBeamwidth(beamwidth_deg),
      antenna_pattern_adapter::ToCommonLookOffset(offset_deg), antenna_az_length_m,
      antenna_el_width_m, wavelength_m);
}

inline float ComputeScanLossDb(const config::engineering::AntennaPatternConfig& config,
                               const config::AzimuthElevationDeg& beam_pointing_deg) {
  return oneq::common::radar::ComputeScanLossDb(
      antenna_pattern_adapter::ToCommonPatternConfig(config),
      antenna_pattern_adapter::ToCommonPointing(beam_pointing_deg));
}

inline AntennaPatternSample EvaluateAntennaPattern(
    float peak_gain_dbi, const config::engineering::AntennaPatternConfig& config,
    const AntennaPatternBeamwidthDeg& beamwidth_deg, const AntennaLookOffsetDeg& offset_deg,
    const config::AzimuthElevationDeg& beam_pointing_deg, float antenna_az_length_m = 0.0f,
    float antenna_el_width_m = 0.0f, float wavelength_m = 0.0f) {
  return antenna_pattern_adapter::FromCommonSample(
      oneq::common::radar::EvaluateAntennaPattern(
          peak_gain_dbi, antenna_pattern_adapter::ToCommonPatternConfig(config),
          antenna_pattern_adapter::ToCommonBeamwidth(beamwidth_deg),
          antenna_pattern_adapter::ToCommonLookOffset(offset_deg),
          antenna_pattern_adapter::ToCommonPointing(beam_pointing_deg), antenna_az_length_m,
          antenna_el_width_m, wavelength_m));
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_ANTENNA_PATTERN_RUNTIME_H_
