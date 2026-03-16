// Copyright 2026. All Rights Reserved.
//
// Description: SignalDetector 的实现，完成物理化的回波评估与探测判决。

#include "airborne_radar/signal/detection/SignalDetector.h"

#include <cmath>

#include "1q/airborne_radar/common/AntennaPatternUtils.h"
#include "1q/airborne_radar/common/RadarOrientationUtils.h"
#include "1q/airborne_radar/signal/detection/BeamwidthResolution.h"

namespace airborne_radar {
namespace signal {
namespace detection {

namespace {

/// @brief 根据有效方位/俯仰波束宽度计算等效角度测量标准差。
/// @param snr_db   等效积累后的信噪比（dB）。
/// @param beamwidth_deg 已解析的有效方位/俯仰波束宽度。
/// @return 用于横向各向同性量测模型的等效角度标准差（rad）。
float ComputeEquivalentAngleErrorStdDev(float snr_db,
                                        const EffectiveBeamwidthDeg& beamwidth_deg) {
  const float kDeg2Rad = 3.14159265358979f / 180.0f;
  const float az_beamwidth_rad =
      beamwidth_deg.az_beamwidth_deg * kDeg2Rad;
  const float el_beamwidth_rad =
      beamwidth_deg.el_beamwidth_deg * kDeg2Rad;
  const float az_std_rad =
      RadarEquations::ComputeAngleErrorStdDev(snr_db, az_beamwidth_rad);
  const float el_std_rad =
      RadarEquations::ComputeAngleErrorStdDev(snr_db, el_beamwidth_rad);
  return std::sqrt(0.5f * (az_std_rad * az_std_rad +
                           el_std_rad * el_std_rad));
}

/// @brief 将有效波束宽度转换为方向图评估所需的公共结构。
/// @param beamwidth_deg 已解析的有效方位/俯仰波束宽度。
/// @return 方向图评估使用的波束宽度结构。
common::AntennaPatternBeamwidthDeg ToPatternBeamwidth(
    const EffectiveBeamwidthDeg& beamwidth_deg) {
  common::AntennaPatternBeamwidthDeg pattern_beamwidth;
  pattern_beamwidth.az_beamwidth_deg = beamwidth_deg.az_beamwidth_deg;
  pattern_beamwidth.el_beamwidth_deg = beamwidth_deg.el_beamwidth_deg;
  return pattern_beamwidth;
}

/// @brief 解析当前目标方向上的单程天线增益。
/// @param antenna_config 天线配置。
/// @param target 目标回波特征。
/// @param effective_beamwidth 已解析的有效波束宽度。
/// @param orientation_config 雷达方向与控制配置。
/// @return 当前目标方向上的单程天线增益（dB）。
/// @note 当前实现假定 target.look_az_deg / look_el_deg 已在雷达坐标系下表达。
float ResolveOneWayAntennaGainDb(
    const AntennaConfig& antenna_config,
    const TargetReturn& target,
    const EffectiveBeamwidthDeg& effective_beamwidth,
    const common::RadarOrientationConfig* orientation_config) {
  if (!antenna_config.enable_directional_pattern ||
      orientation_config == nullptr || !target.has_look_angles) {
    return antenna_config.main_beam_gain_db;
  }

  const common::AzimuthElevationDeg beam_pointing_deg =
      common::ComputeMountFrameBeamPointing(*orientation_config);
  common::AntennaLookOffsetDeg offset_deg;
  offset_deg.delta_az_deg = target.look_az_deg - beam_pointing_deg.az_deg;
  offset_deg.delta_el_deg = target.look_el_deg - beam_pointing_deg.el_deg;

  const common::AntennaPatternSample sample = common::EvaluateAntennaPattern(
      antenna_config.main_beam_gain_db, antenna_config.pattern,
      ToPatternBeamwidth(effective_beamwidth), offset_deg,
      orientation_config->scan_center_deg);
  return sample.gain_dbi;
}

}  // namespace

SignalDetector::SignalDetector(RadarSystemConfig config)
    : config_(config),
      thermal_noise_w_(RadarEquations::ComputeThermalNoisePower_W(
          config.transmitter, config.receiver)),
      rng_(42u) {  // 默认种子，可通过 SetRandomSeed 重置
}

DetectionResult SignalDetector::Detect(
    const TargetReturn& target,
    const EnvironmentState& env,
    int pulse_count,
    bool coherent_integration,
    const common::RadarOrientationConfig* orientation_config) {
  DetectionResult result;

  EffectiveBeamwidthDeg effective_beamwidth;
  if (orientation_config != nullptr) {
    effective_beamwidth =
        ResolveEffectiveBeamwidth(config_.antenna, *orientation_config);
  } else {
    effective_beamwidth.az_beamwidth_deg =
        config_.antenna.nominal_az_beamwidth_deg;
    effective_beamwidth.el_beamwidth_deg =
        config_.antenna.nominal_el_beamwidth_deg;
  }
  const float one_way_antenna_gain_db = ResolveOneWayAntennaGainDb(
      config_.antenna, target, effective_beamwidth, orientation_config);

  // ① 回波功率计算（对数域雷达方程）
  result.echo_power_dbw = RadarEquations::ComputeEchoPowerWithGain_dBW(
      config_.transmitter, one_way_antenna_gain_db,
      target.rcs_m2, target.range_m, env.propagation_loss_db);

  // （扣除接收机系统损耗）
  result.echo_power_dbw -= config_.receiver.receive_loss_db;

  // 转换为线性功率 (W)
  const float echo_power_w =
      std::pow(10.0f, result.echo_power_dbw / 10.0f);

  // ② 综合噪声基底 = 热噪声 + 杂波 + 干扰
  const float total_noise_w =
      thermal_noise_w_ + env.clutter_noise_w + env.jam_noise_w;

  // ③ 基准单脉冲信噪比 = 回波功率 / 综合噪声
  const float kEps = 1e-30f;
  float base_snr_linear = echo_power_w / total_noise_w;
  if (total_noise_w <= kEps) {
    result.snr_db = 100.0f;  // 无噪声环境保护值
  } else {
    result.snr_db = 10.0f * std::log10(base_snr_linear + kEps);
  }

  // ④ 计算积累增益（改善等效信噪比）
  const float integration_gain = RadarEquations::ComputeIntegrationGain(
      pulse_count, coherent_integration);
  // 对于检测概率计算，使用等效积累信噪比
  const float effective_snr_db = result.snr_db + 10.0f * std::log10(integration_gain);

  // ⑤ 检测概率（结合 Swerling 模型与多脉冲）
  result.detection_prob = RadarEquations::ComputeDetectionProbability(
      effective_snr_db, config_.detection.cfar_pfa,
      target.swerling_type, pulse_count);

  // ⑥ 蒙特卡洛判决
  // 若等效 SNR 低于硬截断下限，直接判为未探测
  if (effective_snr_db < config_.detection.min_snr_db) {
    result.detected = false;
    result.detection_prob = 0.0f;
  } else {
    result.detected = RadarEquations::ThresholdDecision(
        result.detection_prob, rng_);
  }

  // ⑦ 测量误差标准差（仅在探测成功时有意义，基于等效积累 SNR 计算）
  // 积累能显著改善测量精度。
  result.range_error_std_m = RadarEquations::ComputeRangeErrorStdDev(
      effective_snr_db, config_.transmitter.bandwidth_hz);

  // 将方位/俯仰二维波束宽度收敛为单一等效角度标准差，便于后续构建
  // 笛卡尔位置量测协方差时采用横向各向同性近似。
  result.angle_error_std_rad =
      ComputeEquivalentAngleErrorStdDev(effective_snr_db, effective_beamwidth);

  return result;
}

void SignalDetector::SetRandomSeed(unsigned int seed) {
  rng_.seed(seed);
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
