/**
 * @file RirSignalDetector.cpp
 * @brief RIR 统计级 CFAR 探测判决器实现（副本改写自 SignalDetector.cpp；
 *        阶段 2-M M4）。
 */

#include "remote_identification_radar/dwell/RirSignalDetector.h"

#include <algorithm>
#include <cmath>

namespace remote_identification_radar {
namespace dwell {

RirSignalDetector::RirSignalDetector(const RirDetectorConfig& config)
    : config_(config),
      thermal_noise_w_(
          internal::RirRadarEquations::ComputeThermalNoisePower_W(config.transmitter,
                                                                  config.receiver)),
      rng_(42u) {  // 默认种子，可通过 SetRandomSeed 重置
}

void RirSignalDetector::UpdateConfig(const RirDetectorConfig& config) {
  config_ = config;
  thermal_noise_w_ = internal::RirRadarEquations::ComputeThermalNoisePower_W(
      config_.transmitter, config_.receiver);
}

RirDetectionResult RirSignalDetector::Detect(const RirTargetReturn& target,
                                             const RirEnvironmentNoise& env,
                                             float one_way_antenna_gain_db, int pulse_count) {
  RirDetectionResult result;
  if (std::isnan(one_way_antenna_gain_db)) {
    one_way_antenna_gain_db = config_.antenna.main_beam_gain_db;
  }

  // ① 回波功率计算（对数域雷达方程）
  result.echo_power_dbw = internal::RirRadarEquations::ComputeEchoPowerWithGain_dBW(
      config_.transmitter, one_way_antenna_gain_db, target.rcs_m2, target.range_m,
      env.propagation_loss_db);

  // （扣除接收机系统损耗）
  result.echo_power_dbw -= config_.receiver.receive_loss_db;

  // 转换为线性功率 (W)
  const float echo_power_w = std::pow(10.0f, result.echo_power_dbw / 10.0f);

  // ② 综合噪声基底 = max(热噪声,0) + max(杂波,0) + max(干扰,0)
  //    并做最小值保护，避免负噪声或异常输入导致虚高 SNR。
  const float kNoiseFloorW = 1e-30f;
  const float safe_thermal_noise_w = std::max(thermal_noise_w_, 0.0f);
  const float safe_clutter_noise_w = std::max(env.clutter_noise_w, 0.0f);
  const float safe_jam_noise_w = std::max(env.jam_noise_w, 0.0f);
  const float total_noise_w =
      std::max(safe_thermal_noise_w + safe_clutter_noise_w + safe_jam_noise_w, kNoiseFloorW);

  // ③ 单脉冲 SNR = 回波功率 / 综合噪声
  const float base_snr_linear = echo_power_w / total_noise_w;
  result.snr_db = 10.0f * std::log10(base_snr_linear + kNoiseFloorW);

  // ④ 检测概率：使用单脉冲 SNR + 脉冲积累数量 N 的统一语义。
  const int effective_pulse_count = std::max(1, pulse_count);
  result.detection_prob = internal::RirRadarEquations::ComputeDetectionProbability(
      result.snr_db, config_.detection_policy.cfar_pfa, target.swerling_type,
      effective_pulse_count);

  // ⑤ 蒙特卡洛判决
  // 若单脉冲 SNR 低于硬截断下限，直接判为未探测。
  if (result.snr_db < config_.detection_policy.min_snr_db) {
    result.detected = false;
    result.detection_prob = 0.0f;
  } else {
    result.detected =
        internal::RirRadarEquations::ThresholdDecision(result.detection_prob, rng_);
  }

  // ⑥ 可靠性裕量门限：即使蒙特卡洛判为检测成功，
  //    SNR 低于 min_detection_margin_db 的检测也不可靠。
  if (result.detected && result.snr_db < config_.min_detection_margin_db) {
    result.detected = false;
  }

  return result;
}

RirDetectionResult RirSignalDetector::DetectResolvedCell(const RirTargetReturn& target,
                                                          const RirDetectionCellResult& cell) {
  RirDetectionResult result;
  if (!std::isfinite(cell.echo_power_w) || cell.echo_power_w < 0.0 ||
      !std::isfinite(cell.processed_single_pulse_sinr_db) ||
      cell.effective_pulse_count == 0U) {
    return result;
  }
  constexpr double kLinearFloor = 1.0e-300;
  result.echo_power_dbw = static_cast<float>(
      10.0 * std::log10(std::max(cell.echo_power_w, kLinearFloor)));
  result.snr_db = static_cast<float>(cell.processed_single_pulse_sinr_db);
  result.detection_prob = internal::RirRadarEquations::ComputeDetectionProbability(
      result.snr_db, config_.detection_policy.cfar_pfa, target.swerling_type,
      static_cast<int>(cell.effective_pulse_count));
  if (result.snr_db < config_.detection_policy.min_snr_db) {
    result.detection_prob = 0.0f;
    return result;
  }
  result.detected =
      internal::RirRadarEquations::ThresholdDecision(result.detection_prob, rng_);
  // 可靠性裕量门限：即使蒙特卡洛判为检测成功，
  // SNR 低于 min_detection_margin_db 的检测也不可靠。
  if (result.detected && result.snr_db < config_.min_detection_margin_db) {
    result.detected = false;
  }
  return result;
}

void RirSignalDetector::SetRandomSeed(unsigned int seed) { rng_.seed(seed); }

}  // namespace dwell
}  // namespace remote_identification_radar
