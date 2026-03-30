#include "airborne_radar/signal/detection/SignalDetector.h"

#include <algorithm>
#include <cmath>

namespace airborne_radar {
namespace signal {
namespace detection {

SignalDetector::SignalDetector(common::config::RadarSystemConfig config)
    : config_(config),
      thermal_noise_w_(
          RadarEquations::ComputeThermalNoisePower_W(config.transmitter, config.receiver)),
      rng_(42u) {  // 默认种子，可通过 SetRandomSeed 重置
}

void SignalDetector::UpdateConfig(common::config::RadarSystemConfig config) {
  config_ = config;
  thermal_noise_w_ =
      RadarEquations::ComputeThermalNoisePower_W(config_.transmitter, config_.receiver);
}

DetectionResult SignalDetector::Detect(const TargetReturn& target, const EnvironmentState& env,
                                       float one_way_antenna_gain_db, int pulse_count) {
  DetectionResult result;
  if (std::isnan(one_way_antenna_gain_db)) {
    one_way_antenna_gain_db = config_.antenna.main_beam_gain_db;
  }

  // ① 回波功率计算（对数域雷达方程）
  result.echo_power_dbw = RadarEquations::ComputeEchoPowerWithGain_dBW(
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

  // ④ 积累增益：相参积累 SNR ∝ N，非相参 ∝ √N；将增益折算到等效单脉冲 SNR。
  const float integration_gain = RadarEquations::ComputeIntegrationGain(pulse_count);
  const float integrated_snr_linear = base_snr_linear * integration_gain;
  const float integrated_snr_db = 10.0f * std::log10(integrated_snr_linear + kNoiseFloorW);

  // ⑤ 检测概率（以等效单脉冲 SNR + N=1 送入，积累已在上一步体现）
  result.detection_prob = RadarEquations::ComputeDetectionProbability(
      integrated_snr_db, config_.detection.cfar_pfa, target.swerling_type, 1);

  // ⑥ 蒙特卡洛判决
  // 若单脉冲 SNR 低于硬截断下限，直接判为未探测。
  if (result.snr_db < config_.detection.min_snr_db) {
    result.detected = false;
    result.detection_prob = 0.0f;
  } else {
    result.detected = RadarEquations::ThresholdDecision(result.detection_prob, rng_);
  }

  return result;
}

void SignalDetector::SetRandomSeed(unsigned int seed) { rng_.seed(seed); }

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
