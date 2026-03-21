#include "airborne_radar/signal/detection/SignalDetector.h"

#include <cmath>

namespace airborne_radar {
namespace signal {
namespace detection {

SignalDetector::SignalDetector(RadarSystemConfig config)
    : config_(config),
      thermal_noise_w_(RadarEquations::ComputeThermalNoisePower_W(
          config.transmitter, config.receiver)),
      rng_(42u) {  // 默认种子，可通过 SetRandomSeed 重置
}

void SignalDetector::UpdateConfig(RadarSystemConfig config) {
  config_ = config;
  thermal_noise_w_ = RadarEquations::ComputeThermalNoisePower_W(
      config_.transmitter, config_.receiver);
}

DetectionResult SignalDetector::Detect(
    const TargetReturn& target,
    const EnvironmentState& env,
    float one_way_antenna_gain_db,
    int pulse_count,
    bool coherent_integration) {
  DetectionResult result;
  if (std::isnan(one_way_antenna_gain_db)) {
    one_way_antenna_gain_db = config_.antenna.main_beam_gain_db;
  }

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

  return result;
}

void SignalDetector::SetRandomSeed(unsigned int seed) {
  rng_.seed(seed);
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
