/**
 * @file RadarEquations.cpp
 * @brief AR 雷达方程薄适配层（common 单源）。
 */

#include "airborne_radar/signal/detection/RadarEquations.h"

#include "common/radar/RadarEquations.h"

namespace airborne_radar {
namespace signal {
namespace detection {

namespace {

oneq::common::radar::SwerlingModel ToCommonSwerling(config::profiles::SwerlingModel model) {
  switch (model) {
    case config::profiles::SwerlingModel::kSwerling1:
      return oneq::common::radar::SwerlingModel::kSwerling1;
    case config::profiles::SwerlingModel::kSwerling2:
      return oneq::common::radar::SwerlingModel::kSwerling2;
    case config::profiles::SwerlingModel::kSwerling3:
      return oneq::common::radar::SwerlingModel::kSwerling3;
    case config::profiles::SwerlingModel::kSwerling4:
      return oneq::common::radar::SwerlingModel::kSwerling4;
    case config::profiles::SwerlingModel::kSwerling0:
    default:
      return oneq::common::radar::SwerlingModel::kSwerling0;
  }
}

}  // namespace

float RadarEquations::ComputeEchoPowerWithGain_dBW(const config::engineering::TransmitterConfig& tx,
                                                   float one_way_gain_db, float rcs_m2,
                                                   float range_m, float propagation_loss_db) {
  return oneq::common::radar::RadarEquations::ComputeEchoPowerWithGain_dBW(
      tx.peak_power_w, tx.transmit_loss_db, tx.pulse_width_s, tx.frequency_hz, one_way_gain_db,
      rcs_m2, range_m, propagation_loss_db);
}

float RadarEquations::ComputeEchoPower_dBW(const config::engineering::TransmitterConfig& tx,
                                           const config::engineering::AntennaConfig& ant,
                                           float rcs_m2, float range_m, float propagation_loss_db) {
  return oneq::common::radar::RadarEquations::ComputeEchoPower_dBW(
      tx.peak_power_w, tx.transmit_loss_db, tx.pulse_width_s, tx.frequency_hz,
      ant.main_beam_gain_db, rcs_m2, range_m, propagation_loss_db);
}

float RadarEquations::ComputeThermalNoisePower_W(const config::engineering::TransmitterConfig& tx,
                                                 const config::engineering::ReceiverConfig& rx) {
  return oneq::common::radar::RadarEquations::ComputeThermalNoisePower_W(tx.bandwidth_hz,
                                                                          rx.noise_figure_db);
}

float RadarEquations::ComputeIntegrationGain(int pulse_count) {
  return oneq::common::radar::RadarEquations::ComputeIntegrationGain(pulse_count);
}

// AR 暂持旧合成口径（std 含系统偏差）：common 已于 2026-08-30 拆分 bias/std
// （RIR 专场），AR 适配层在随机项之上加回偏置以保持拆分前行为不变，待 AR 专场
// 对齐（欠账登记 docs/airborne_radar/algorithms.md）。低 SNR 下限分支
// （snr_db < -10 dB，与 common 内部 kMinSnrDb 同源耦合）拆分前本就不含偏置、
// 提前返回，因此该分支不加，保证全 SNR 域逐值一致。
float RadarEquations::ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz) {
  const float random_std =
      oneq::common::radar::RadarEquations::ComputeRangeErrorStdDev(snr_db, bandwidth_hz);
  return snr_db < -10.0f ? random_std
                         : random_std + oneq::common::radar::kRangeMeasurementBiasM;
}

float RadarEquations::ComputeAngleErrorStdDev(float snr_db, float beamwidth_rad) {
  const float random_std =
      oneq::common::radar::RadarEquations::ComputeAngleErrorStdDev(snr_db, beamwidth_rad);
  return snr_db < -10.0f
             ? random_std
             : random_std + oneq::common::radar::ComputeAngleMeasurementBiasRad(beamwidth_rad);
}

double RadarEquations::ComputeThreshold(double pfa, int num_pulses) {
  return oneq::common::radar::RadarEquations::ComputeThreshold(pfa, num_pulses);
}

double RadarEquations::MarcumQ(int order, double a, double b) {
  return oneq::common::radar::RadarEquations::MarcumQ(order, a, b);
}

float RadarEquations::ComputeDetectionProbability(float snr_db, float pfa,
                                                  config::profiles::SwerlingModel model,
                                                  int num_pulses) {
  return oneq::common::radar::RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, ToCommonSwerling(model), num_pulses);
}

bool RadarEquations::ThresholdDecision(float detection_prob, std::mt19937& rng) {
  return oneq::common::radar::RadarEquations::ThresholdDecision(detection_prob, rng);
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
