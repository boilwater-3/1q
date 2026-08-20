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

float RadarEquations::ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz) {
  return oneq::common::radar::RadarEquations::ComputeRangeErrorStdDev(snr_db, bandwidth_hz);
}

float RadarEquations::ComputeAngleErrorStdDev(float snr_db, float beamwidth_rad) {
  return oneq::common::radar::RadarEquations::ComputeAngleErrorStdDev(snr_db, beamwidth_rad);
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
