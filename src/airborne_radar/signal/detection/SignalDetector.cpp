#include "airborne_radar/signal/detection/SignalDetector.h"

#include <cmath>

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

oneq::common::radar::StatisticalCfarPolicy ToPolicy(
    const config::engineering::DetectionConfig& config) {
  oneq::common::radar::StatisticalCfarPolicy policy;
  policy.cfar_pfa = config.detection_policy.cfar_pfa;
  policy.min_snr_db = config.detection_policy.min_snr_db;
  policy.min_detection_margin_db = config.min_detection_margin_db;
  return policy;
}

DetectionResult FromCommon(const oneq::common::radar::StatisticalCfarResult& in) {
  DetectionResult out;
  out.echo_power_dbw = in.echo_power_dbw;
  out.snr_db = in.snr_db;
  out.detection_prob = in.detection_prob;
  out.detected = in.detected;
  return out;
}

}  // namespace

SignalDetector::SignalDetector(config::engineering::DetectionConfig config)
    : config_(config),
      cfar_(RadarEquations::ComputeThermalNoisePower_W(config.transmitter, config.receiver),
            ToPolicy(config)) {}

void SignalDetector::UpdateConfig(config::engineering::DetectionConfig config) {
  config_ = config;
  cfar_.Update(RadarEquations::ComputeThermalNoisePower_W(config_.transmitter, config_.receiver),
               ToPolicy(config_));
}

DetectionResult SignalDetector::Detect(const TargetReturn& target, const EnvironmentState& env,
                                       float one_way_antenna_gain_db, int pulse_count) {
  if (std::isnan(one_way_antenna_gain_db)) {
    one_way_antenna_gain_db = config_.antenna.main_beam_gain_db;
  }

  float echo_power_dbw = RadarEquations::ComputeEchoPowerWithGain_dBW(
      config_.transmitter, one_way_antenna_gain_db, target.rcs_m2, target.range_m,
      env.propagation_loss_db);
  echo_power_dbw -= config_.receiver.receive_loss_db;

  return FromCommon(cfar_.DetectFromEchoBudget(echo_power_dbw, env.clutter_noise_w, env.jam_noise_w,
                                               ToCommonSwerling(target.swerling_type),
                                               pulse_count));
}

DetectionResult SignalDetector::DetectResolvedCell(const TargetReturn& target,
                                                   const ArDetectionCellResult& cell) {
  return FromCommon(cfar_.DetectResolvedCell(
      static_cast<float>(cell.echo_power_w),
      static_cast<float>(cell.processed_single_pulse_sinr_db),
      ToCommonSwerling(target.swerling_type), static_cast<int>(cell.effective_pulse_count)));
}

void SignalDetector::SetRandomSeed(unsigned int seed) { cfar_.SetRandomSeed(seed); }

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
