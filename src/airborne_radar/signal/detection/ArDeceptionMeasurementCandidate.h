/**
 * @file ArDeceptionMeasurementCandidate.h
 * @brief 定义由接收端欺骗残差解析出的内部候选量测。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DECEPTION_MEASUREMENT_CANDIDATE_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DECEPTION_MEASUREMENT_CANDIDATE_H_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace signal {
namespace detection {

constexpr double kVgpoCarrierOffsetGateHz = 1000.0;
constexpr double kRgpoFirstPulseDelayGateS = 1.0e-7;

/**
 * @brief 一条由 waveform delay/Doppler 残差解析出的独立欺骗候选量测。
 *
 * 该类型是 detection/resolution-cell 到 association/lifecycle 的内部端口，不进入公开
 * result 或 replay。RF identity 仅作内部 provenance；关联键必须由正常位置关联产生，
 * 不得由 emission id 或本周期簇顺序预分配。
 */
struct ArDeceptionMeasurementCandidate {
  std::uint64_t source_observation_id{0U};
  oneq::electromagnetics::RfEmissionIdentity source_emission_identity{};
  double estimated_first_pulse_delay_s{0.0};
  double estimated_carrier_offset_hz{0.0};
  double apparent_slant_range_m{0.0};
  double apparent_range_rate_mps{0.0};
  double jammer_to_noise_db{0.0};
  bool used_local_bearings{false};
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};
  Eigen::Matrix3f measurement_covariance{Eigen::Matrix3f::Zero()};
};

using ArDeceptionMeasurementCandidateList =
    std::vector<ArDeceptionMeasurementCandidate>;

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DECEPTION_MEASUREMENT_CANDIDATE_H_
