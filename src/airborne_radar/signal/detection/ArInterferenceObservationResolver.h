/**
 * @file ArInterferenceObservationResolver.h
 * @brief 定义去真值化 AR 干扰观测的纯求解边界。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_AR_INTERFERENCE_OBSERVATION_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_AR_INTERFERENCE_OBSERVATION_RESOLVER_H_

#include <vector>

#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief 从已求解 incident links 生成超过 J/N 门的本地 RF 观测。
 * @note truth 身份只用于排除自身与查找场景几何，不写入 observation。
 */
bool TryResolveArInterferenceObservations(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double thermal_noise_power_w, double jammer_to_noise_gate_db,
    std::vector<session::ArInterferenceObservation>* observations);

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_AR_INTERFERENCE_OBSERVATION_RESOLVER_H_
