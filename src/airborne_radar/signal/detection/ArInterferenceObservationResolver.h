/**
 * @file ArInterferenceObservationResolver.h
 * @brief 定义去真值化 AR 干扰观测的纯求解边界。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_AR_INTERFERENCE_OBSERVATION_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_AR_INTERFERENCE_OBSERVATION_RESOLVER_H_

#include <vector>

#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief 从已求解 incident links 生成超过 J/N 门的本地 RF 观测。
 * @param[in] platform_frame 雷达局部坐标系（origin_lla + 合成姿态）。用于把 ECEF 视线
 *            转换到雷达局部系，写入 `estimated_bearing_*_local_deg`，与 ArSceneTarget 的
 *            look angle 同系，供假目标鉴别比较。当 origin_lla 非有限值时局部系字段留零
 *            并依赖调用方回退（不静默误标）。
 * @note truth 身份只用于排除自身与查找场景几何，不写入 observation。
 */
bool TryResolveArInterferenceObservations(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double thermal_noise_power_w, double jammer_to_noise_gate_db,
    const oneq::coordinate::LocalFrameReference& platform_frame,
    std::vector<session::ArInterferenceObservation>* observations);

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_AR_INTERFERENCE_OBSERVATION_RESOLVER_H_
