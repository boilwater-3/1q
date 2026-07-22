/**
 * @file ArRfFrontEndResolver.h
 * @brief 定义 AR RF v2 宽带接收前端的纯求解边界。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_AR_RF_FRONT_END_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_AR_RF_FRONT_END_RESOLVER_H_

#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/** @brief 一个接收窗口内冻结的宽带前端结果。 */
struct ArRfFrontEndResult {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{};
  double total_incident_power_w{0.0};
  bool receiver_saturated{false};
};

/**
 * @brief 求解 scene 中全部 emission 到接收设备输入端的单程链路并聚合功率。
 * @return 成功返回 true；非法 scene、缺失 co-site 路径或非法接收边界返回 false。
 * @note 失败时不修改 @p result；输出按完整发射身份稳定排序，与输入顺序无关。
 */
bool TryResolveArRfFrontEnd(const oneq::electromagnetics::RfSceneFrame& scene,
                            const oneq::electromagnetics::RfSceneReceiverState& receiver,
                            double maximum_linear_input_power_w,
                            const oneq::electromagnetics::RfIncidentLinkConfig& link_config,
                            ArRfFrontEndResult* result);

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_AR_RF_FRONT_END_RESOLVER_H_
