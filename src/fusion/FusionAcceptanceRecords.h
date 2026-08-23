/**
 * @file FusionAcceptanceRecords.h
 * @brief 融合层验收行拼装。
 */

#ifndef ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <vector>

#include "1q/fusion/FusedTarget.h"
#include "1q/fusion/FusionConfig.h"

namespace fusion {

/**
 * @brief 相邻两周期视线角速度（单位：deg/s）：|Δ(az,el)| / dt。
 * @param[in] az0_deg/el0_deg 上一周期方位/俯仰（deg）。
 * @param[in] az1_deg/el1_deg 本周期方位/俯仰（deg）。
 * @param[in] dt_sec           周期间隔（s，≤ 0 返回 0）。
 * @note 不受验收日志宏门控，恒编译可测。
 */
double AngularSpeedDegPerSec(double az0_deg, double el0_deg, double az1_deg, double el1_deg,
                             double dt_sec);

/**
 * @brief 接力覆盖时长（单位：s）：视场角宽度 ÷ 视线角速率。
 * @param[in] fov_width_deg       视场角宽度（deg，FusionConfig::relay_fov_width_deg）。
 * @param[in] angular_speed_deg_per_s 视线角速率（deg/s）。
 * @return 速率为正且视场为正时返回覆盖时长；否则返回 -1.0（不可用，调用方写无）。
 * @note 简易外推：目标以当前角速率匀速扫掠一个视场宽度——2026-08-22 甲方批注
 *       「加个简易算法预测」的口径，库内无接力协议。
 */
double RelayCoverageSec(double fov_width_deg, double angular_speed_deg_per_s);

void WriteFusionAcceptance(std::uint32_t cycle, const std::vector<FusedTarget>& tracks,
                           bool filtering_enabled, const FusionConfig& config);

}  // namespace fusion

#endif
