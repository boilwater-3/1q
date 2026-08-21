/**
 * @file EsrResolutionRules.h
 * @brief ESR 配置解析的单一规则源（work-mode 调整、scan policy 解析、scan 边界归一）。
 *
 * 这些规则同时被会话初始化（EsrSessionConfigResolver）和运行期补丁
 * （EsrRuntimeConfigResolver）消费。收敛到单一实现，避免两份副本各自维护导致的
 * 语义漂移。本单元为模块内部私有，不对外暴露。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_RESOLUTION_RULES_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_RESOLUTION_RULES_H_

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"

namespace electronic_surveillance_radar {
namespace session {
namespace resolution_rules {

/**
 * @brief 归一化扫描边界，保证 start <= end（原地交换）。
 * @param[in,out] start 起始角度，可为 nullptr。
 * @param[in,out] end   结束角度，可为 nullptr。
 */
void NormalizeScanBounds(float* start, float* end);

/**
 * @brief 按 work-mode 调整检测参数（脉冲数、阈值缩放）。
 *
 * - kHgesm：脉冲数 ×4（上限 4096），阈值 ×0.85（下限 0.1）。
 * - kRwr：脉冲数 ÷2（下限 1），阈值 ×1.25（下限 0.1）。
 * - kEsm：不调整。
 *
 * 进入前会对 pulse_count 做下限 1 钳制，对非法/非正 threshold_scale 回落到 1.0。
 *
 * @param[in]     mode             工作 mode。
 * @param[in,out] detection_config 待调整的检测参数，可为 nullptr（直接返回）。
 */
void ApplyWorkModeAdjustment(config::EsrWorkMode mode, DetectionConfig* detection_config);

/**
 * @brief 解析 scan policy 与 hardware 到运行态扫描配置。
 *
 * 解析优先级：显式扫描边界（explicit bounds）> 中心角（center）> 当前透传。
 * beam 波束宽度会覆盖 az/el 步进；天线安装角（mount）会从边界/中心角中扣除。
 *
 * @param[in]     hardware     硬件配置（beam 宽度、扫描范围）。
 * @param[in]     orientation  静态安装指向（mount az/el）。
 * @param[in]     scan_policy  扫描策略（显式边界 / 中心角 / 起点、序列）。
 * @param[in,out] scan_config  待填充的运行态扫描配置，可为 nullptr（直接返回）。
 */
void ApplyScanPolicy(const config::EsrHardwareConfig& hardware,
                     const config::EsrOrientationConfig& orientation,
                     const config::EsrScanPolicyConfig& scan_policy,
                     extension::InterceptScanConfig* scan_config);

}  // namespace resolution_rules
}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_RESOLUTION_RULES_H_
