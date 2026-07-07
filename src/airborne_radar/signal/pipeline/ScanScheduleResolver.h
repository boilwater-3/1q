/**
 * @file ScanScheduleResolver.h
 * @brief 定义机载雷达二维扫描调度解析工具。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "airborne_radar/utils/ArOrientationUtils.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 解析有限扫描中心，将 NaN 分量替换为 0。
 * @param[in] orientation_config 雷达方向与控制配置。
 * @return 分量均为有限值的扫描中心。
 */
config::AzimuthElevationDeg ResolveFiniteScanCenter(
    const config::ArOrientationConfig& orientation_config);

/**
 * @brief 解析给定工作模式下的扫描步长缩放因子。
 * @param[in] mode 工作模式。
 * @return 步长缩放因子（TAS=0.5，其余=1.0）。
 */
float ResolveScanStepScale(config::ArWorkMode mode);

/**
 * @brief 按扫描范围、步长、起点与顺序构建二维扫描波位序列。
 * @param[in] limits 扫描方位/俯仰范围。
 * @param[in] az_step_deg 方位步长（度）。
 * @param[in] el_step_deg 俯仰步长（度）。
 * @param[in] start_position 扫描起点位置。
 * @param[in] sequence 扫描顺序（方位优先或俯仰优先，蛇形往返）。
 * @return 波位序列；范围或步长非法时返回空向量。
 * @note 每轴采样数上限为 4096，超出后截断。
 */
std::vector<config::AzimuthElevationDeg> BuildScheduledScanPattern(
    const config::AzimuthElevationLimitsDeg& limits, float az_step_deg, float el_step_deg,
    oneq::foundation::ScanStartPosition start_position, oneq::foundation::ScanSequence sequence);

/**
 * @brief 解析当前周期调度的波束指向（使用默认调度配置）。
 * @param[in] orientation_config 雷达方向与控制配置。
 * @param[in] effective_beamwidth_deg 有效波束宽度。
 * @param[in] cycle_index 1 基周期编号。
 * @return 当前周期波束指向；STBY 返回零位、STT 返回扫描中心。
 */
config::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const config::ArOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

/**
 * @brief 解析当前周期调度的波束指向（使用显式调度配置）。
 * @param[in] orientation_config 雷达方向与控制配置。
 * @param[in] effective_beamwidth_deg 有效波束宽度。
 * @param[in] scheduler_config 波束调度配置（步长提示、密集采样偏好等）。
 * @param[in] cycle_index 1 基周期编号。
 * @return 当前周期波束指向；范围或波束宽度非法时回退到限幅后的扫描中心。
 */
config::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const config::ArOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg,
    const config::BeamSchedulerConfig& scheduler_config,
    std::uint32_t cycle_index);

/**
 * @brief 解析当前周期调度波束相对扫描中心的驻留偏移量。
 * @param[in] orientation_config 雷达方向与控制配置。
 * @param[in] effective_beamwidth_deg 有效波束宽度。
 * @param[in] cycle_index 1 基周期编号。
 * @return 驻留偏移量（波束指向减去扫描中心）；STT 模式返回零偏。
 */
config::AzimuthElevationDeg ResolveScheduledDwellCenter(
    const config::ArOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

/**
 * @brief 将当前周期调度波束指向就地写入运行时配置。
 * @param[in] cycle_index 1 基周期编号。
 * @param[in,out] runtime_config 待修改的运行时配置。
 * @note runtime_config 为 nullptr 时直接返回。
 */
void ApplyScanScheduleToRuntimeConfig(std::uint32_t cycle_index,
                                      ExecutionConfig* runtime_config);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
