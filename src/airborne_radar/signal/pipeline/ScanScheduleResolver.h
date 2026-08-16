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
 * @return 当前周期波束指向；STBY 返回零位、STT/LRR 返回扫描中心。
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
 * @return 驻留偏移量（波束指向减去扫描中心）；STT/LRR 模式返回零偏。
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

/**
 * @brief 从执行配置只读解析当前周期调度的波束指向。
 * @param[in] runtime_config 运行时执行配置（读取 orientation、beam_control
 *            scheduler 与天线名义波束宽度，不修改）。
 * @param[in] cycle_index 1 基周期编号。
 * @return 当前周期调度波束指向；与 ApplyScanScheduleToRuntimeConfig 写入
 *         scan_center_deg 的值一致，供 session 级指向（发射/接收/增益/检测）
 *         与 pipeline 本地副本共用同一扫描相位。
 */
config::AzimuthElevationDeg ResolveScheduledBeamPointingFromExecutionConfig(
    const ExecutionConfig& runtime_config, std::uint32_t cycle_index);

/**
 * @brief 由雷达局部笛卡尔位置换算方位/俯仰指向。
 * @param[in] position_x 雷达局部 x（m；平台 ENU 切平面东向，含平台姿态旋转）。
 * @param[in] position_y 雷达局部 y（m；北向，含平台姿态旋转）。
 * @param[in] position_z 雷达局部 z（m；上向，含平台姿态旋转）。
 * @param[out] pointing 换算出的方位/俯仰指向（单位：deg）。
 * @return 位置有效（分量有限且范数 > 0）时返回 true；否则返回 false 且不写指向。
 * @note 口径与 RIR ComputeLookAngles 一致：az = atan2(y, x)、el = atan2(z, hypot(x, y))。
 */
bool TryTrackPositionToLookAnglesDeg(float position_x, float position_y, float position_z,
                                     config::AzimuthElevationDeg* pointing);

/**
 * @brief STT 指定航迹跟随指向的解析结果。
 */
struct SttTrackFollowingResolution {
  bool track_following_active{
      false}; /**< 本次指向是否跟随指定航迹（true 时 scan_center_deg 为航迹指向）。 */
  config::AzimuthElevationDeg scan_center_deg{}; /**< 传给挂架指向解算的扫描中心。 */
  config::AzimuthElevationDeg
      dwell_center_deg{}; /**< 传给挂架指向解算的驻留偏移（非显式时为零偏）。 */
};

/**
 * @brief 解析 STT 指定航迹跟随的指向来源。
 *
 * 指向来源优先级（冻结语义，不得改序；外部见 ArRuntimeConfigPatch 同款注释）：
 *   1) 显式 dwell_center_deg 非零：最终指向 = scan_center + dwell（现状语义，最高优先，
 *      老行为不回归）；
 *   2) work_mode == kStt 且已指定目标且其航迹 confirmed：最终指向 = 指定航迹位置换算的
 *      az/el（雷达局部系），dwell 视为零偏移（雷达自动跟随自身航迹）；
 *   3) 其余（未指定/航迹未确认/丢失/不存在/非 STT）：最终指向 = scan_center（现状行为，
 *      dwell 非显式时为零偏）。
 * @param[in] orientation_config 雷达方向与控制配置（仅读取 work_mode 与 scan_center_deg）。
 * @param[in] dwell_center_deg 显式驻留偏移（运行期 patch 给定，零值视为未显式设置）。
 * @param[in] has_designated_target 是否已指定跟踪目标。
 * @param[in] designated_track_confirmed 指定目标航迹是否 confirmed。
 * @param[in] track_pointing_deg 指定航迹位置换算的雷达局部指向（未确认时忽略）。
 * @return 指向来源解析结果（track_following_active 供结果/日志标记本次自动跟随）。
 */
SttTrackFollowingResolution ResolveSttTrackFollowingPointing(
    const config::ArOrientationConfig& orientation_config,
    const config::AzimuthElevationDeg& dwell_center_deg, bool has_designated_target,
    bool designated_track_confirmed, const config::AzimuthElevationDeg& track_pointing_deg);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
