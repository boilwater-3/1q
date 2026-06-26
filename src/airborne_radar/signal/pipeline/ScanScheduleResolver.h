/**
 * @file ScanScheduleResolver.h
 * @brief 定义机载雷达二维扫描调度解析工具。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "airborne_radar/utils/RadarOrientationUtils.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

config::AzimuthElevationDeg ResolveFiniteScanCenter(
    const config::RadarOrientationConfig& orientation_config);

float ResolveScanStepScale(config::RadarWorkMode mode);

std::vector<config::AzimuthElevationDeg> BuildScheduledScanPattern(
    const config::AzimuthElevationLimitsDeg& limits, float az_step_deg, float el_step_deg,
    oneq::foundation::ScanStartPosition start_position, oneq::foundation::ScanSequence sequence);

config::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const config::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

config::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const config::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg,
    const config::BeamSchedulerConfig& scheduler_config,
    std::uint32_t cycle_index);

config::AzimuthElevationDeg ResolveScheduledDwellCenter(
    const config::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

void ApplyScanScheduleToRuntimeConfig(std::uint32_t cycle_index,
                                      ExecutionConfig* runtime_config);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
