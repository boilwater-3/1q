/**
 * @file ScanScheduleResolver.h
 * @brief 定义机载雷达二维扫描调度解析工具。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_

#include <cstdint>
#include <vector>

#include "airborne_radar/utils/RadarOrientationUtils.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace core {

namespace internal {

model::AzimuthElevationDeg ResolveFiniteScanCenter(
    const model::RadarOrientationConfig& orientation_config);

float ResolveScanStepScale(model::RadarWorkSubMode mode);

std::vector<model::AzimuthElevationDeg> BuildScheduledScanPattern(
    const model::AzimuthElevationLimitsDeg& limits, float az_step_deg, float el_step_deg,
    oneq::foundation::ScanStartPosition start_position, oneq::foundation::ScanSequence sequence);

model::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const model::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

model::AzimuthElevationDeg ResolveScheduledDwellCenter(
    const model::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

void ApplyScanScheduleToRuntimeConfig(std::uint32_t cycle_index,
                                      SignalPipelineConfig* runtime_config);

}  // namespace internal
}  // namespace core
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
