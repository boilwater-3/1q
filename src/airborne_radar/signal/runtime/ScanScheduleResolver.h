/**
 * @file ScanScheduleResolver.h
 * @brief 定义机载雷达二维扫描调度解析工具。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_

#include <cstdint>
#include <vector>

#include "airborne_radar/common/utils/RadarOrientationUtils.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace runtime {

using pipeline::SignalPipelineConfig;

namespace internal {

common::model::AzimuthElevationDeg ResolveFiniteScanCenter(
    const common::model::RadarOrientationConfig& orientation_config);

float ResolveScanStepScale(common::model::RadarWorkSubMode mode);

std::vector<common::model::AzimuthElevationDeg> BuildScheduledScanPattern(
    const common::model::AzimuthElevationLimitsDeg& limits, float az_step_deg, float el_step_deg,
    oneq::common::ScanStartPosition start_position, oneq::common::ScanSequence sequence);

common::model::AzimuthElevationDeg ResolveScheduledBeamPointing(
    const common::model::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

common::model::AzimuthElevationDeg ResolveScheduledDwellCenter(
    const common::model::RadarOrientationConfig& orientation_config,
    const detection::EffectiveBeamwidthDeg& effective_beamwidth_deg, std::uint32_t cycle_index);

void ApplyScanScheduleToRuntimeConfig(std::uint32_t cycle_index,
                                      SignalPipelineConfig* runtime_config);

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SCAN_SCHEDULE_RESOLVER_H_
