/**
 * @file SignalPipelineRuntimeTypes.h
 * @brief 汇聚 SignalPipeline 内部运行时需要的配置与结果类型。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RUNTIME_TYPES_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RUNTIME_TYPES_H_

#include <vector>

#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "airborne_radar/signal/pipeline/SignalCycleResult.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/electromagnetics/RfScene.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using AssociationQualityMetrics = session::AssociationQualityMetrics;
using SignalCycleResult = session::SignalCycleResult;

/** @brief Complete 冻结并由下一次成功 pipeline 周期一次性消费的 RF v2 detection 上下文。 */
struct RfV2DetectionContext {
  oneq::electromagnetics::RfEmissionIdentity own_emission_identity{};
  oneq::electromagnetics::RfWaveformSchedule own_transmit_waveform{};
  double receive_window_start_time_s{0.0};
  double receive_window_duration_s{0.0};
  config::AzimuthElevationDeg beam_pointing_deg{};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{};
  bool enable_anti_rgpo_leading_edge{false}; /**< 前沿跟踪对抗 RGPO */
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RUNTIME_TYPES_H_
