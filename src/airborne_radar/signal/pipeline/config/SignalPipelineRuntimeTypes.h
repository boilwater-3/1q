/**
 * @file SignalPipelineRuntimeTypes.h
 * @brief 汇聚 SignalPipeline 内部运行时需要的配置与结果类型。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RUNTIME_TYPES_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RUNTIME_TYPES_H_

#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineExecutionConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using AssociationQualityMetrics = extension::AssociationQualityMetrics;
using SignalCycleResult = extension::SignalCycleResult;

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RUNTIME_TYPES_H_
