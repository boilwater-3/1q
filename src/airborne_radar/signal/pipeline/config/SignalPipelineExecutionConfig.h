/**
 * @file SignalPipelineExecutionConfig.h
 * @brief 定义 SignalPipeline 内部执行阶段使用的配置类型别名。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_EXECUTION_CONFIG_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_EXECUTION_CONFIG_H_

#include "airborne_radar/config/legacy/PipelineConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using PipelineConfig = config::PipelineConfig;

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_EXECUTION_CONFIG_H_
