/**
 * @file SignalPipelineExecutionConfig.h
 * @brief 定义 SignalPipeline 内部执行阶段使用的配置类型别名。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_EXECUTION_CONFIG_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_EXECUTION_CONFIG_H_

#include "1q/airborne_radar/config/SignalPipelineConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using SignalPipelineConfig = common::config::SignalPipelineConfig;

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_EXECUTION_CONFIG_H_
