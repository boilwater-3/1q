/**
 * @file InternalPipelineConfig.h
 * @brief 定义机载雷达流水线内部扩展配置。
 *
 * 本文件保留兼容类型别名，供尚未完成 InternalExecutionConfig 迁移的模块过渡使用。
 * 新代码应直接使用 config::execution::InternalExecutionConfig。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_PIPELINE_CONFIG_H_

#include "airborne_radar/config/execution/InternalExecutionConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

using InternalPipelineConfig = ::airborne_radar::config::execution::InternalExecutionConfig;
using JammingEffectsConfig = ::airborne_radar::config::execution::JammingEffectsConfig;
using ControlProfileEffectsConfig =
    ::airborne_radar::config::execution::ControlProfileEffectsConfig;
using ExecutionConfig = ::airborne_radar::config::execution::InternalExecutionConfig;

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_PIPELINE_CONFIG_H_
