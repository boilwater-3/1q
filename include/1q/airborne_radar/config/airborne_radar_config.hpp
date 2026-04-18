/**
 * @file airborne_radar_config.hpp
 * @brief 机载雷达模块配置统一入口。
 * @note 当调用方只需要公开配置 API 时，优先包含本头而不是逐个引入 `config/` 子头。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AIRBORNE_RADAR_CONFIG_HPP_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AIRBORNE_RADAR_CONFIG_HPP_

#include "1q/airborne_radar/config/ConfigModel.h"
#include "1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/airborne_radar/config/expert/ExpertPipelineConfig.h"
#include "1q/airborne_radar/config/presets/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/config/presets/PipelineConfigPresets.h"
#include "1q/airborne_radar/config/semantic/AntennaPatternConfig.h"
#include "1q/airborne_radar/config/semantic/BeamControlConfig.h"
#include "1q/airborne_radar/config/semantic/DetectionConfig.h"
#include "1q/airborne_radar/config/semantic/LifecycleConfig.h"
#include "1q/airborne_radar/config/semantic/TrackingConfig.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AIRBORNE_RADAR_CONFIG_HPP_
