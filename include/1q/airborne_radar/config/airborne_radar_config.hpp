/**
 * @file airborne_radar_config.hpp
 * @brief 机载雷达模块配置统一入口。
 * @note 当调用方只需要公开配置 API 时，优先包含本头而不是逐个引入 `config/` 子头。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AIRBORNE_RADAR_CONFIG_HPP_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AIRBORNE_RADAR_CONFIG_HPP_

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArMissionConfig.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRecognitionConfig.h"
#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfigValidation.h"

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AIRBORNE_RADAR_CONFIG_HPP_
