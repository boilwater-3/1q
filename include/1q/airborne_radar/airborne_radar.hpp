/**
 * @file airborne_radar.hpp
 * @brief 机载雷达模块对外统一入口。
 * @note 当调用方需要稳定会话、输入输出、环境与配置 API 时，优先包含本头；
 *       trace/replay 工具头按需单独包含。
 */

#ifndef ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_
#define ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_

#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArExternalInputAdapter.h"
#include "1q/airborne_radar/session/ArExternalOutputAdapter.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTrackOutput.h"

#endif  // ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_
