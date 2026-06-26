/**
 * @file airborne_radar.hpp
 * @brief 机载雷达模块对外统一入口。
 * @note 当调用方需要稳定会话、输入输出、环境与配置 API 时，优先包含本头；
 *       trace/replay 工具头按需单独包含。
 */

#ifndef ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_
#define ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_

#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "1q/airborne_radar/model/JammingSemantics.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/model/TargetCategory.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleInputBuilder.h"
#include "1q/airborne_radar/session/RadarCycleOutputBuilder.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarEnvironmentInput.h"
#include "1q/airborne_radar/session/RadarEnvironmentInputPatch.h"
#include "1q/airborne_radar/session/RadarEnvironmentInputState.h"
#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/airborne_radar/session/RadarExternalOutputAdapter.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

#endif  // ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_
