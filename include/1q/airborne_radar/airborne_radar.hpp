/**
 * @file airborne_radar.hpp
 * @brief 机载雷达模块对外统一入口。
 * @note 当调用方需要会话、输入输出与配置的完整公开 API 时，优先包含本头。
 */

#ifndef ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_
#define ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/environment/airborne_radar_environment.hpp"
#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "1q/airborne_radar/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/model/JammingSemantics.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/model/TargetCategory.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/model/TargetFeatureBuilder.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/output/TrackOutputQueries.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarExternalInputAdapter.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"

#endif  // ONEQ_AIRBORNE_RADAR_AIRBORNE_RADAR_HPP_
