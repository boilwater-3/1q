/**
 * @file api.hpp
 * @brief 1Q 库的公共导出宏、版本信息与快速入门 Umbrella Include。
 *
 * 快速入门路径（包含所有常用公共 API）：
 * @code
 * #include "1q/api.hpp"
 * @endcode
 */

#pragma once

// ---------------------------------------------------------------------------
// 版本常量
// ---------------------------------------------------------------------------
/** @brief 主版本号 */
#define ONEQ_VERSION_MAJOR 1
/** @brief 次版本号 */
#define ONEQ_VERSION_MINOR 0
/** @brief 修订版本号 */
#define ONEQ_VERSION_PATCH 0

// ---------------------------------------------------------------------------
// 导出宏（ONEQ_API）
// ---------------------------------------------------------------------------
#ifndef ONEQ_API
#ifdef _WIN32
#if defined(ONEQ_LIBRARY_BUILD)
#define ONEQ_API __declspec(dllexport)
#elif defined(ONEQ_STATIC)
#define ONEQ_API
#elif defined(ONEQ_LIBRARY_SHARED)
#define ONEQ_API __declspec(dllimport)
#else
#define ONEQ_API __declspec(dllimport)
#endif
#else
#ifdef ONEQ_LIBRARY_BUILD
#define ONEQ_API __attribute__((visibility("default")))
#else
#define ONEQ_API
#endif
#endif
#endif

// 向后兼容别名，保持已有代码不报错
#ifndef TEMPLATE_API
#define TEMPLATE_API ONEQ_API
#endif

// ---------------------------------------------------------------------------
// Umbrella Include（快速入门路径）
// ---------------------------------------------------------------------------
#include "1q/airborne_radar/common/TargetFeatureBuilder.h"
#include "1q/airborne_radar/common/TargetFeatureUtils.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/output/TrackOutputQueries.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"