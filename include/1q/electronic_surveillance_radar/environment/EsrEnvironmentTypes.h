/**
 * @file EsrEnvironmentTypes.h
 * @brief 定义电子侦察环境层公共输入与快照类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_TYPES_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrJammingTechnique 表示干扰技术类型。
 */
enum class ONEQ_API EsrJammingTechnique {
  kUnknown = 0,      /**< 未知或未分类干扰 */
  kNoiseSuppression, /**< 压制式/噪声式干扰 */
  kDeception,        /**< 欺骗式干扰 */
  kMixed             /**< 压制与欺骗并存 */
};

/**
 * @brief EsrJammerSource 描述场景中的单个干扰源输入。
 */
struct ONEQ_API EsrJammerSource {
  EsrJammingTechnique technique{EsrJammingTechnique::kUnknown}; /**< 干扰技术类型 */
  bool active{false};                                           /**< 干扰源是否激活 */
  double center_hz{0.0};                                        /**< 干扰中心频率（单位：Hz） */
  double bandwidth_hz{0.0};                                     /**< 干扰带宽（单位：Hz） */
  float power_w{0.0f};                                          /**< 干扰功率（单位：W） */
  float deception_risk{0.0f}; /**< 外部情报给出的欺骗可能性提示，范围 [0, 1] */
  float confidence{1.0f};                                       /**< 干扰置信度，范围 [0, 1] */
};

/** @brief EsrJammerSourceList 表示干扰源列表。 */
using EsrJammerSourceList = std::vector<EsrJammerSource>;

/**
 * @brief EsrPropagationEnvironmentProfile 描述高层传播环境类型。
 */
enum class ONEQ_API EsrPropagationEnvironmentProfile {
  kOpen = 0,      /**< 开阔环境 */
  kTypical = 1,   /**< 常规环境 */
  kComplex = 2    /**< 复杂地形/城市环境 */
};

/**
 * @brief EsrClutterDensityLevel 描述高层杂波密度级别。
 */
enum class ONEQ_API EsrClutterDensityLevel {
  kLow = 0,     /**< 低杂波 */
  kMedium = 1,  /**< 中等杂波 */
  kHigh = 2     /**< 高杂波 */
};

/**
 * @brief EsrAtmosphericObservation 描述外部可观测天气事实。
 */
struct ONEQ_API EsrAtmosphericObservation {
  float relative_humidity_ratio{0.5f}; /**< 相对湿度，范围 [0, 1] */
  float precipitation_rate_mmph{0.0f}; /**< 降水强度（单位：mm/h） */
  float visibility_km{20.0f};          /**< 能见度（单位：km） */
};

/**
 * @brief EsrEnvironmentObservation 描述待冻结环境高层观测输入。
 */
struct ONEQ_API EsrEnvironmentObservation {
  EsrPropagationEnvironmentProfile propagation_profile{
      EsrPropagationEnvironmentProfile::kTypical}; /**< 高层传播环境类型 */
  EsrClutterDensityLevel clutter_density{EsrClutterDensityLevel::kMedium}; /**< 杂波密度级别 */
  float spectrum_occupancy_ratio{0.0f};   /**< 频谱占用率，范围 [0, 1] */
  EsrAtmosphericObservation atmospheric_observation{}; /**< 外部天气观测输入 */
  EsrJammerSourceList jammer_sources{};                /**< 场景干扰源列表 */
};

/**
 * @brief EsrEnvironmentCycleContext 描述单周期冻结上下文。
 */
struct ONEQ_API EsrEnvironmentCycleContext {
  std::uint32_t cycle_index{0U};                       /**< 当前周期号 */
  float dt_sec{0.0f};                                  /**< 当前周期步长（单位：s） */
  EsrEnvironmentObservation observation{}; /**< 当前周期待冻结环境观测 */
};

/**
 * @brief EsrEnvironmentSnapshot 描述单周期环境快照。
 */
struct ONEQ_API EsrEnvironmentSnapshot {
  std::uint32_t cycle_index{0U};        /**< 当前周期号 */
  float dt_sec{0.0f};                   /**< 当前周期步长（单位：s） */
  float propagation_loss_db{0.0f};      /**< 聚合传播损耗（单位：dB） */
  float clutter_noise_w{0.0f};          /**< 杂波噪声功率（单位：W） */
  float suppression_power_w{0.0f};      /**< 聚合压制干扰功率（单位：W） */
  float deception_risk{0.0f};           /**< 聚合欺骗风险度，范围 [0, 1] */
  float spectrum_occupancy_ratio{0.0f}; /**< 频谱占用率，范围 [0, 1] */
  bool jamming_detected{false};         /**< 是否检测到显著干扰 */
  EsrJammerSourceList jammer_sources{}; /**< 当前周期可见干扰源 */
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_TYPES_H_
