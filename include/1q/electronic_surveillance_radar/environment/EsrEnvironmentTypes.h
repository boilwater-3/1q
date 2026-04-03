/**
 * @file EsrEnvironmentTypes.h
 * @brief 定义电子侦察环境层公共输入、快照与配置类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_TYPES_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrJammingTechnique 表示干扰技术类型。
 */
enum class EsrJammingTechnique {
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
  float deception_risk{0.0f};                                   /**< 欺骗风险度，范围 [0, 1] */
  float confidence{1.0f};                                       /**< 干扰置信度，范围 [0, 1] */
};

/** @brief EsrJammerSourceList 表示干扰源列表。 */
using EsrJammerSourceList = std::vector<EsrJammerSource>;

/**
 * @brief EsrAtmosphericPhysicsConfig 描述可选物理传播参数。
 */
struct ONEQ_API EsrAtmosphericPhysicsConfig {
  bool enable_physical_model{false}; /**< 是否启用物理传播模型 */
  float frequency_hz{10.0e9f};       /**< 雷达频率（单位：Hz） */
  float path_length_m{10.0e3f};      /**< 传播路径长度（单位：m） */
  float radar_altitude_m{1.0e3f};    /**< 雷达高度（单位：m） */
  float target_altitude_m{1.0e3f};   /**< 目标高度（单位：m） */
  float elevation_deg{5.0f};         /**< 传播仰角（单位：deg） */
  float pressure_hpa{1013.25f};      /**< 气压（单位：hPa） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float relative_humidity{0.5f};     /**< 相对湿度 [0, 1] */
  float k_factor{4.0f / 3.0f};       /**< 地球有效半径因子 */
  std::int32_t day_of_year{172};     /**< 年积日 [1, 366] */
  float solar_flux_f107a{150.0f};    /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};     /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};        /**< 地磁活动指数 */
};

/**
 * @brief EsrEnvironmentSceneState 描述待冻结环境场景。
 */
struct ONEQ_API EsrEnvironmentSceneState {
  float base_propagation_loss_db{3.0f};   /**< 基础传播损耗（单位：dB） */
  float atmospheric_attenuation_db{1.0f}; /**< 大气附加衰减（单位：dB） */
  float terrain_reflection_db{0.0f};      /**< 地形/多径附加项（单位：dB） */
  float clutter_noise_w{1.0e-12f};        /**< 杂波噪声功率（单位：W） */
  float spectrum_occupancy_ratio{0.0f};   /**< 频谱占用率，范围 [0, 1] */
  EsrAtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选物理传播参数 */
  EsrJammerSourceList jammer_sources{};   /**< 场景干扰源列表 */
};

/**
 * @brief EsrEnvironmentCycleContext 描述单周期冻结上下文。
 */
struct ONEQ_API EsrEnvironmentCycleContext {
  std::uint32_t cycle_index{0U};          /**< 当前周期号 */
  float dt_sec{0.0f};                     /**< 当前周期步长（单位：s） */
  EsrEnvironmentSceneState scene_state{}; /**< 当前周期待冻结场景 */
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
  float jammer_power_w{0.0f};           /**< 兼容字段：等价于 suppression_power_w（单位：W） */
  float deception_risk{0.0f};           /**< 聚合欺骗风险度，范围 [0, 1] */
  float spectrum_occupancy_ratio{0.0f}; /**< 频谱占用率，范围 [0, 1] */
  bool jamming_detected{false};         /**< 是否检测到显著干扰 */
  EsrJammerSourceList jammer_sources{}; /**< 当前周期可见干扰源 */
};

/**
 * @brief EsrEnvironmentModelConfig 描述环境模型配置。
 */
struct ONEQ_API EsrEnvironmentModelConfig {
  float default_clutter_noise_w{1.0e-12f};      /**< 默认杂波噪声功率（单位：W） */
  float jamming_detection_threshold_w{1.0e-9f}; /**< 干扰检测阈值（单位：W） */
  EsrAtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选物理传播参数 */
};

/**
 * @brief EsrEnvironmentDefaultConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EsrEnvironmentDefaultConfig {
  EsrEnvironmentModelConfig model_config{}; /**< 默认环境模型配置 */
};

/**
 * @brief EsrEnvironmentRuntimeConfigPatch 描述运行期可变环境补丁。
 */
struct ONEQ_API EsrEnvironmentRuntimeConfigPatch {
  bool has_model_config{false};                  /**< 是否更新环境模型配置 */
  EsrEnvironmentModelConfig model_config{};      /**< 运行期环境模型配置 */
  bool has_jamming_detection_threshold_w{false}; /**< 是否更新干扰检测阈值 */
  float jamming_detection_threshold_w{1.0e-9f};  /**< 干扰检测阈值（单位：W） */
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_TYPES_H_
