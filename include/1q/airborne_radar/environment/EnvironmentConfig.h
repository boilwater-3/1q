/**
 * @file EnvironmentConfig.h
 * @brief 定义环境模型配置相关的公开类型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/DecisionSourceInfo.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief JammingTechnique 与 `model::JammingTechnique` 保持统一的别名。
 */
using JammingTechnique = model::JammingTechnique;

/**
 * @brief JammerEmitterState 表示对外注入的干扰源场景事实输入。
 * @note 该结构仅承载外部场景事实，不暴露运行期派生量。
 */
struct JammerEmitterState {
  JammerEmitterState() = default;

  JammingTechnique technique{JammingTechnique::kUnknown}; /**< 干扰技术类型 */
  float power_db{0.0f};                                   /**< 干扰功率估计（单位：dB） */
  float js_db{0.0f};                                      /**< 干扰与信号比估计（单位：dB） */
  bool has_direction_deg{false};       /**< 是否提供干扰来向方位/俯仰角 */
  float azimuth_deg{0.0f};             /**< 干扰来向方位角（单位：deg，可选） */
  float elevation_deg{0.0f};           /**< 干扰来向俯仰角（单位：deg，可选） */
  float angular_span_deg{0.0f};        /**< 干扰角域宽度（单位：deg） */
  float confidence{1.0f};              /**< 干扰事实置信度，范围 [0, 1] */
};

/** @brief 场景中的干扰源输入列表 */
using JammerEmitterStateList = std::vector<JammerEmitterState>;

/**
 * @brief JammerDirectionDeg 表示干扰来向角。
 */
struct JammerDirectionDeg {
  float azimuth_deg{0.0f};   /**< 干扰来向方位角（单位：deg） */
  float elevation_deg{0.0f}; /**< 干扰来向俯仰角（单位：deg） */
};

/**
 * @brief JammerSourceFact 表示单周期冻结后的干扰事实输出。
 * @note 其中 overlap/prf/sidelobe 等量由库内基于场景事实与运行状态派生。
 */
struct JammerSourceFact {
  JammingTechnique technique{JammingTechnique::kUnknown}; /**< 干扰技术类型 */
  float power_db{0.0f};                                   /**< 干扰功率估计（单位：dB） */
  float js_db{0.0f};                                      /**< 干扰与信号比估计（单位：dB） */
  float frequency_overlap_ratio{0.0f}; /**< 干扰与当前工作频率的重叠度，范围 [0, 1] */
  float prf_lock_risk{0.0f};           /**< 干扰对当前 PRF 锁定的风险度，范围 [0, 1] */
  bool has_direction_deg{false};       /**< 是否具有可用的干扰来向方位/俯仰角 */
  JammerDirectionDeg direction_deg{};  /**< 干扰来向角（仅在 has_direction_deg=true 时有效） */
  float angular_span_deg{0.0f};        /**< 干扰角域宽度（单位：deg） */
  bool in_sidelobe{false};             /**< 干扰是否主要经由旁瓣进入 */
  float confidence{1.0f};              /**< 干扰事实置信度，范围 [0, 1] */
};

/** @brief 单周期内可见的干扰源列表 */
using JammerSourceFactList = std::vector<JammerSourceFact>;

/**
 * @brief AtmosphericPhysicsConfig 描述可选的大气传播物理参数。
 */
struct AtmosphericPhysicsConfig {
  bool enable_physical_model{false}; /**< 是否启用物理传播模型 */
  float pressure_hpa{1013.25f};      /**< 气压（单位：hPa） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float relative_humidity{0.5f};     /**< 相对湿度 [0, 1] */
  /** @note 该结构仅承载基础气象量；时间/空间天气高级量见 AtmosphericDerivedContext。 */
};

/**
 * @brief AtmosphericDerivedContext 描述由高层时间/空间天气输入解析得到的大气高级上下文。
 * @note 这些量依然来源于库边界外部输入，但不与基础气象量混放，避免普通用户误填。
 */
struct AtmosphericDerivedContext {
  float k_factor{4.0f / 3.0f};    /**< 地球有效半径因子 */
  std::int32_t day_of_year{172};  /**< 年积日 [1, 366] */
  float solar_flux_f107a{150.0f}; /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};  /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};     /**< 地磁活动指数 */
  /**
   * @note 这些量仍然由库边界外部输入提供，但作为高层时间/空间天气上下文进入，
   * 不与普通用户直接填写的基础气象量混在同一个配置组里。
   */
};

/**
 * @brief VegetationScatterPhysicsConfig 描述可选植被散射杂波参数。
 */
struct VegetationScatterPhysicsConfig {
  bool enable_physical_model{false}; /**< 是否启用植被散射物理建模 */
  float leaf_size_m{0.05f};          /**< 叶片等效尺度（单位：m） */
  float dielectric_constant_real{2.5f}; /**< 植被等效介电常数实部 */
  std::uint32_t leaf_count{64U};     /**< 叶片数量 */
  float canopy_radius_m{1.2f};       /**< 冠层半径（单位：m） */
  float canopy_height_m{3.5f};       /**< 冠层高度（单位：m） */
  /** @note 植被散射内部标定项仍保留在库实现中，不作为公开输入。 */
};

/**
 * @brief EnvironmentModelConfig 描述环境模型参数。
 */
struct EnvironmentModelConfig {
  AtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选物理传播参数 */
  AtmosphericDerivedContext atmospheric_context{}; /**< 高层时间/空间天气上下文 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 可选植被散射参数 */
  JammerEmitterStateList jammer_sources{};  /**< 多源干扰事实输入 */
};

/**
 * @brief EnvironmentScenarioConfig 描述对外场景输入（不暴露内部传播/杂波调参项）。
 */
struct EnvironmentScenarioConfig {
  AtmosphericPhysicsConfig atmospheric_physics{}; /**< 场景气象/电离层输入 */
  AtmosphericDerivedContext atmospheric_context{}; /**< 场景时间/空间天气输入 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 场景植被散射输入 */
  JammerEmitterStateList jammer_sources{}; /**< 场景干扰事实输入 */
};

/**
 * @brief EnvironmentDefaultConfig 描述初始化阶段的默认环境配置。
 */
struct EnvironmentDefaultConfig {
  EnvironmentScenarioConfig scenario_config{};  /**< 默认环境场景输入 */
  float jamming_detection_threshold_db{6.0f};  /**< 默认干扰判定阈值（单位：dB） */
};

/**
 * @brief 将对外场景输入映射为内部环境模型配置。
 */
inline EnvironmentModelConfig BuildModelConfigFromScenario(
    const EnvironmentScenarioConfig& scenario_config) {
  EnvironmentModelConfig model_config;
  model_config.atmospheric_physics = scenario_config.atmospheric_physics;
  model_config.atmospheric_context = scenario_config.atmospheric_context;
  model_config.vegetation_scatter_physics = scenario_config.vegetation_scatter_physics;
  model_config.jammer_sources = scenario_config.jammer_sources;
  return model_config;
}

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_
