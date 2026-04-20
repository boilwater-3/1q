/**
 * @file EnvironmentConfig.h
 * @brief 定义环境模型配置相关的公开类型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "1q/foundation/atmospheric_types.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief 干扰判定灵敏度档位。
 *
 * 控制环境服务将接收功率判定为"干扰存在"的功率门限（dB）。
 * 档位越严格，门限越低，越容易触发干扰响应；
 * 档位越宽松，门限越高，需要更强的干扰功率才会触发。
 */
enum class JammingSensitivityProfile {
  kRelaxed = 0,  /**< 宽松——门限 8 dB，仅强干扰触发响应，减少虚警。 */
  kBalanced = 1, /**< 均衡——门限 6 dB，兼顾检测灵敏度与虚警率。 */
  kStrict = 2    /**< 严格——门限 4 dB，低功率干扰即可触发，适合高威胁场景。 */
};

/**
 * @brief 将 dB 阈值近似映射为语义化干扰灵敏度档位。
 *
 * 反向映射：threshold_db <= 5 → kStrict，>= 7 → kRelaxed，其余 → kBalanced。
 *
 * @param threshold_db 干扰功率判定门限（dB）。
 * @return 对应的灵敏度档位。
 */
JammingSensitivityProfile ResolveJammingSensitivityProfile(float threshold_db);

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
  bool has_direction_deg{false};                          /**< 是否提供干扰来向方位/俯仰角 */
  float azimuth_deg{0.0f};                                /**< 干扰来向方位角（单位：deg，可选） */
  float elevation_deg{0.0f};                              /**< 干扰来向俯仰角（单位：deg，可选） */
  float angular_span_deg{0.0f};                           /**< 干扰角域宽度（单位：deg） */
  float confidence{1.0f};                                 /**< 干扰事实置信度，范围 [0, 1] */
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

/** @brief AtmosphericPhysicsConfig 复用 foundation 层统一基础气象观测类型。 */
using AtmosphericPhysicsConfig = oneq::foundation::AtmosphericObservation;

/**
 * @brief AtmosphericDerivedContext 描述 AR 的高层时间/空间天气上下文输入。
 * @note k_factor/day_of_year 由库内从基础观测与时间语义自动推导，不作为对外输入字段。
 */
struct AtmosphericDerivedContext {
  bool has_simulation_unix_seconds{false}; /**< 是否显式提供仿真 UTC 秒级时间戳 */
  std::int64_t simulation_unix_seconds{0}; /**< 仿真 UTC 秒级时间戳（Unix epoch） */
  float solar_flux_f107a{150.0f};          /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};           /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};              /**< 地磁活动指数 */
};

/**
 * @brief 推导环境上下文中的有效 k_factor（由基础观测自动推导）。
 *
 * 基于地面气象观测估计近地层折射率梯度，映射到有效地球半径因子 k。
 * 若推导结果超出合理范围 [0.5, 2.5]，回退到标准 4/3 近似。
 *
 * @param context 时间/空间天气上下文（当前未使用，保留扩展）。
 * @param physics 基础气象观测输入。
 * @return 有效地球半径因子 k。
 */
float ResolveEffectiveKFactor(const AtmosphericDerivedContext& context,
                              const AtmosphericPhysicsConfig& physics);

/**
 * @brief 推导环境上下文中的有效 day_of_year（由仿真时间自动推导）。
 *
 * 若未提供仿真时间戳，回退到默认值 172（夏至附近）。
 *
 * @param context 时间/空间天气上下文。
 * @return 年积日 [1, 366]。
 */
std::int32_t ResolveEffectiveDayOfYear(const AtmosphericDerivedContext& context);

/**
 * @brief 地表植被覆盖档位。
 *
 * 选择档位后自动填写叶片尺寸、介电常数、叶片密度、
 * 冠层半径和冠层高度等植被散射物理参数，
 * 影响近地传播路径上的多径散射和杂波估计。
 */
enum class VegetationCoverProfile {
  kDisabled = 0,     /**< 不建模植被散射。 */
  kOpenGrassland,    /**< 开阔草地——低矮冠层（0.8 m），少量小叶片。 */
  kSparseWoodland,   /**< 稀疏林地——中等冠层（3 m），中等密度散射体。 */
  kDeciduousForest,  /**< 落叶林——高大冠层（6 m），高密度大叶片。 */
  kConiferousForest, /**< 针叶林——高冠层（8 m），密集细小针叶散射体。 */
  kTropicalDense     /**< 热带密林——高冠层（9 m），最高密度与介电常数。 */
};

/**
 * @brief VegetationScatterPhysicsConfig 描述可选植被散射杂波参数。
 */
struct VegetationScatterPhysicsConfig {
  VegetationCoverProfile cover_profile{VegetationCoverProfile::kDisabled}; /**< 植被覆盖语义档位 */
  bool enable_physical_model{false}; /**< 是否启用植被散射物理建模 */
};

/**
 * @brief EnvironmentScenarioConfig 描述对外场景输入（不暴露内部传播/杂波调参项）。
 */
struct EnvironmentScenarioConfig {
  AtmosphericPhysicsConfig atmospheric_physics{};              /**< 场景气象/电离层输入 */
  AtmosphericDerivedContext atmospheric_context{};             /**< 场景时间/空间天气输入 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 场景植被散射输入 */
  JammerEmitterStateList jammer_sources{};                     /**< 场景干扰事实输入 */
};

/** @brief EnvironmentModelConfig 与 EnvironmentScenarioConfig 统一。 */
using EnvironmentModelConfig = EnvironmentScenarioConfig;

/**
 * @brief EnvironmentDefaultConfig 描述初始化阶段的默认环境配置。
 */
struct EnvironmentDefaultConfig {
  EnvironmentScenarioConfig scenario_config{}; /**< 默认环境场景输入 */
  JammingSensitivityProfile jamming_sensitivity_profile{
      JammingSensitivityProfile::kBalanced}; /**< 默认干扰判定灵敏度语义档位 */
};

/**
 * @brief 将对外场景输入映射为内部环境模型配置。
 */
inline EnvironmentModelConfig BuildModelConfigFromScenario(
    const EnvironmentScenarioConfig& scenario_config) {
  return scenario_config;
}

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_
