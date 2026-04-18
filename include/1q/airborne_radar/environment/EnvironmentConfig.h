/**
 * @file EnvironmentConfig.h
 * @brief 定义环境模型配置相关的公开类型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "1q/foundation/atmospheric_types.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief JammingSensitivityProfile 描述干扰判定灵敏度语义档位。
 */
enum class JammingSensitivityProfile {
  kRelaxed = 0,  /**< 更保守，不易判定为干扰 */
  kBalanced = 1, /**< 平衡策略 */
  kStrict = 2    /**< 更敏感，容易判定为干扰 */
};

/**
 * @brief 将语义化干扰灵敏度档位映射为内部 dB 阈值。
 */
inline float ResolveJammingDetectionThresholdDb(JammingSensitivityProfile profile) {
  switch (profile) {
    case JammingSensitivityProfile::kRelaxed:
      return 8.0f;
    case JammingSensitivityProfile::kStrict:
      return 4.0f;
    case JammingSensitivityProfile::kBalanced:
    default:
      return 6.0f;
  }
}

/**
 * @brief 将 dB 阈值近似映射为语义化干扰灵敏度档位。
 */
inline JammingSensitivityProfile ResolveJammingSensitivityProfile(float threshold_db) {
  if (threshold_db <= 5.0f) {
    return JammingSensitivityProfile::kStrict;
  }
  if (threshold_db >= 7.0f) {
    return JammingSensitivityProfile::kRelaxed;
  }
  return JammingSensitivityProfile::kBalanced;
}

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

/** @brief AtmosphericPhysicsConfig 复用 foundation 层统一基础气象观测类型。 */
using AtmosphericPhysicsConfig = oneq::foundation::AtmosphericObservation;

/**
 * @brief AtmosphericDerivedContext 描述 AR 的高层时间/空间天气上下文输入。
 * @note k_factor/day_of_year 由库内从基础观测与时间语义自动推导，不作为对外输入字段。
 */
struct AtmosphericDerivedContext {
  bool has_simulation_unix_seconds{false}; /**< 是否显式提供仿真 UTC 秒级时间戳 */
  std::int64_t simulation_unix_seconds{0};  /**< 仿真 UTC 秒级时间戳（Unix epoch） */
  float solar_flux_f107a{150.0f};           /**< 平滑太阳流量指数 */
  float solar_flux_f107{150.0f};            /**< 当日太阳流量指数 */
  float geomagnetic_ap{4.0f};               /**< 地磁活动指数 */
};

namespace internal {

inline std::int64_t FloorDiv(std::int64_t numerator, std::int64_t denominator) {
  const std::int64_t quotient = numerator / denominator;
  const std::int64_t remainder = numerator % denominator;
  if (remainder != 0 && ((remainder > 0) != (denominator > 0))) {
    return quotient - 1;
  }
  return quotient;
}

inline std::int32_t ResolveDayOfYearFromUnixSeconds(std::int64_t unix_seconds) {
  // Howard Hinnant civil calendar conversion: days since 1970-01-01 -> (year, month, day)
  const std::int64_t days_since_epoch = FloorDiv(unix_seconds, 86400);
  std::int64_t z = days_since_epoch + 719468;
  const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const std::int64_t doe = z - era * 146097;
  const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const std::int64_t mp = (5 * doy + 2) / 153;
  const std::int32_t day = static_cast<std::int32_t>(doy - (153 * mp + 2) / 5 + 1);
  const std::int32_t month = static_cast<std::int32_t>(mp + (mp < 10 ? 3 : -9));
  const std::int32_t year = static_cast<std::int32_t>(yoe + era * 400 + (month <= 2 ? 1 : 0));

  const bool is_leap_year = (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
  static const std::int32_t cumulative_days_before_month[12] = {
      0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  const std::int32_t base = cumulative_days_before_month[month - 1];
  const std::int32_t leap_day = (is_leap_year && month > 2) ? 1 : 0;
  return base + day + leap_day;
}

}  // namespace internal

/**
 * @brief 推导环境上下文中的有效 k_factor（由基础观测自动推导）。
 */
inline float ResolveEffectiveKFactor(const AtmosphericDerivedContext& context,
                                     const AtmosphericPhysicsConfig& physics) {
  (void)context;

  // 基于地面气象观测估计近地层折射率梯度，映射到有效地球半径因子 k。
  const float temperature_k = (physics.temperature_k > 1.0f) ? physics.temperature_k : 288.15f;
  const float pressure_hpa = (physics.pressure_hpa > 0.0f) ? physics.pressure_hpa : 1013.25f;
  const float relative_humidity = physics.relative_humidity < 0.0f
                                      ? 0.0f
                                      : (physics.relative_humidity > 1.0f ? 1.0f
                                                                          : physics.relative_humidity);

  const float temperature_c = temperature_k - 273.15f;
  const float saturation_vapor_pressure_hpa =
      6.1121f * std::exp((17.502f * temperature_c) / (240.97f + temperature_c));
  const float water_vapor_pressure_hpa = relative_humidity * saturation_vapor_pressure_hpa;

  // 表面无线电折射率 N（N-units）。
  const float refractivity_n = 77.6f * (pressure_hpa / temperature_k) +
                               3.73e5f * (water_vapor_pressure_hpa /
                                          (temperature_k * temperature_k));

  // 假设近地层折射率沿高度近似指数衰减：N(h)=N0*exp(-h/H)，H 取典型对流层尺度高度。
  const float refractivity_scale_height_km = 7.35f;
  const float dndh_n_per_km = -refractivity_n / refractivity_scale_height_km;

  // k = 1 / (1 + (dN/dh)/157)，其中 dN/dh 单位为 N/km。
  const float denominator = 1.0f + dndh_n_per_km / 157.0f;
  if (denominator <= 0.1f) {
    return 4.0f / 3.0f;
  }
  const float derived_k_factor = 1.0f / denominator;
  if (derived_k_factor < 0.5f || derived_k_factor > 2.5f) {
    return 4.0f / 3.0f;
  }
  return derived_k_factor;
}

/**
 * @brief 推导环境上下文中的有效 day_of_year（由仿真时间自动推导）。
 */
inline std::int32_t ResolveEffectiveDayOfYear(const AtmosphericDerivedContext& context) {
  if (!context.has_simulation_unix_seconds) {
    return 172;
  }
  return internal::ResolveDayOfYearFromUnixSeconds(context.simulation_unix_seconds);
}

/**
 * @brief VegetationCoverProfile 描述植被覆盖语义档位。
 */
enum class VegetationCoverProfile {
  kDisabled = 0,      /**< 无植被散射 */
  kOpenGrassland,     /**< 开阔草地 */
  kSparseWoodland,    /**< 稀疏林地 */
  kDeciduousForest,   /**< 落叶林 */
  kConiferousForest,  /**< 针叶林 */
  kTropicalDense      /**< 热带密林 */
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
  AtmosphericPhysicsConfig atmospheric_physics{}; /**< 场景气象/电离层输入 */
  AtmosphericDerivedContext atmospheric_context{}; /**< 场景时间/空间天气输入 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 场景植被散射输入 */
  JammerEmitterStateList jammer_sources{}; /**< 场景干扰事实输入 */
};

/** @brief EnvironmentModelConfig 与 EnvironmentScenarioConfig 统一。 */
using EnvironmentModelConfig = EnvironmentScenarioConfig;

/**
 * @brief EnvironmentDefaultConfig 描述初始化阶段的默认环境配置。
 */
struct EnvironmentDefaultConfig {
  EnvironmentScenarioConfig scenario_config{};  /**< 默认环境场景输入 */
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
