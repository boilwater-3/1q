/**
 * @file EnvironmentConfig.h
 * @brief 定义环境模型配置相关的公开类型。
 *
 * @par 迁移模板
 * 本文件作为 EOS/ESR 环境配置迁移的标准模板。迁移步骤：
 * 1. 复制 ScenarioConfig -> ModelConfig (alias) -> DefaultConfig 层次结构；
 * 2. 替换领域特定类型（如 JammerEmitterState -> 对应域场景输入类型）；
 * 3. 实现对应领域的 BuildModelConfigFromScenario 映射函数；
 * 4. 实现 RuntimeConfigPatch 及其 Builder，遵循 has_* 布尔标志模式；
 * 5. 参考 ar_environment_config_contract_test.cpp 建立等价测试覆盖。
 */

#ifndef ONEQ_AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
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
enum class ONEQ_API JammingSensitivityProfile {
  kRelaxed = 0,  /**< 宽松——门限 8 dB，仅强干扰触发响应，减少虚警。 */
  kBalanced = 1, /**< 均衡——门限 6 dB，兼顾检测灵敏度与虚警率。 */
  kStrict = 2    /**< 严格——门限 4 dB，低功率干扰即可触发，适合高威胁场景。 */
};

/**
 * @brief 将 dB 阈值近似映射为语义化干扰灵敏度档位。
 *
 * @par 解析合约
 * - 输入：threshold_db（干扰功率判定门限，单位 dB）。
 * - 输出：JammingSensitivityProfile 语义档位。
 * - 边界规则：threshold_db <= 5.0 → kStrict；5.0 < threshold_db < 7.0 → kBalanced；
 *   threshold_db >= 7.0 → kRelaxed。
 * - 恰好在 5.0 归入 kStrict（<= 含等号），恰好在 7.0 归入 kRelaxed（>= 含等号）。
 * - 无异常抛出；任何浮点值均可安全映射。
 *
 * @param[in] threshold_db 干扰功率判定门限（dB）。
 * @return 对应的灵敏度档位。
 */
ONEQ_API JammingSensitivityProfile ResolveJammingSensitivityProfile(float threshold_db);

/**
 * @brief JammingTechnique 与 `model::JammingTechnique` 保持统一的别名。
 */
using JammingTechnique = model::JammingTechnique;

/**
 * @brief JammerEmitterState 表示对外注入的干扰源场景事实输入。
 *
 * @par 类型合约
 * - 该结构仅承载外部场景事实，不暴露运行期派生量。
 * - 运行期派生量（如 frequency_overlap_ratio、prf_lock_risk、in_sidelobe）
 *   由库内 EnvironmentService 从场景事实与运行状态自动推导。
 * - 字段值范围约束（由库内钳位处理）：
 *   power_db/js_db/angular_span_deg 负值钳位到 0；
 *   confidence 超出 [0, 1] 钳位到区间端点。
 */
struct ONEQ_API JammerEmitterState {
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

/** @brief AtmosphericPhysicsConfig 复用 foundation 层统一基础气象观测类型。 */
using AtmosphericPhysicsConfig = oneq::foundation::AtmosphericObservation;

/**
 * @brief AtmosphericDerivedContext 描述 AR 的高层时间/空间天气上下文输入。
 * @note k_factor/day_of_year 由库内从基础观测与时间语义自动推导，不作为对外输入字段。
 */
struct ONEQ_API AtmosphericDerivedContext {
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
 *
 * @par 解析合约
 * - 输入：AtmosphericDerivedContext（当前未使用，保留扩展）+ AtmosphericPhysicsConfig（基础气象观测）。
 * - 输出：有效地球半径因子 k，物理合理范围 (0.5, 2.5)。
 * - Fallback 语义：
 *   1. temperature_k <= 1 K 或 pressure_hpa <= 0：使用 ISA 标准值替代（288.15 K / 1013.25 hPa）。
 *   2. relative_humidity 负值钳位到 0，超出 1.0 钳位到 1.0。
 *   3. 折射率分母 <= 0.1（极端超折射陷阱条件）：回退到 4/3。
 *   4. 推导结果 < 0.5 或 > 2.5：回退到标准 4/3 近似（~1.333）。
 * - 无异常抛出；所有非法输入均通过 fallback 或钳位处理。
 *
 * @param[in] context 时间/空间天气上下文（当前未使用，保留扩展）。
 * @param[in] physics 基础气象观测输入。
 * @return 有效地球半径因子 k。
 */
ONEQ_API float ResolveEffectiveKFactor(const AtmosphericDerivedContext& context,
                              const AtmosphericPhysicsConfig& physics);

/**
 * @brief 推导环境上下文中的有效 day_of_year（由仿真时间自动推导）。
 *
 * @par 解析合约
 * - 输入：AtmosphericDerivedContext（需 has_simulation_unix_seconds == true 方使用仿真时间）。
 * - 输出：年积日 [1, 366]，由仿真 UTC 秒级时间戳推导。
 * - Fallback 语义：若 has_simulation_unix_seconds == false，回退到 172（夏至附近）。
 * - 无异常抛出。
 *
 * @param[in] context 时间/空间天气上下文。
 * @return 年积日 [1, 366]。
 */
ONEQ_API std::int32_t ResolveEffectiveDayOfYear(const AtmosphericDerivedContext& context);

/**
 * @brief 地表植被覆盖档位。
 *
 * 选择档位后自动填写叶片尺寸、介电常数、叶片密度、
 * 冠层半径和冠层高度等植被散射物理参数，
 * 影响近地传播路径上的多径散射和杂波估计。
 */
enum class ONEQ_API VegetationCoverProfile {
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
struct ONEQ_API VegetationScatterPhysicsConfig {
  VegetationCoverProfile cover_profile{VegetationCoverProfile::kDisabled}; /**< 植被覆盖语义档位 */
  bool enable_physical_model{false}; /**< 是否启用植被散射物理建模 */
};

/**
 * @brief EnvironmentScenarioConfig 描述对外场景输入（不暴露内部传播/杂波调参项）。
 *
 * @par 类型合约
 * - 仅承载外部输入事实（气象观测、时间/空间天气、植被场景、干扰源事实）。
 * - 不包含内部算法调参字段（如传播损耗系数、杂波增益）。
 * - 不包含运行期派生量（如 effective_k_factor、effective_day_of_year）。
 * - 新增字段须为外部可观测或可注入的场景事实，不得引入内部调参项。
 */
struct ONEQ_API EnvironmentScenarioConfig {
  AtmosphericPhysicsConfig atmospheric_physics{};              /**< 场景气象/电离层输入 */
  AtmosphericDerivedContext atmospheric_context{};             /**< 场景时间/空间天气输入 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 场景植被散射输入 */
  JammerEmitterStateList jammer_sources{};                     /**< 场景干扰事实输入 */
};

/**
 * @brief EnvironmentModelConfig 描述环境服务/算法执行直接消费的参数。
 *
 * @par 类型合约
 * - 独立于 EnvironmentScenarioConfig 的 struct，禁止退化为 type alias。
 * - 字段与 ScenarioConfig 一致时，通过 BuildModelConfigFromScenario 显式字段映射构造。
 * - 若未来需差异化（增加派生字段或移除场景字段），直接修改本 struct 并更新映射函数。
 * - 调用方不得假设 ModelConfig 与 ScenarioConfig 同型。
 */
struct ONEQ_API EnvironmentModelConfig {
  AtmosphericPhysicsConfig atmospheric_physics{};
  AtmosphericDerivedContext atmospheric_context{};
  VegetationScatterPhysicsConfig vegetation_scatter_physics{};
  JammerEmitterStateList jammer_sources{};
};

/**
 * @brief EnvironmentDefaultConfig 描述初始化阶段的默认环境配置。
 *
 * @par 类型合约
 * - 初始化阶段一次性构造，构造后不再变更。
 * - 仅承载 scenario_config（外部场景事实），不包含策略枚举或调参项。
 * - 不提供运行期热更新语义；运行期更新须通过 EnvironmentRuntimeConfigPatch。
 * - 无复杂校验逻辑，构造后即视为合法。
 */
struct ONEQ_API EnvironmentDefaultConfig {
  EnvironmentScenarioConfig scenario_config{}; /**< 默认环境场景输入 */
  JammingSensitivityProfile jamming_sensitivity_profile{
      JammingSensitivityProfile::kBalanced}; /**< 干扰判定灵敏度语义档位 */
};

/**
 * @brief 将对外场景输入映射为内部环境模型配置。
 *
 * @par 映射合约
 * - 显式字段映射：逐字段从 ScenarioConfig 拷贝到 ModelConfig。
 * - 若未来引入非恒等映射（字段转换、派生、过滤），须同步添加映射单元测试。
 * - 无 fallback 语义：输入合法即输出合法。
 *
 * @param[in] scenario_config 外部场景输入。
 * @return 逐字段拷贝后的模型配置。
 */
inline ONEQ_API EnvironmentModelConfig BuildModelConfigFromScenario(
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

#endif  // ONEQ_AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_CONFIG_H_
