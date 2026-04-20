/**
 * @file EnvironmentDefaultConfigBuilder.h
 * @brief 提供链式构造 EnvironmentDefaultConfig 的 Builder。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_DEFAULT_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_DEFAULT_CONFIG_BUILDER_H_

#include "1q/airborne_radar/environment/EnvironmentConfig.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief EnvironmentDefaultConfig 链式构造器。
 *
 * 以 `EnvironmentDefaultConfig` 的直接字段为粒度提供 setter，
 * 用于覆盖默认环境模型配置与干扰判定语义档位：
 *
 * @par 构造器合约
 * - 仅修改 EnvironmentDefaultConfig 的直接字段，不执行复杂校验。
 * - 不执行 ScenarioConfig -> ModelConfig 映射（映射由会话初始化阶段完成）。
 * - 不执行策略枚举到工程参数的解析（解析由会话初始化阶段完成）。
 * - Build() 返回的配置对象构造后即视为合法。
 *
 * @code
 * EnvironmentScenarioConfig scenario;
 * scenario.atmospheric_physics.enable_physical_model = true;
 *
 * EnvironmentDefaultConfig env = EnvironmentDefaultConfigBuilder()
 *     .WithScenarioConfig(scenario)
 *     .Build();
 * env.jamming_sensitivity_profile = JammingSensitivityProfile::kRelaxed;
 * @endcode
 */
class EnvironmentDefaultConfigBuilder {
 public:
  /**
   * @brief 以给定配置为基础构造 Builder。
   * @param[in] config 基础配置，默认为零值构造的 EnvironmentDefaultConfig。
   */
  explicit EnvironmentDefaultConfigBuilder(const EnvironmentDefaultConfig& config = {})
      : config_(config) {}

  /** @brief 覆盖默认环境场景输入（气象、植被、干扰事实等）。 */
  EnvironmentDefaultConfigBuilder& WithScenarioConfig(
      const EnvironmentScenarioConfig& scenario_config) {
    config_.scenario_config = scenario_config;
    return *this;
  }

  /** @brief 生成配置对象。 */
  EnvironmentDefaultConfig Build() const { return config_; }

 private:
  EnvironmentDefaultConfig config_;
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_DEFAULT_CONFIG_BUILDER_H_
