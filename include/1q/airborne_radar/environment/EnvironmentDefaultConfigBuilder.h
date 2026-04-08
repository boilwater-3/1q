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
 * 分别对应默认环境模型配置和干扰判定阈值：
 *
 * @code
 * EnvironmentModelConfig model;
 * model.base_propagation_loss_db = 5.0f;
 * model.clutter_power_db         = 3.0f;
 *
 * EnvironmentDefaultConfig env = EnvironmentDefaultConfigBuilder()
 *     .WithModelConfig(model)
 *     .Build();
 * env.jamming_detection_threshold_db = 8.0f;
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

  /** @brief 覆盖默认环境模型配置（传播损耗、杂波功率、物理参数等）。 */
  EnvironmentDefaultConfigBuilder& WithModelConfig(const EnvironmentModelConfig& model_config) {
    config_.model_config = model_config;
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
