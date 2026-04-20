/**
 * @file EnvironmentRuntimeConfigPatchBuilder.h
 * @brief 提供 EnvironmentRuntimeConfigPatch 链式构造器。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_

#include "1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief EnvironmentRuntimeConfigPatch 链式构造器。
 *
 * @par 构造器合约
 * - 每个 With* 方法同时设置对应的 has_* 标志为 true。
 * - Build() 返回的补丁对象可直接提交给运行期解析器。
 * - 不执行输入校验（校验由解析器负责）。
 */
class EnvironmentRuntimeConfigPatchBuilder {
 public:
  explicit EnvironmentRuntimeConfigPatchBuilder(const EnvironmentRuntimeConfigPatch& patch = {})
      : patch_(patch) {}

  EnvironmentRuntimeConfigPatchBuilder& WithScenarioConfig(
      const EnvironmentScenarioConfig& scenario_config) {
    patch_.has_scenario_config = true;
    patch_.scenario_config = scenario_config;
    return *this;
  }

  EnvironmentRuntimeConfigPatchBuilder& WithJammingSensitivityProfile(
      JammingSensitivityProfile profile) {
    patch_.has_jamming_sensitivity_profile = true;
    patch_.jamming_sensitivity_profile = profile;
    return *this;
  }

  EnvironmentRuntimeConfigPatch Build() const { return patch_; }

 private:
  EnvironmentRuntimeConfigPatch patch_{};
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
