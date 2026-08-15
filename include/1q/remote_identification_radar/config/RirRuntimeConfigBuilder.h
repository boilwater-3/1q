/**
 * @file RirRuntimeConfigBuilder.h
 * @brief 远程识别雷达运行期配置补丁建造者。
 *
 * 链式方法统一 With* 动词（对齐 check_cross_domain_naming 规约 P3-b）；
 * 必须提供 `WithRuntimeConfigPatch` 整块覆盖入口与 `WithSensorEnabled` 电源入口。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_BUILDER_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirRuntimeConfigBuilder 运行期补丁建造者（薄封装）。
 */
class ONEQ_API RirRuntimeConfigBuilder {
 public:
  RirRuntimeConfigBuilder() = default;

  /** @brief 叶子覆盖：工作模式。 */
  RirRuntimeConfigBuilder& WithWorkMode(RirWorkMode work_mode) {
    patch_.has_work_mode = true;
    patch_.work_mode = work_mode;
    return *this;
  }

  /** @brief 整域覆盖：策略域（识别策略整块覆盖）。 */
  RirRuntimeConfigBuilder& WithPolicy(const RirPolicyConfig& policy) {
    patch_.has_policy = true;
    patch_.policy = policy;
    return *this;
  }

  /** @brief 叶子覆盖：传感器电源。 */
  RirRuntimeConfigBuilder& WithSensorEnabled(bool sensor_enabled) {
    patch_.has_sensor_enabled = true;
    patch_.sensor_enabled = sensor_enabled;
    return *this;
  }

  /** @brief 整块覆盖入口：整包替换当前补丁。 */
  RirRuntimeConfigBuilder& WithRuntimeConfigPatch(const RirRuntimeConfigPatch& patch) {
    patch_ = patch;
    return *this;
  }

  /** @brief 返回当前补丁副本。 */
  RirRuntimeConfigPatch Build() const { return patch_; }

 private:
  RirRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_BUILDER_H_
