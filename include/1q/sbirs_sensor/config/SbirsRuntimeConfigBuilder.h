/**
 * @file SbirsRuntimeConfigBuilder.h
 * @brief 定义 SBIRS-inspired runtime patch builder。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_BUILDER_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"

namespace sbirs_sensor {
namespace config {

class ONEQ_API SbirsRuntimeConfigBuilder {
 public:
  /**
   * @brief 设置整域 mission 配置。
   * @param[in] mission 任务与视场参数
   * @return 自身引用，支持链式调用
   */
  SbirsRuntimeConfigBuilder& WithMission(const SbirsMissionConfig& mission);
  /**
   * @brief 设置整域 policy 配置。
   * @param[in] policy 检测/误差/调度策略
   * @return 自身引用，支持链式调用
   */
  SbirsRuntimeConfigBuilder& WithPolicy(const SbirsPolicyConfig& policy);
  /**
   * @brief 设置整域 environment 配置。
   * @param[in] environment 环境与气象衰减参数
   * @return 自身引用，支持链式调用
   */
  SbirsRuntimeConfigBuilder& WithEnvironment(const SbirsEnvironmentConfig& environment);
  /**
   * @brief 覆盖工作模式字段。
   * @param[in] work_mode 目标工作模式
   * @return 自身引用，支持链式调用
   */
  SbirsRuntimeConfigBuilder& WithWorkMode(SbirsWorkMode work_mode);
  /**
   * @brief 覆盖扫描速率字段。
   * @param[in] scan_rate_deg_per_sec WFOV 扫描速率，单位 deg/s
   * @return 自身引用，支持链式调用
   */
  SbirsRuntimeConfigBuilder& WithScanRateDegPerSec(float scan_rate_deg_per_sec);
  /**
   * @brief 覆盖传感器电源状态。
   * @param[in] power_on 是否开启传感器
   * @return 自身引用，支持链式调用
   */
  SbirsRuntimeConfigBuilder& WithPowerOn(bool power_on);
  /**
   * @brief 构造并返回累积的运行期配置补丁。
   * @return 已设置字段的 `SbirsRuntimeConfigPatch`
   */
  SbirsRuntimeConfigPatch Build() const;

 private:
  SbirsRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_BUILDER_H_
