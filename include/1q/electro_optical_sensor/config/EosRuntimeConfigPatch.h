/**
 * @file EosRuntimeConfigPatch.h
 * @brief 定义 EOS 会话运行期配置补丁结构。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/config/EosPolicyConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosEnvironmentRuntimeConfigPatch 描述运行期可变环境补丁。
 */
struct ONEQ_API EosEnvironmentRuntimeConfigPatch {
  bool has_scenario_config{false};
  EosEnvironmentScenarioConfig scenario_config{};
};

/**
 * @brief config::EosRuntimeConfigPatch 描述运行期可变高层策略补丁。
 * @note 覆盖顺序：先应用整块域覆盖（mission/policy/environment），
 *       再应用叶子快捷字段（work_mode/scan_rate_deg_per_sec/frame_rate_hz）。
 */
struct ONEQ_API EosRuntimeConfigPatch {
  bool has_mission{false}; /**< 是否整块覆盖 mission */
  EosMissionConfig mission{}; /**< mission 覆盖值 */

  bool has_policy{false}; /**< 是否整块覆盖 policy */
  EosPolicyConfig policy{}; /**< policy 覆盖值 */

  bool has_environment{false}; /**< 是否应用 environment 白名单补丁 */
  EosEnvironmentRuntimeConfigPatch environment{}; /**< environment 白名单补丁 */

  bool has_work_mode{false};            /**< 是否显式设置工作模式 */
  EosWorkMode work_mode{EosWorkMode::kFused}; /**< 工作模式值 */

  bool has_scan_rate_deg_per_sec{false}; /**< 是否显式设置扫描角速度 */
  float scan_rate_deg_per_sec{20.0f};     /**< 扫描角速度（单位：deg/s） */

  bool has_frame_rate_hz{false}; /**< 是否显式设置帧率 */
  float frame_rate_hz{30.0f};    /**< 帧率（单位：Hz） */

  bool has_sensor_enabled{false}; /**< [补丁标志] 是否显式设置传感器开关状态 */
  bool sensor_enabled{true};      /**< [可外部调整] 传感器开关状态 */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_
