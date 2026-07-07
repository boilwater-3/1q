/**
 * @file EosRuntimeConfigResolver.h
 * @brief 定义 EOS 运行期补丁解析接口，直接操作内部执行配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RUNTIME_CONFIG_RESOLVER_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RUNTIME_CONFIG_RESOLVER_H_

#include "electro_optical_sensor/config/EosInternalExecutionConfig.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {

/**
 * @brief EosRuntimeConfigResolveResult 描述 EOS 运行期补丁解析结果。
 */
struct EosRuntimeConfigResolveResult {
  config::execution::EosInternalExecutionConfig next_config{}; /**< 解析后的下一份内部执行配置 */
  bool has_requested_update{false};  /**< 补丁是否请求了任一字段更新 */
  bool is_valid{true};               /**< 补丁是否合法；非法时 next_config 保持原值 */
  bool reset_scan_phase{false};      /**< 是否需要重置扫描相位 */
};

/**
 * @brief 解析 EOS 运行期补丁并直接修改内部执行配置。
 * @param[in] current_config 当前内部执行配置。
 * @param[in] patch 运行期补丁。
 * @return 解析结果。
 */
EosRuntimeConfigResolveResult ResolveEosRuntimeConfigPatch(
    const config::execution::EosInternalExecutionConfig& current_config,
    const ::electro_optical_sensor::config::EosRuntimeConfigPatch& patch);

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RUNTIME_CONFIG_RESOLVER_H_
