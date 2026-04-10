#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RUNTIME_CONFIG_RESOLVER_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RUNTIME_CONFIG_RESOLVER_H_

#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

/**
 * @brief EosRuntimeConfigResolveResult 描述 EOS 运行期补丁解析结果。
 */
struct EosRuntimeConfigResolveResult {
  EosSessionConfig next_config{};
  bool has_requested_update{false};
  bool is_valid{true};
  bool reset_scan_phase{false};
};

/**
 * @brief 解析 EOS 运行期补丁并生成统一更新结果。
 * @param[in] current_config 当前运行态配置。
 * @param[in] patch 运行期补丁。
 * @return 解析结果。
 */
EosRuntimeConfigResolveResult ResolveEosRuntimeConfigPatch(const EosSessionConfig& current_config,
                                                           const EosRuntimeConfigPatch& patch);

}  // namespace internal
}  // namespace session

namespace core {
namespace session {
namespace internal {
using ::electro_optical_sensor::session::internal::EosRuntimeConfigResolveResult;
using ::electro_optical_sensor::session::internal::ResolveEosRuntimeConfigPatch;
}  // namespace internal
}  // namespace session
}  // namespace core

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RUNTIME_CONFIG_RESOLVER_H_
