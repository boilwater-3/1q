/**
 * @file SarRuntimeConfigPatch.h
 * @brief 定义 SAR 运行期可变配置补丁。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_SAR_CONFIG_SAR_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"

namespace sar {
namespace config {

/**
 * @brief SAR 会话运行期可变配置补丁。
 */
struct ONEQ_API SarRuntimeConfigPatch {
  bool has_enable_raw_echo_generation{false};
  bool enable_raw_echo_generation{true};

  bool has_enable_range_compression{false};
  bool enable_range_compression{true};

  bool has_enable_l1_rda_imaging{false};
  bool enable_l1_rda_imaging{false};

  /// 保留字段补丁：当前 public result 不返回 raw phase history；应用该补丁仅保持
  /// runtime/replay 配置保真，不改变本周期输出载荷。
  bool has_retain_raw_phase_history{false};
  bool retain_raw_phase_history{false};

  bool has_retain_focused_image{false};
  bool retain_focused_image{true};

  bool has_minimum_snr_db{false};
  double minimum_snr_db{-10.0};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_RUNTIME_CONFIG_PATCH_H_
