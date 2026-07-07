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
  bool has_enable_raw_echo_generation{false};  /**< 是否携带 enable_raw_echo_generation 变更 */
  bool enable_raw_echo_generation{true};       /**< 目标 raw echo 生成开关值 */

  bool has_enable_range_compression{false};    /**< 是否携带 enable_range_compression 变更 */
  bool enable_range_compression{true};         /**< 目标距离压缩开关值 */

  bool has_enable_l1_rda_imaging{false};       /**< 是否携带 enable_l1_rda_imaging 变更 */
  bool enable_l1_rda_imaging{false};           /**< 目标 L1 RDA 成像开关值 */

  bool has_retain_raw_phase_history{false};    /**< 保留字段补丁：当前 public result 不返回 raw phase history；应用该补丁仅保持 runtime/replay 配置保真，不改变本周期输出载荷 */
  bool retain_raw_phase_history{false};        /**< 目标 retain_raw_phase_history 值 */

  bool has_retain_focused_image{false};        /**< 是否携带 retain_focused_image 变更 */
  bool retain_focused_image{true};             /**< 目标 retain_focused_image 值 */

  bool has_minimum_snr_db{false};              /**< 是否携带 minimum_snr_db 变更 */
  double minimum_snr_db{-10.0};                /**< 目标最低有效 SNR 门限值（dB） */
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_RUNTIME_CONFIG_PATCH_H_
