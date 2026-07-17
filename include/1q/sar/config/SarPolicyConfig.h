/**
 * @file SarPolicyConfig.h
 * @brief 定义 SAR 会话执行策略配置。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_POLICY_CONFIG_H_
#define ONEQ_SAR_CONFIG_SAR_POLICY_CONFIG_H_

#include "1q/api.hpp"

namespace sar {
namespace config {

/**
 * @brief SAR 算法与运行策略。
 */
struct ONEQ_API SarPolicyConfig {
  bool enable_raw_echo_generation{true};      /**< 是否启用 raw echo 生成阶段 */
  bool enable_range_compression{true};        /**< L1 RDA 与 L3 BP 的距离压缩前置条件开关。当前不产出独立距离压缩载荷；RDA/BP 在内部执行真实距离压缩并于成功后发布 `has_range_compressed_echo`，仅启用本开关不发布完成状态 */
  bool enable_l1_rda_imaging{false};          /**< 是否启用 L1 RDA 聚焦成像 */
  bool enable_l2_motion_compensation{false};  /**< 是否启用 L2 一阶运动补偿 */
  bool enable_l3_bp_imaging{false};           /**< 是否启用 L3 BP 聚焦成像 */
  bool enable_diagnostics{true};              /**< 是否返回非错误诊断。关闭时仍保留错误诊断和 abort_reason */
  bool retain_raw_phase_history{false};       /**< 是否在成功周期的 SarCycleResult 中返回实际使用的完整孔径 I/Q 相位历史；要求同时启用 raw echo generation。 */
  bool retain_focused_image{true};            /**< 是否在 SarCycleResult 中返回完整聚焦复图像。默认开启以保持向后兼容；关闭时 focused_image 仅含占位元数据（is_placeholder=true），可避免大图拷贝 */
  double max_allowed_squint_angle_deg{5.0};   /**< 成像路径允许的最大绝对 squint 角，范围 [0, 90) deg；raw-echo-only 不执行该门。 */
  double minimum_snr_db{-10.0};               /**< 内部生成孔径的接收信号功率/热噪声功率最低门限（dB），低于时中止。全零孔径及无信号/噪声元数据的 external raw IQ 返回不可估计值，不触发门控 */
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_POLICY_CONFIG_H_
