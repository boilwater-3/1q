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
  bool enable_raw_echo_generation{true};
  /// 距离压缩开关。当前 Phase 1 不产出独立可消费的距离压缩产物，真实距离压缩在
  /// RDA / BP 内部完成。此开关作为 L3 BP 的前置条件门（参见 l3_bp_session_integration
  /// 契约：L3 BP 必须同时启用 raw echo generation 与 range compression），并在启用时
  /// 触发 `SarProcessingStage::kRangeCompression` 阶段标记与 `has_range_compressed_echo`
  /// 摘要；该标记是前置条件摘要，不是独立输出载荷。
  bool enable_range_compression{true};
  bool enable_l1_rda_imaging{false};
  bool enable_l2_motion_compensation{false};
  bool enable_l3_bp_imaging{false};
  /// 是否返回非错误诊断。关闭时仍保留错误诊断和 abort_reason。
  bool enable_diagnostics{true};
  /// 保留字段：当前 public result 不返回 raw phase history，仅 replay/config 保真。
  bool retain_raw_phase_history{false};
  /// 是否在 SarCycleResult 中返回完整聚焦复图像。默认开启以保持向后兼容；
  /// 关闭时 focused_image 仅含占位元数据（is_placeholder=true），可避免大图拷贝。
  bool retain_focused_image{true};
  /// 保留字段：当前 session 尚未实现 squint-angle runtime gate。
  double max_allowed_squint_angle_deg{5.0};
  /// 原始孔径峰均功率比估算 SNR 的最低有效门限，低于该值时本周期中止。
  /// 全零/空孔径返回不可估计值，不触发该低 SNR 门控。
  double min_valid_snr_db{-10.0};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_POLICY_CONFIG_H_
