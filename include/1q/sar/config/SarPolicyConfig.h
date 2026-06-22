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
  bool enable_range_compression{true};
  bool enable_l1_rda_imaging{false};
  bool enable_l2_motion_compensation{false};
  bool enable_l3_bp_imaging{false};
  bool enable_diagnostics{true};
  bool retain_raw_phase_history{false};
  /// 是否在 SarCycleResult 中返回完整聚焦复图像。默认开启以保持向后兼容；
  /// 关闭时 focused_image 仅含占位元数据（is_placeholder=true），可避免大图拷贝。
  bool retain_focused_image{true};
  double max_allowed_squint_angle_deg{5.0};
  double min_valid_snr_db{-10.0};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_POLICY_CONFIG_H_
