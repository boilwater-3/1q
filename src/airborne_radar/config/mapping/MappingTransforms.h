/**
 * @file MappingTransforms.h
 * @brief 定义四域配置映射层的共享变换函数。
 *
 * 提取 SessionToExecutionMapper 与 RuntimePatchMapper 中重复的
 * sigma↔代价变换和 policy→engineering 调和逻辑，确保单一事实来源。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_MAPPING_TRANSFORMS_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_MAPPING_TRANSFORMS_H_

#include <cmath>
#include <vector>

#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/config/mapping/EngineeringResolvers.h"

namespace airborne_radar {
namespace config {
namespace mapping {

/**
 * @brief 将公开 API 的 distance_gate_sigma 倍数转换为内部归一化代价。
 *
 * 公开 API 使用 sigma 倍数（如 3.0），内部关联引擎使用 sigma² 作为
 * 归一化距离代价（如 9.0）。逆变换见 SquaredCostToSigma()。
 */
inline float SigmaToSquaredCost(float sigma) {
  return sigma * sigma;
}

/**
 * @brief 将内部归一化代价恢复为公开 API 的 distance_gate_sigma 倍数。
 *
 * SigmaToSquaredCost() 的逆运算。用于 MapExecutionToSession() 反向映射。
 */
inline float SquaredCostToSigma(float cost) {
  return std::sqrt(cost);
}

/**
 * @brief 将 policy 侧字段调和到 engineering 侧。
 *
 * 包含以下同步：
 * - tracking.policy → tracking.engineering（直接拷贝）
 * - lifecycle.policy → lifecycle.engineering（通过 ResolveLifecycleEngineering）
 * - 若 IMM 启用，重建 imm_model_noise_diff_coeffs；若禁用，清空该向量
 *
 * @note imm_initial_weights 和 imm_transition_probability 始终保持为空，
 *       由 ImmMatrixDefaults 在读取时自动生成默认值。
 *
 * @pre exec.lifecycle.policy 和 exec.tracking.policy 已就绪。
 * @post exec.tracking.engineering、exec.lifecycle.engineering 及 IMM 向量与 policy 一致。
 */
inline void ReconcilePolicyToEngineering(execution::InternalExecutionConfig& exec) {
  exec.tracking.engineering = exec.tracking.policy;
  exec.lifecycle.engineering = ResolveLifecycleEngineering(exec.lifecycle.policy);

  if (exec.lifecycle.engineering.enable_imm_lifecycle) {
    exec.lifecycle.imm_model_noise_diff_coeffs =
        BuildDefaultImmNoiseDiffCoeffs(exec.lifecycle.policy.model_count_hint);
  } else {
    exec.lifecycle.imm_model_noise_diff_coeffs.clear();
  }
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_MAPPING_TRANSFORMS_H_
