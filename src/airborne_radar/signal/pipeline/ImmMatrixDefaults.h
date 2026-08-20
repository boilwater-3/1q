/**
 * @file ImmMatrixDefaults.h
 * @brief IMM 转移概率矩阵与初始权重的单一构建/校验源。
 *
 * SignalComponentFactory（初始组装）与 RuntimeAssemblySupport（runtime 重组）原先
 * 各维护一份相同语义的构建/校验逻辑，存在 3 处行为分叉（model_count==0 防御、NaN/Inf
 * 校验、违规日志）。本单元将其唯一化为一份，统一收敛到更严的校验语义，并通过
 * ViolationReporter 回调保留各调用上下文的可观测性差异。
 *
 * 本单元为模块内部私有，不对外暴露。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_IMM_MATRIX_DEFAULTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_IMM_MATRIX_DEFAULTS_H_

#include <Eigen/Core>
#include <cstddef>
#include <functional>

#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace imm_defaults {

using ExecutionConfig = ::airborne_radar::config::execution::InternalExecutionConfig;

/**
 * @brief 违规报告回调类型。
 *
 * 当矩阵/权重校验失败时被调用。调用方可传入记录日志的函数（初始组装路径），
 * 或传入空回调（runtime 重组路径保持静默）。
 *
 * @param message 违规描述（静态字符串字面量）。
 * @param value   与违规相关的数值（如越界值、行和、尺寸等）。
 */
using ViolationReporter = std::function<void(const char* message, float value)>;

/**
 * @brief 构建 IMM 转移概率矩阵（行随机：每行和为 1）。
 *
 * - 空配置（imm_transition_probability 为空）时返回默认矩阵：对角 0.95、
 *   非对角 0.05/(n-1)（n==1 时为 1.0）。
 * - 显式配置时校验：尺寸须为 model_count*model_count、每个值有限且在 [0,1]、
 *   每行和 ≈1（阈值 1e-3）。任一不满足返回空矩阵。
 * - model_count==0 返回空矩阵。
 *
 * @param config           执行配置（读取 lifecycle.imm_transition_probability）。
 * @param model_count      IMM 模型数。
 * @param report_violation 校验失败时的报告回调（可为空）。
 * @return 转移概率矩阵；校验失败或 model_count==0 时返回空矩阵。
 */
Eigen::MatrixXf BuildTransitionProbability(const ExecutionConfig& config, std::size_t model_count,
                                           const ViolationReporter& report_violation);

/**
 * @brief 构建 IMM 初始权重向量（和为 1）。
 *
 * - 空配置（imm_initial_weights 为空）时返回默认向量：均匀 1/n。
 * - 显式配置时校验：尺寸须为 model_count、每个值有限且在 [0,1]、和 ≈1（阈值 1e-3）。
 *   任一不满足返回空向量。
 * - model_count==0 返回空向量。
 *
 * @param config           执行配置（读取 lifecycle.imm_initial_weights）。
 * @param model_count      IMM 模型数。
 * @param report_violation 校验失败时的报告回调（可为空）。
 * @return 初始权重向量；校验失败或 model_count==0 时返回空向量。
 */
Eigen::VectorXf BuildInitialWeights(const ExecutionConfig& config, std::size_t model_count,
                                    const ViolationReporter& report_violation);

}  // namespace imm_defaults
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_IMM_MATRIX_DEFAULTS_H_
