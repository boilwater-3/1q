/**
 * @file AhpEvaluator.h
 * @brief 层次分析法（AHP）纯函数算法面：判断矩阵 → 权重 + 一致性（需求 3.2.1.6.3.2）。
 */

#ifndef ONEQ_PRECISION_EVALUATION_AHP_EVALUATOR_H_
#define ONEQ_PRECISION_EVALUATION_AHP_EVALUATOR_H_

#include "1q/api.hpp"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"

namespace precision_evaluation {

/**
 * @brief 求解 AHP 判断矩阵：主特征向量权重 + λmax/CI/CR 一致性。
 * @param[in] matrix 5×5 Saaty 互反判断矩阵（对角须 1、须倒数对称 a_ij·a_ji=1、
 *                   元素须在 [1/9, 9]；校验容差 1e-6）
 * @param[out] out 求解结果（权重 Σ=1）
 * @return 矩阵合法且数值求解收敛返回 true；非法矩阵或幂迭代不收敛返回 false
 *        （out 不写入，调用方按"评估不可用"处理，不得静默退化为等权）。
 * @note 主特征向量用幂迭代（Perron 向量非负保证收敛，双精度，容差 1e-12、上限
 *       1000 迭代）；λmax 用 Rayleigh 商；CI=(λmax−n)/(n−1)；CR=CI/RI，n=5 取
 *       RI=1.12（Saaty 随机一致性指标表固化 n≤5）；is_consistent = CR ≤ 0.1。
 *       一致性不满足时仍返回权重，但 is_consistent=false——调用方须提示重标定
 *       判断矩阵，不得隐藏。
 */
bool ONEQ_API TryEvaluateAhp(const AhpJudgmentMatrix& matrix, AhpEvaluation* out);

/**
 * @brief 由五指标 RMSE 与 AHP 权重合成综合定位精度得分与贡献分解。
 * @param[in] weights 五指标归一化权重（Σ=1，来自 TryEvaluateAhp）
 * @param[in] rmse 五指标 RMSE（与 PrecisionMetric 枚举同序）
 * @param[in] reference_errors 五指标参考误差（与枚举同序；≤0 的项得分记 0）
 * @param[out] report 填写 report 的 metric_scores / metric_contributions /
 *                    composite_score / reference_errors 字段（metrics 与 ahp 由
 *                    调用方另行填充）
 * @note composite = Σ w_i · ref_i/(ref_i+rmse_i) ∈ [0,1]，越大定位精度越好；
 *       贡献分解 w_i·score_i 之和恒等于综合分（可解释性，先例 ThreatEvaluator）。
 */
void ONEQ_API ComposePrecisionScore(const double (&weights)[kPrecisionMetricCount],
                                     const double (&rmse)[kPrecisionMetricCount],
                                     const double (&reference_errors)[kPrecisionMetricCount],
                                     PrecisionEvaluationReport* report);

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_AHP_EVALUATOR_H_
