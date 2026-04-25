/**
 * @file LpiEvaluator.h
 * @brief 定义 LPI（低截获概率）评估器类。
 */

#ifndef AIRBORNE_RADAR_DECISION_LPI_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_LPI_EVALUATOR_H_

#include <vector>

#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "airborne_radar/decision/LpiSourceInfo.h"

namespace airborne_radar {
namespace decision {

/**
 * @brief 负责根据威胁评估结果生成低截获概率（LPI）控制提案。
 *
 * 当前阶段实现自适应功率管理：根据威胁类型、距离、接近速度等信息
 * 动态计算降功率比例，生成 LPI 功率控制提案。
 */
class LpiEvaluator final {
 public:
  LpiEvaluator() = default;

  /**
   * @brief LPI 评估结果。
   */
  struct Result {
    bool requests_power_reduction{false}; /**< 是否请求 LPI 降功率 */
    float power_scale{1.0f};              /**< 推荐降功率比例 [0,1]，值越低功率压得越低 */
  };

  /**
   * @brief 基于威胁来源信息生成 LPI 战术提案。
   * @param lpi_source_info  ThreatAssessment 输出的 LPI 威胁来源信息
   * @param proposals        [out] 追加 LPI 提案
   * @return 本评估周期的 LPI 评估结果
   */
  Result Evaluate(const model::LpiSourceInfo& lpi_source_info,
                  std::vector<extension::TacticalProposal>* proposals);

 private:
  /**
   * @brief 根据威胁信息计算自适应降功率比例。
   *
   * 综合考虑威胁距离、接近速度和目标类型进行分级降功率决策：
   * - 近距离 + 高速 → 激进降功率（最低比例）
   * - 中距离 + 中速 → 适度降功率（中等比例）
   * - 远距离 + 低速 → 保守降功率或保持
   *
   * @param info LPI 威胁来源信息
   * @return 功率比例 [0, 1]，值越低代表功率压得越低
   */
  float ComputePowerScale(const model::LpiSourceInfo& info) const;
};

}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_LPI_EVALUATOR_H_
