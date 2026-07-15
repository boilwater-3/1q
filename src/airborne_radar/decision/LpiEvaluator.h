/**
 * @file LpiEvaluator.h
 * @brief 定义 LPI（低截获概率）评估器类。
 */

#ifndef AIRBORNE_RADAR_DECISION_LPI_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_LPI_EVALUATOR_H_

#include <vector>

#include "1q/airborne_radar/session/DecisionControlTypes.h"
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
    float dwell_scale{1.0f};              /**< 推荐驻留比例 [0.65,0.90] */
  };

  /**
   * @brief 基于威胁来源信息生成 LPI 战术提案。
   * @param[in] lpi_source_info  ThreatAssessment 输出的 LPI 威胁来源信息
   * @param[out] proposals        追加 LPI 提案的输出列表
   * @return 本评估周期的 LPI 评估结果
   */
  Result Evaluate(const model::LpiSourceInfo& lpi_source_info,
                  std::vector<session::TacticalProposal>* proposals);

 private:
  /**
   * @brief 根据威胁信息计算自适应降功率比例。
   *
   * 综合考虑威胁距离、接近速度和目标类型进行分级降功率决策：
   * - 近距离 + 高速 → 激进降功率（最低比例）
   * - 中距离 + 中速 → 适度降功率（中等比例）
   * - 远距离 + 低速 → 保守降功率或保持
   *
   * @param[in] info LPI 威胁来源信息
   * @return 功率比例 [0, 1]，值越低代表功率压得越低
   */
  float ComputePowerScale(const model::LpiSourceInfo& info) const;

  /** @brief 根据功率比例生成同步的 LPI 驻留比例。 */
  float ComputeDwellScale(float power_scale) const;
};

}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_LPI_EVALUATOR_H_
