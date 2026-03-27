/**
 * @file TacticalEvaluation.h
 * @brief 定义默认决策协调器内部评估拼装类型。
 */

#ifndef AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_
#define AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_

#include <vector>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/**
 * @brief 表示 evaluator 间共享的中间结果。
 *
 * ## 字段所有权与读写约定
 *
 * 字段分为两类：
 * - **输入字段**：由 `TacticalCoordinator` 在调用 evaluator 前初始化，evaluator 只读。
 * - **输出字段**：由各 evaluator 在执行时累加写入，后续 evaluator 可读取前序结果。
 *
 * ## Evaluator 执行顺序约束（load-bearing，不可随意调整）
 *
 * ```
 * [Pre-fill] TacticalCoordinator 初始化 eccm_source_info，
 *            BackfillAssociationDrivenEccmTrigger 可能设置 has_jamming_signal = true
 *     ↓
 * [1] ThreatAssessmentEvaluator
 *       写: target_classification_result, should_reduce_power,
 *           eccm_source_info.has_jamming_signal（基于 track 干扰事实）,
 *           lpi_source_info.has_recon_platform
 *     ↓
 * [2] EmissionControlEvaluator
 *       读: lpi_source_info（[1] 已写入）
 *       写: proposals（LPI 相关），should_reduce_power（可能更新）
 *     ↓
 * [3] SurvivabilityEvaluator
 *       读: eccm_source_info（Pre-fill + [1] 已写入）
 *       写: should_enable_eccm, proposals（ECCM 相关）
 * ```
 *
 * **[3] 依赖 [1] 对 `eccm_source_info.has_jamming_signal` 的写入**。
 * 若顺序改变或引入并行，必须重新审查各字段的数据依赖。
 */
struct TacticalEvaluationState {
  // ---------- 输入字段（TacticalCoordinator 初始化，evaluator 只读） ----------

  /** @brief 供各 evaluator 使用的 ECCM 来源信息。
   *  @note 由 TacticalCoordinator 预填充，ThreatAssessmentEvaluator 可补写
   *        `has_jamming_signal`（基于 track 干扰事实）；SurvivabilityEvaluator 只读。 */
  common::EccmSourceInfo eccm_source_info;

  // ---------- 输出字段（evaluator 写入，后续 evaluator 或 Coordinator 读取） ----------

  /** @brief 当前周期的目标分类结果（由 ThreatAssessmentEvaluator 写入）。 */
  common::TargetCategoryList target_classification_result;
  /** @brief LPI 来源信息（由 ThreatAssessmentEvaluator 写入，EmissionControlEvaluator 读取）。 */
  common::LpiSourceInfo lpi_source_info;
  /** @brief 是否应触发降功率路径（由 ThreatAssessmentEvaluator 写入）。 */
  bool should_reduce_power{false};
  /** @brief 是否应触发 ECCM 保护发射路径（由 SurvivabilityEvaluator 写入）。 */
  bool should_enable_eccm{false};
  /** @brief 当前周期累计的控制提案列表（由各 evaluator 依序追加）。 */
  std::vector<TacticalProposal> proposals;

  TacticalEvaluationState() : eccm_source_info(false) {}
};

/**
 * @brief 抽象单个 evaluator 的评估接口。
 */
class ITacticalEvaluator {
 public:
  virtual ~ITacticalEvaluator() = default;

  /**
   * @brief 执行单个 evaluator 的评估逻辑。
   * @param input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期战术状态存储。
   * @param[in,out] evaluation_state evaluator 间共享的中间结果。
   */
  virtual void Evaluate(const common::DecisionInputFrame& input_frame,
                        TacticalStateStore& state_store,
                        TacticalEvaluationState& evaluation_state) const = 0;
};

}  // namespace pipeline
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_
