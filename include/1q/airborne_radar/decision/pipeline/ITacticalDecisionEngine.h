/**
 * @file ITacticalDecisionEngine.h
 * @brief 定义决策协调器的公共接口与相关类型。
 */

#ifndef AIRBORNE_RADAR_DECISION_I_TACTICAL_DECISION_ENGINE_H_
#define AIRBORNE_RADAR_DECISION_I_TACTICAL_DECISION_ENGINE_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/TargetCategory.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/**
 * @brief TacticalMode 表示当前战术模式。
 */
enum class TacticalMode {
  kBaseline = 0,     /**< 基线巡航模式 */
  kThreatResponse,   /**< 威胁响应模式 */
  kProtectedEmission /**< 抗干扰保护模式 */
};

/**
 * @brief TacticalStateStore 表示跨周期战术内存。
 */
struct TacticalStateStore {
  TacticalMode current_mode{TacticalMode::kBaseline};         /**< 当前战术模式 */
  std::unordered_map<std::uint64_t, float> threat_memory;     /**< 每轨威胁记忆 */
  std::unordered_map<std::uint64_t, float> confidence_memory; /**< 每轨置信度记忆 */
  std::uint32_t lpi_hold_cycles_remaining{0};                 /**< LPI 保持计数 */
  std::uint32_t eccm_hold_cycles_remaining{0};                /**< ECCM 保持计数 */
  std::vector<std::string> last_classification_labels;        /**< 上一周期分类标签摘要 */
  std::string last_decision_summary;                          /**< 上一周期决策摘要 */
};

/**
 * @brief TacticalProposal 表示单个 evaluator 输出的战术建议。
 */
struct TacticalProposal {
  common::ControlDirective directive; /**< 控制意图 */
  int priority{0};                    /**< 建议优先级，数值越大优先级越高 */
  std::string rationale;              /**< 生成原因 */

  TacticalProposal() = default;

  /**
   * @brief 构造战术建议。
   * @param[in] proposal_directive 控制意图。
   * @param[in] proposal_priority 建议优先级，数值越大优先级越高。
   * @param[in] proposal_rationale 生成原因。
   */
  TacticalProposal(const common::ControlDirective& proposal_directive, int proposal_priority,
                   const std::string& proposal_rationale)
      : directive(proposal_directive), priority(proposal_priority), rationale(proposal_rationale) {}
};

/**
 * @brief TacticalDecisionResult 表示决策引擎单周期输出。
 */
struct TacticalDecisionResult {
  common::TargetCategoryList target_classification_result; /**< 目标分类结果 */
  std::vector<TacticalProposal> proposals;                 /**< 汇总后的战术建议集合 */
  TacticalMode selected_mode{TacticalMode::kBaseline};     /**< 当前选定战术模式 */
};

/**
 * @brief ITacticalDecisionEngine 抽象新的决策协调器接口。
 */
class ONEQ_API ITacticalDecisionEngine {
 public:
  virtual ~ITacticalDecisionEngine() = default;

  /**
   * @brief 在单周期输入帧和跨周期战术内存上执行决策。
   * @param[in] input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期战术内存，决策执行过程中会更新。
   * @return 当前周期决策输出结果。
   */
  virtual TacticalDecisionResult Evaluate(const common::DecisionInputFrame& input_frame,
                                          TacticalStateStore& state_store) = 0;
};

}  // namespace pipeline
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_I_TACTICAL_DECISION_ENGINE_H_
