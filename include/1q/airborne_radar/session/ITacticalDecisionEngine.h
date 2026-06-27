/**
 * @file ITacticalDecisionEngine.h
 * @brief 定义决策协调器的公共接口与相关类型。
 */

#ifndef ONEQ_AIRBORNE_RADAR_EXTENSION_I_TACTICAL_DECISION_ENGINE_H_
#define ONEQ_AIRBORNE_RADAR_EXTENSION_I_TACTICAL_DECISION_ENGINE_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/session/ControlDirective.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/DecisionSourceInfo.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief TargetCategory 封装了单个处理周期的单个目标类别信息。
 * 目前该结构体在数据库匹配后只输出最优匹配类型以及对应该类别的特征值列表，
 * 后续可根据需要扩展更多字段。
 */
struct ONEQ_API TargetCategory {
  std::string target_type{"UNKNOWN"}; /**< 目标类别标识符 */

  float probability{0.0f}; /**< 归一化后的类别概率（来自特征库匹配；启发式分类时为 0） */

  std::unordered_map<std::string, double>
      feature_values; /**< 目标类别的特征值列表，表示该类别的特征集合 */

  TargetCategory() = default; /**< 默认构造函数 */

  explicit TargetCategory(const std::string& type) : target_type(type) {} /**< 带参数的构造函数 */

  /** @brief 添加或更新特征值 */
  void SetFeature(const std::string& feature_name, double value) {
    feature_values[feature_name] = value;
  }
};

/** @brief TargetCategoryList 是 TargetCategory 的列表，表示当前处理周期内所有相关目标类别的特征集合
 */
using TargetCategoryList = std::vector<TargetCategory>;

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
 *
 * 该结构仅承载决策引擎自身需要跨周期保留的战术语义状态。
 * ControlReducer 的 hold/cooldown 等内部运行态不属于此契约的一部分。
 */
struct ONEQ_API TacticalStateStore {
  TacticalMode current_mode{TacticalMode::kBaseline};         /**< 当前战术模式 */
  std::unordered_map<std::uint64_t, float> threat_memory;     /**< 每轨威胁记忆 */
  std::unordered_map<std::uint64_t, float> confidence_memory; /**< 每轨置信度记忆 */
  std::vector<std::string> last_classification_labels; /**< 上一周期分类标签摘要 */
  std::string last_decision_summary;                   /**< 上一周期决策摘要 */
};

/**
 * @brief TacticalProposal 表示单个 evaluator 输出的战术建议。
 */
struct ONEQ_API TacticalProposal {
  session::ControlDirective directive; /**< 控制意图 */
  int priority{0};                             /**< 建议优先级，数值越大优先级越高 */
  std::string rationale;                       /**< 生成原因 */

  TacticalProposal() = default;

  /**
   * @brief 构造战术建议。
   * @param[in] proposal_directive 控制意图。
   * @param[in] proposal_priority 建议优先级，数值越大优先级越高。
   * @param[in] proposal_rationale 生成原因。
   */
  TacticalProposal(const session::ControlDirective& proposal_directive,
                   int proposal_priority, const std::string& proposal_rationale)
      : directive(proposal_directive), priority(proposal_priority), rationale(proposal_rationale) {}
};

/**
 * @brief TacticalDecisionResult 表示决策引擎单周期输出。
 */
struct ONEQ_API TacticalDecisionResult {
  session::TargetCategoryList target_classification_result; /**< 目标分类结果 */
  std::vector<TacticalProposal> proposals;                        /**< 汇总后的战术建议集合 */
  TacticalMode selected_mode{TacticalMode::kBaseline};            /**< 当前选定战术模式 */
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
  virtual TacticalDecisionResult Evaluate(const session::DecisionInputFrame& input_frame,
                                          TacticalStateStore& state_store) = 0;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_EXTENSION_I_TACTICAL_DECISION_ENGINE_H_
