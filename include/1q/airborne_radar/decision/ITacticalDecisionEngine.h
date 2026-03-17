// Copyright 2026. All Rights Reserved.
//
// Description: 定义决策协调器的公共接口与相关类型。

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

namespace airborne_radar {
namespace decision {

/// @brief TacticalMode 表示当前战术模式。
enum class TacticalMode {
  /// @brief 基线巡航模式。
  kBaseline = 0,

  /// @brief 威胁响应模式。
  kThreatResponse,

  /// @brief 抗干扰保护模式。
  kProtectedEmission
};

/// @brief TacticalStateStore 表示跨周期战术内存。
struct TacticalStateStore {
  /// @brief 当前战术模式。
  TacticalMode current_mode{TacticalMode::kBaseline};

  /// @brief 每轨威胁记忆。
  std::unordered_map<std::uint64_t, float> threat_memory;

  /// @brief 每轨置信度记忆。
  std::unordered_map<std::uint64_t, float> confidence_memory;

  /// @brief LPI 保持计数。
  std::uint32_t lpi_hold_cycles_remaining{0};

  /// @brief ECCM 保持计数。
  std::uint32_t eccm_hold_cycles_remaining{0};

  /// @brief 上一周期分类标签摘要。
  std::vector<std::string> last_classification_labels;

  /// @brief 上一周期决策摘要。
  std::string last_decision_summary;
};

/// @brief TacticalProposal 表示单个 evaluator 输出的战术建议。
struct TacticalProposal {
  /// @brief 控制意图。
  common::ControlDirective directive;

  /// @brief 建议优先级，数值越大优先级越高。
  int priority{0};

  /// @brief 生成原因。
  std::string rationale;

  TacticalProposal() = default;

  TacticalProposal(const common::ControlDirective& proposal_directive,
                   int proposal_priority,
                   const std::string& proposal_rationale)
      : directive(proposal_directive),
        priority(proposal_priority),
        rationale(proposal_rationale) {}
};

/// @brief TacticalDecisionResult 表示决策引擎单周期输出。
struct TacticalDecisionResult {
  /// @brief 目标分类结果。
  common::TargetCategoryList target_classification_result;

  /// @brief 汇总后的战术建议集合。
  std::vector<TacticalProposal> proposals;

  /// @brief 当前选定战术模式。
  TacticalMode selected_mode{TacticalMode::kBaseline};
};

/// @brief TacticalEvaluationState 表示 evaluator 间共享的中间结果。
struct TacticalEvaluationState {
  /// @brief 目标分类结果。
  common::TargetCategoryList target_classification_result;

  /// @brief LPI 来源信息。
  common::LpiSourceInfo lpi_source_info;

  /// @brief ECCM 来源信息。
  common::EccmSourceInfo eccm_source_info;

  /// @brief 是否应进入威胁响应控制路径。
  bool should_reduce_power{false};

  /// @brief 是否应进入 ECCM 保护路径。
  bool should_enable_eccm{false};

  /// @brief evaluator 生成的战术建议集合。
  std::vector<TacticalProposal> proposals;

  TacticalEvaluationState() : eccm_source_info(false) {}
};

/// @brief ITacticalEvaluator 抽象单个 evaluator 的评估接口。
class ITacticalEvaluator {
 public:
  virtual ~ITacticalEvaluator() = default;

  /// @brief 读取输入帧与跨周期状态，并更新共享评估状态。
  virtual void Evaluate(const common::DecisionInputFrame& input_frame,
                        TacticalStateStore& state_store,
                        TacticalEvaluationState& evaluation_state) const = 0;
};

/// @brief ITacticalDecisionEngine 抽象新的决策协调器接口。
class ITacticalDecisionEngine {
 public:
  virtual ~ITacticalDecisionEngine() = default;

  /// @brief 在单周期输入帧和跨周期战术内存上执行决策。
  virtual TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame& input_frame,
      TacticalStateStore& state_store) = 0;
};

} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_I_TACTICAL_DECISION_ENGINE_H_
