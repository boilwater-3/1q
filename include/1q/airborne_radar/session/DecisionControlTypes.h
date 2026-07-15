/**
 * @file DecisionControlTypes.h
 * @brief 定义 AR 决策控制公共 DTO。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_DECISION_CONTROL_TYPES_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_DECISION_CONTROL_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ControlDirective.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

struct ONEQ_API TacticalProposal {
  session::ControlDirective directive;
  int priority{0};
  std::string rationale;

  TacticalProposal() = default;
  TacticalProposal(const session::ControlDirective& proposal_directive,
                   int proposal_priority, const std::string& proposal_rationale)
      : directive(proposal_directive),
        priority(proposal_priority),
        rationale(proposal_rationale) {}
};

/** @brief 一个成功探测周期结束后发布给外部决策模块的观测。 */
struct ONEQ_API DecisionObservation {
  session::DecisionInputFrame input_frame{};
  session::ArControlProfile active_control_profile{};
};

/** @brief 外部模块针对指定观测周期提交的完整 LPI/ECCM 控制建议。 */
struct ONEQ_API ExternalDecisionResponse {
  std::uint32_t source_cycle_index{0U};
  std::uint64_t source_batch_id{0U};
  std::vector<TacticalProposal> proposals{};
};

enum class ONEQ_API ExternalDecisionSubmitStatus {
  kAccepted = 0,
  kNoPendingObservation,
  kSourceMismatch,
  kAlreadySubmitted,
  kInvalidProposal
};

enum class ONEQ_API DecisionControlSource {
  kNone = 0,
  kInternal,
  kExternal
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_DECISION_CONTROL_TYPES_H_
