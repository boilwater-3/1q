/**
 * @file ArReplayCycleRecord.h
 * @brief 定义 AR replay 专用周期记录与内部决策状态。
 */

#ifndef AIRBORNE_RADAR_SESSION_AR_REPLAY_CYCLE_RECORD_H_
#define AIRBORNE_RADAR_SESSION_AR_REPLAY_CYCLE_RECORD_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "airborne_radar/decision/ControlReducer.h"

namespace airborne_radar {
namespace session {

/** @brief replay 比对所需、但不属于业务周期输出的决策内部状态。 */
struct ArDecisionReplayState {
  bool has_pending_internal_decision{false};
  std::uint32_t pending_internal_cycle_index{0U};
  std::uint64_t pending_internal_batch_id{0U};
  std::vector<session::TacticalProposal> pending_internal_proposals{};
  session::DecisionControlSource applied_decision_source{
      session::DecisionControlSource::kNone};
  std::uint32_t applied_decision_cycle_index{0U};
  std::uint64_t applied_decision_batch_id{0U};
  std::vector<session::TacticalProposal> applied_decision_proposals{};
  bool has_pending_external_decision{false};
  session::ExternalDecisionResponse pending_external_decision{};
  decision::ControlReducerRuntimeState reducer_state{};
};

/** @brief replay 的单周期输出记录，由公开业务结果和内部决策状态组成。 */
struct ArReplayCycleRecord {
  ArCycleResult result{};
  ArDecisionReplayState decision_state{};
};

class ArSession;

/** @brief 仅供 AR replay 包装器读取 ArSession 内部快照的窄访问适配器。 */
class ArSessionReplayAccess {
 public:
  static ArDecisionReplayState CaptureDecisionState(const ArSession& session);
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_REPLAY_CYCLE_RECORD_H_
