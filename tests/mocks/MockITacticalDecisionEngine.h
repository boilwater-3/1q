/**
 * @file MockITacticalDecisionEngine.h
 * @brief ITacticalDecisionEngine 的 GMock mock 实现，供单元测试隔离使用。
 */

#ifndef TESTS_MOCK_I_TACTICAL_DECISION_ENGINE_H_
#define TESTS_MOCK_I_TACTICAL_DECISION_ENGINE_H_

#include <gmock/gmock.h>

#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

class MockITacticalDecisionEngine : public ITacticalDecisionEngine {
 public:
  MOCK_METHOD(TacticalDecisionResult, Evaluate,
              (const common::model::DecisionInputFrame& input_frame, TacticalStateStore& state_store),
              (override));
};

}  // namespace pipeline
}  // namespace decision
}  // namespace airborne_radar

#endif  // TESTS_MOCK_I_TACTICAL_DECISION_ENGINE_H_
