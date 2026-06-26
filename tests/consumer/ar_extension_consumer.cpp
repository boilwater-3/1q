/**
 * @file ar_extension_consumer.cpp
 * @brief 验证安装后机载雷达扩展接口可被外部工程实现并接入会话。
 *
 * 本 consumer 演示 public API 的唯一自定义扩展点:外部实现 ITacticalDecisionEngine
 * 替换 AR 决策逻辑,其余组件(context / pipeline / environment service)由
 * RadarSessionFactory 内部默认装配,不对外暴露。
 */

#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarSession.h"

namespace airborne_radar {
namespace {

class DummyDecisionEngine : public session::ITacticalDecisionEngine {
 public:
  session::TacticalDecisionResult Evaluate(const session::DecisionInputFrame& input_frame,
                                             session::TacticalStateStore& state_store) override {
    (void)input_frame;
    (void)state_store;
    return {};
  }
};

}  // namespace
}  // namespace airborne_radar

int main() {
  airborne_radar::DummyDecisionEngine decision_engine;

  // 注入自定义决策引擎创建会话;context/pipeline/environment 由工厂内部装配。
  airborne_radar::session::RadarSession session =
      airborne_radar::session::RadarSession::CreateWithDecisionEngine(
          airborne_radar::config::RadarSessionConfig{}, decision_engine);

  airborne_radar::session::RadarCycleInput input;
  const airborne_radar::session::RadarCycleResult result = session.StepWithResult(input);
  return result.executed_this_cycle ? 0 : 1;
}
