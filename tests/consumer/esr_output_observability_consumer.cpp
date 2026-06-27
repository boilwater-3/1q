/**
 * @file esr_output_observability_consumer.cpp
 * @brief 验证安装后 ESR 三层输出可观测性 API 可被外部工程编译链接。
 *
 * 覆盖要点（阶段 9 三层输出）：
 *   - EsrOutputDebugViewBuilder 把三通道输出 + 输入辐射源表合成开发可读视图
 *   - EsrEmitterLifecycleRecorder 跨周期生命周期记录
 *   - 真实输出通道不含 emitter_name（name 只经 debug view 回填）
 *
 * 是 esr_session_consumer 的可观测性补充。
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrEmitterLifecycleRecorder.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputDebugView.h"
#include "1q/electronic_surveillance_radar/session/EsrSceneTypes.h"

namespace esr = electronic_surveillance_radar;

int main() {
  // 1. 输入辐射源带 name。
  esr::session::EsrCycleInput input;
  input.cycle_index = 1U;
  esr::session::EsrSceneEmitter emitter;
  emitter.emitter_id = 7U;
  emitter.emitter_name = "consumer-emitter";
  emitter.is_emitting = true;
  input.scene.push_back(emitter);

  // 2. 手填一个含 truth association 的结果。
  esr::session::EsrCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.executed_this_cycle = true;
  result.output_frame.cycle_index = input.cycle_index;
  esr::session::TruthAssociationRecord association;
  association.observation_id = 100U;
  association.truth_emitter_id = 7U;
  association.matched = true;
  association.confidence = 0.8f;
  result.output_frame.truth_evaluation_output.associations.push_back(association);

  // 3. Debug view：name 经 truth association 从输入表回填。
  const esr::session::EsrOutputDebugView view =
      esr::session::EsrOutputDebugViewBuilder::Build(input, result);
  if (view.emitters.size() != 1U) {
    return 1;
  }
  if (view.emitters[0].emitter_name != "consumer-emitter") {
    return 2;
  }

  // 4. Lifecycle recorder。
  esr::session::EsrEmitterLifecycleRecorder recorder;
  std::vector<esr::session::EsrEmitterLifecycleEvent> events = recorder.Update(input, result);
  (void)events;

  return 0;
}
