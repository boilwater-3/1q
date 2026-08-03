/**
 * @file eos_output_observability_consumer.cpp
 * @brief 验证安装后 EOS 三层输出可观测性 API 可被外部工程编译链接。
 *
 * 覆盖要点（阶段 9 三层输出）：
 *   - EosCycleResult.detection_attributions 仿真归属访问
 *   - EosOutputDebugViewBuilder 把输出 + 输入目标表合成开发可读视图
 *   - EosDetectionLifecycleRecorder 跨周期生命周期记录
 *
 * 本 consumer 不驱动完整 session 配置，只验证三层类型在安装后可达、
 * 方法签名稳定。是 eos_session_consumer 的可观测性补充。
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"

namespace eos = electro_optical_sensor;

int main() {
  // 1. 输入目标带 name（阶段 2 实体名称贯穿）。
  eos::session::EosCycleInput input;
  input.cycle_index = 1U;
  eos::session::EosSceneTarget target;
  target.target_id = 42U;
  target.target_name = "consumer-target";
  target.range_m = 1500.0f;
  input.scene.push_back(target);

  // 2. 手填一个含 attribution 的结果，验证 detection_attributions 字段可达。
  eos::session::EosCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.executed_this_cycle = true;
  result.output_frame.cycle_index = input.cycle_index;
  eos::attribution::EosDetectionAttributionRecord attribution;
  attribution.detection_id = 1U;
  attribution.target_id = 42U;
  attribution.target_name = "consumer-target";
  result.detection_attributions.push_back(attribution);

  // 3. Debug view：把输出 + 输入目标表合成为开发可读视图（name 经 attribution 回填）。
  const eos::session::EosOutputDebugView view =
      eos::session::EosOutputDebugViewBuilder::Build(input, result);
  if (view.targets.size() != 1U) {
    return 1;
  }
  if (view.targets[0].target_name != "consumer-target") {
    return 2;
  }

  // 4. Lifecycle recorder：调用方经 Session::Attach*Recorder 注册后由 Session 自动驱动
  //    （见 docs/common/session_contract.md 规则 10），不手动调用 Update()。
  //    本测试以合成 result 验证工具链，仅验证类型与 GetLastEvents 在安装后可达。
  eos::session::EosDetectionLifecycleRecorder recorder;
  (void)recorder.GetLastEvents();

  return 0;
}
