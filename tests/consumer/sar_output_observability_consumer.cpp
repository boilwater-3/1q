/**
 * @file sar_output_observability_consumer.cpp
 * @brief 验证安装后 SAR 三层输出可观测性 API 可被外部工程编译链接。
 *
 * 覆盖要点（阶段 9 三层输出）：
 *   - SarProductDebugViewBuilder 把成像产品输出 + 输入点目标合成开发可读视图
 *   - SarProductLifecycleRecorder 按产品生命周期记录
 *   - 产品输出不含点目标真值标识（name 只在 debug view 的 point_targets 中）
 *
 * 同时补齐 SAR 模块此前缺失的 consumer 覆盖。
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarProductDebugView.h"
#include "1q/sar/session/SarProductLifecycleRecorder.h"

namespace sar_ns = sar;

int main() {
  // 1. 输入点目标带 name/id。
  sar::session::SarCycleInput input;
  input.cycle_index = 1U;
  sar::session::SarPointTarget target;
  target.target_id = 9U;
  target.target_name = "consumer-point";
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);

  // 2. 手填一个含产品输出的结果。
  sar::session::SarCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.status = sar::session::SarCycleStatus::kCompleted;
  result.output_frame.cycle_index = input.cycle_index;
  result.output_frame.completed_stage = sar::session::SarProcessingStage::kL1RdaImage;
  result.output_frame.has_l1_image = true;
  result.output_frame.estimated_snr_db = 18.0;

  // 3. Debug view：产品输出 + 点目标解释（name 只在 point_targets 中）。
  const sar::session::SarProductDebugView view =
      sar::session::SarProductDebugViewBuilder::Build(input, result);
  if (view.point_targets.size() != 1U) {
    return 1;
  }
  if (view.point_targets[0].target_name != "consumer-point") {
    return 2;
  }
  if (!view.has_l1_image) {
    return 3;
  }

  // 4. Lifecycle recorder：调用方经 Session::Attach*Recorder 注册后由 Session 自动驱动
  //    （见 docs/common/session_contract.md 规则 10），不手动调用 Update()。
  //    本测试以合成 result 验证工具链，仅验证类型与 GetLastEvents 在安装后可达。
  sar::session::SarProductLifecycleRecorder recorder;
  (void)recorder.GetLastEvents();

  return 0;
}
