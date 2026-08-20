/**
 * @file eos_extension_consumer.cpp
 * @brief 验证安装后 EOS 扩展接口可被外部工程访问。
 *
 * 覆盖要点：
 *   - EosSession 构建（EosSession::Create 默认装配）、Step、StepWithResult、TryApplyRuntimeConfig
 *   - EosCycleResult::issues 问题列表可访问
 *   - EosPipelineAbortReason 公共结果类型可达
 *
 * 注：环境服务与管线已内部化，不再支持外部注入；本 consumer 仅验证安装后公共面可达。
 */

#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

int main() {
  // 1. Default session assembly
  electro_optical_sensor::session::EosSession session =
      electro_optical_sensor::session::EosSession::Create({});

  // 2. StepWithResult
  electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1f;  // 合法步长：受 53c56e21 收紧的 dt_sec <= 10/frame_rate_hz 上界约束（30Hz → ≈0.333s）。

  const electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);
  if (result.status != electro_optical_sensor::session::EosCycleStatus::kCompleted) {
    return 1;
  }
  (void)result.output_frame.detections.size();

  // 3. Step
  electro_optical_sensor::session::EosCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 0.1f;  // 同上，合法步长。
  const electro_optical_sensor::session::EosOutputFrame frame = session.Step(input_2);
  (void)frame.cycle_index;

  // 4. Runtime config patch
  electro_optical_sensor::config::EosRuntimeConfigPatch patch;
  patch.has_frame_rate_hz = true;
  patch.frame_rate_hz = 15.0f;
  (void)session.TryApplyRuntimeConfig(patch);

  // 5. Validation access
  const electro_optical_sensor::session::EosIssueList& issues =
      result.issues;
  (void)issues.size();

  // 6. Pipeline result types accessible
  electro_optical_sensor::session::EosPipelineAbortReason abort_reason{
      electro_optical_sensor::session::EosPipelineAbortReason::kNone};
  (void)abort_reason;

  return 0;
}
