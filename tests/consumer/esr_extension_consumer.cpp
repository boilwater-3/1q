/**
 * @file esr_extension_consumer.cpp
 * @brief 验证安装后 ESR 扩展接口可被外部工程实现并访问。
 *
 * 覆盖要点：
 *   - EsrPipelineAbortReason 公共结果类型可达
 *   - EsrSession 构建、Step、StepWithResult、ApplyRuntimeConfig
 *   - GetLastValidationIssues 字段可访问
 *
 * 注：环境服务与控制器已内部化，不再支持外部注入；本 consumer 仅验证安装后公共面可达。
 */

#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"

int main() {
  // 1. Default session assembly
  electronic_surveillance_radar::session::EsrSession session =
      electronic_surveillance_radar::session::EsrSession::Create({});

  // 2. StepWithResult
  electronic_surveillance_radar::session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const electronic_surveillance_radar::session::EsrCycleResult result = session.StepWithResult(input);
  if (result.status !=
      electronic_surveillance_radar::session::EsrCycleExecutionStatus::kCompleted) {
    return 1;
  }
  (void)result.output_frame.observation_output.observations.size();
  (void)result.output_frame.emitter_output.hypotheses.size();

  // 3. Step
  electronic_surveillance_radar::session::EsrCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 1.0f;
  const electronic_surveillance_radar::session::EsrOutputFrame frame = session.Step(input_2);
  (void)frame.cycle_index;

  // 4. Runtime config patch
  electronic_surveillance_radar::config::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = true;
  session.ApplyRuntimeConfig(patch);

  // 5. Validation access
  const electronic_surveillance_radar::session::ValidationIssueList& issues =
      result.validation_issues;
  (void)issues.size();

  // 6. EsrPipelineAbortReason accessible
  electronic_surveillance_radar::session::EsrPipelineAbortReason abort_reason{
      electronic_surveillance_radar::session::EsrPipelineAbortReason::kNone};
  (void)abort_reason;

  return 0;
}
