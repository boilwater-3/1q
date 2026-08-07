#include "electronic_surveillance_radar/runtime/EsrController.h"

#include <memory>

#include "1q/coordinate/position_transform.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace electronic_surveillance_radar {
namespace extension {

struct EsrController::Impl {
  Impl(pipeline::InterceptPipeline& pipeline_ref,
       environment::IEsrEnvironmentService& environment_service_ref)
      : pipeline(pipeline_ref), environment_service(environment_service_ref) {}

  pipeline::InterceptPipeline& pipeline;
  environment::IEsrEnvironmentService& environment_service;
  oneq::common::runtime::RuntimeCycleState<session::EsrOutputFrame,
                                          session::EsrIssueList>
      runtime_state{};
  session::EsrCycleExecutionStatus last_cycle_status{
      session::EsrCycleExecutionStatus::kRejected};
  session::EsrPipelineAbortReason last_abort_reason{session::EsrPipelineAbortReason::kNone};
  session::EsrIssueList latest_issues{}; /**< 正常周期按发射源排除的 kInfo 诊断（规则 13b）。 */
};

EsrController::EsrController(pipeline::InterceptPipeline& pipeline,
                             environment::IEsrEnvironmentService& environment_service)
    : impl_(new Impl(pipeline, environment_service)) {}

EsrController::~EsrController() = default;

void EsrController::RunOnce(const session::EsrCycleInput& input) {
  const oneq::common::runtime::RuntimeCycleStamp stamp =
      oneq::common::runtime::MakeRuntimeCycleStamp(
          input.cycle_index, impl_->runtime_state.next_batch_id);

  // 校验
  session::EsrIssueList issues = session::ValidateEsrCycleInput(input);
  impl_->runtime_state.last_validation_issues = issues;

  if (session::HasValidationError(issues)) {
    impl_->last_cycle_status = session::EsrCycleExecutionStatus::kRejected;
    impl_->last_abort_reason = session::EsrPipelineAbortReason::kValidationRejected;
    // 中译：ESR 周期输入校验被拒绝（周期号）。
    // 标识：输入校验失败——本周期不执行、输出为空；
    //       排查输入字段校验问题（详见 EsrIssueList）。
    PROJECT_LOG_WARN("ESR validation rejected for cycle_index={}", stamp.cycle_index);
    return;
  }

  // 冻结环境
  {
    oneq::coordinate::LlaPositionDegM platform_lla;
    if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla)) {
      impl_->last_cycle_status = session::EsrCycleExecutionStatus::kRejected;
      impl_->last_abort_reason = session::EsrPipelineAbortReason::kValidationRejected;
      return;
    }
    impl_->environment_service.BeginCycle(
        stamp.cycle_index, input.dt_sec, static_cast<float>(platform_lla.altitude_m));
  }

  // 执行
  extension::InterceptPipelineResult pipeline_result =
      impl_->pipeline.RunCycle(input, impl_->environment_service);
  if (pipeline_result.rf_v2_rejected) {
    impl_->last_cycle_status = session::EsrCycleExecutionStatus::kRejected;
    impl_->last_abort_reason = session::EsrPipelineAbortReason::kRfReceiverRejected;
    // 中译：ESR RF v2 接收机拒绝了本周期（周期号）。
    // 标识：射频接收级拒绝——周期不产出观测；排查 RF 帧与接收机窗口。
    PROJECT_LOG_WARN("ESR RF v2 receiver rejected cycle_index={}", stamp.cycle_index);
    return;
  }
  if (pipeline_result.sensor_powered_off) {
    impl_->last_cycle_status = session::EsrCycleExecutionStatus::kPoweredOff;
    impl_->last_abort_reason = session::EsrPipelineAbortReason::kSensorPoweredOff;
    return;
  }
  session::EsrOutputFrame output_frame;
  output_frame.cycle_index = stamp.cycle_index;
  output_frame.batch_id = stamp.batch_id;
  output_frame.scan_azimuth_deg = pipeline_result.scan_azimuth_deg;
  output_frame.observation_output = std::move(pipeline_result.observation_output);
  output_frame.emitter_output = std::move(pipeline_result.emitter_output);
  // 规则 13b：正常周期按发射源排除的 kInfo 诊断转写（abort 路径不变）。
  impl_->latest_issues = std::move(pipeline_result.issues);

  impl_->runtime_state.latest_output = std::move(output_frame);
  impl_->runtime_state.has_latest_output = true;
  impl_->last_cycle_status = session::EsrCycleExecutionStatus::kCompleted;
  impl_->last_abort_reason = session::EsrPipelineAbortReason::kNone;
  ++impl_->runtime_state.next_batch_id;
  // 中译：本周期已执行（周期号、观测数、假设数）。
  // 标识：执行成功摘要——确认周期正常完成并产出了观测与假设。
  PROJECT_LOG_DEBUG(
      "[EsrController] cycle_index={} executed obs={} hyp={}",
      stamp.cycle_index,
      impl_->runtime_state.latest_output.observation_output.observations.size(),
      impl_->runtime_state.latest_output.emitter_output.hypotheses.size());
}

bool EsrController::HasLatestInterceptOutputFrame() const { return impl_->runtime_state.has_latest_output; }

const session::EsrOutputFrame& EsrController::GetLatestInterceptOutputFrame() const {
  return impl_->runtime_state.latest_output;
}

const session::EsrIssueList& EsrController::GetLastValidationIssues() const {
  return impl_->runtime_state.last_validation_issues;
}

session::EsrCycleExecutionStatus EsrController::GetLatestCycleStatus() const {
  return impl_->last_cycle_status;
}

const session::EsrIssueList& EsrController::GetLatestIssues() const {
  return impl_->latest_issues;
}

session::EsrPipelineAbortReason EsrController::GetLastInterceptCycleAbortReason() const {
  return impl_->last_abort_reason;
}

environment::IEsrEnvironmentService& EsrController::GetEnvironmentService() {
  return impl_->environment_service;
}

EsrControllerRuntimeState EsrController::CaptureRuntimeState() const {
  EsrControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 1U;
  state.has_latest_output = impl_->runtime_state.has_latest_output;
  state.latest_output = impl_->runtime_state.latest_output;
  state.last_validation_issues = impl_->runtime_state.last_validation_issues;
  state.next_batch_id = impl_->runtime_state.next_batch_id;
  state.last_cycle_status = impl_->last_cycle_status;
  state.last_abort_reason = impl_->last_abort_reason;
  return state;
}

bool EsrController::RestoreRuntimeState(const EsrControllerRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 1U) {
    return false;
  }
  impl_->runtime_state.has_latest_output = state.has_latest_output;
  impl_->runtime_state.latest_output = state.latest_output;
  impl_->runtime_state.last_validation_issues = state.last_validation_issues;
  impl_->runtime_state.next_batch_id = state.next_batch_id;
  impl_->last_cycle_status = state.last_cycle_status;
  impl_->last_abort_reason = state.last_abort_reason;
  return true;
}

}  // namespace extension

}  // namespace electronic_surveillance_radar
