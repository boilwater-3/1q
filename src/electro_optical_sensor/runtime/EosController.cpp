#include "1q/electro_optical_sensor/extension/EosController.h"

#include <memory>

#include "1q/electro_optical_sensor/extension/IEosPipeline.h"

namespace electro_optical_sensor {
namespace extension {

namespace {

constexpr std::uint32_t kControllerRuntimeStateSchemaVersion = 1U;

bool IsCompatibleControllerRuntimeState(const extension::EosControllerRuntimeState& state,
																				const void* owner_identity) {
	return state.owner_identity == owner_identity &&
				 state.schema_version == kControllerRuntimeStateSchemaVersion;
}

extension::EosPipelineAbortReason NormalizeAbortReason(
		extension::EosPipelineAbortReason abort_reason) {
	if (abort_reason == extension::EosPipelineAbortReason::kNone) {
		return extension::EosPipelineAbortReason::kOutputContractViolation;
	}
	return abort_reason;
}

bool IsExecuteResultContractValid(const extension::EosPipelineExecuteResult& execute_result,
																	const session::EosCycleInput& input) {
	if (!execute_result.executed_this_cycle) {
		return false;
	}
	return execute_result.abort_reason == extension::EosPipelineAbortReason::kNone &&
				 execute_result.output_frame.cycle_index == input.cycle_index;
}

}  // namespace

struct EosController::Impl {
	explicit Impl(extension::IEosPipeline& pipeline_ref) : pipeline(pipeline_ref) {}

	extension::IEosPipeline& pipeline;
	output::EosOutputFrame latest_output{};
	model::EosValidationIssueList last_validation_issues{};
	bool has_latest_output{false};
	bool has_validation_error{false};
	bool last_cycle_executed{false};
	bool last_cycle_reused_previous_output{false};
	extension::EosPipelineAbortReason last_abort_reason{extension::EosPipelineAbortReason::kNone};
};

EosController::EosController(extension::IEosPipeline& pipeline) : impl_(new Impl(pipeline)) {}

EosController::~EosController() = default;

void EosController::RunOnce(const session::EosCycleInput& input) {
	const output::EosOutputFrame previous_output = impl_->latest_output;
	const bool had_previous_output = impl_->has_latest_output;
	const extension::EosPipelineRuntimeState previous_pipeline_state =
			impl_->pipeline.CaptureRuntimeState();

	impl_->last_cycle_executed = false;
	impl_->last_cycle_reused_previous_output = false;
	impl_->last_abort_reason = extension::EosPipelineAbortReason::kNone;
	impl_->last_validation_issues = model::ValidateEosCycleInput(input);
	impl_->has_validation_error = model::HasEosValidationError(impl_->last_validation_issues);
	if (impl_->has_validation_error) {
		impl_->last_abort_reason = extension::EosPipelineAbortReason::kValidationRejected;
		impl_->last_cycle_reused_previous_output = had_previous_output;
		impl_->latest_output = previous_output;
		impl_->has_latest_output = had_previous_output;
		return;
	}

	const extension::EosPipelineExecuteResult execute_result = impl_->pipeline.Execute(input);
	if (!IsExecuteResultContractValid(execute_result, input)) {
		const bool restore_ok = impl_->pipeline.RestoreRuntimeState(previous_pipeline_state);
		if (!restore_ok) {
			impl_->latest_output = output::EosOutputFrame{};
			impl_->has_latest_output = false;
			impl_->last_cycle_executed = false;
			impl_->last_cycle_reused_previous_output = false;
			impl_->last_abort_reason = extension::EosPipelineAbortReason::kRuntimeStateRestoreRejected;
			return;
		}
		impl_->latest_output = previous_output;
		impl_->has_latest_output = had_previous_output;
		impl_->last_cycle_executed = false;
		impl_->last_cycle_reused_previous_output = had_previous_output;
		impl_->last_abort_reason = NormalizeAbortReason(execute_result.abort_reason);
		return;
	}
	impl_->latest_output = execute_result.output_frame;
	impl_->has_latest_output = true;
	impl_->last_cycle_executed = true;
}

bool EosController::HasLatestOutputFrame() const { return impl_->has_latest_output; }

const output::EosOutputFrame& EosController::GetLatestOutputFrame() const {
	return impl_->latest_output;
}

const model::EosValidationIssueList& EosController::GetLastValidationIssues() const {
	return impl_->last_validation_issues;
}

bool EosController::HasValidationError() const { return impl_->has_validation_error; }

bool EosController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

bool EosController::ReusedPreviousOutputLatestCycle() const {
	return impl_->last_cycle_reused_previous_output;
}

extension::EosPipelineAbortReason EosController::GetLastAbortReason() const {
	return impl_->last_abort_reason;
}

model::EosCycleResult EosController::BuildCycleResult(
		const session::EosCycleInput& input) const {
	model::EosCycleResult result;
	result.validation_issues = impl_->last_validation_issues;
	result.has_validation_error = impl_->has_validation_error;
	result.executed_this_cycle = impl_->last_cycle_executed;
	result.reused_previous_output = impl_->last_cycle_reused_previous_output;
	result.abort_reason = impl_->last_abort_reason;
	if (impl_->has_latest_output) {
		result.output_frame = impl_->latest_output;
	} else {
		result.output_frame.cycle_index = input.cycle_index;
	}
	return result;
}

extension::IEosPipeline& EosController::GetPipeline() { return impl_->pipeline; }

extension::EosControllerRuntimeState EosController::CaptureRuntimeState() const {
	extension::EosControllerRuntimeState state;
	state.owner_identity = this;
	state.schema_version = kControllerRuntimeStateSchemaVersion;
	state.latest_output = impl_->latest_output;
	state.last_validation_issues = impl_->last_validation_issues;
	state.has_latest_output = impl_->has_latest_output;
	state.has_validation_error = impl_->has_validation_error;
	state.last_cycle_executed = impl_->last_cycle_executed;
	state.last_cycle_reused_previous_output = impl_->last_cycle_reused_previous_output;
	state.last_abort_reason = impl_->last_abort_reason;
	state.pipeline_state = impl_->pipeline.CaptureRuntimeState();
	return state;
}

void EosController::RestoreRuntimeState(const extension::EosControllerRuntimeState& state) {
	if (!IsCompatibleControllerRuntimeState(state, this)) {
		return;
	}
	if (!impl_->pipeline.RestoreRuntimeState(state.pipeline_state)) {
		return;
	}
	impl_->latest_output = state.latest_output;
	impl_->last_validation_issues = state.last_validation_issues;
	impl_->has_latest_output = state.has_latest_output;
	impl_->has_validation_error = state.has_validation_error;
	impl_->last_cycle_executed = state.last_cycle_executed;
	impl_->last_cycle_reused_previous_output = state.last_cycle_reused_previous_output;
	impl_->last_abort_reason = state.last_abort_reason;
}

}  // namespace extension
}  // namespace electro_optical_sensor
