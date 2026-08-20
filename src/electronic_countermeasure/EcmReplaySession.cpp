#include "1q/electronic_countermeasure/EcmReplaySession.h"

#include <memory>
#include <string>

#include "1q/electronic_countermeasure/EcmSession.h"
#include "electronic_countermeasure/EcmReplayFlatbufferCodec.h"

namespace electronic_countermeasure {
namespace session {
namespace {

struct ReplayState {
  std::unique_ptr<EcmSession> session{};
  EcmCycleInput pending_input{};
  bool has_pending_input{false};
};

bool OnSessionConfig(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                     std::string* error) {
  if (event.payload_type != "EcmSessionConfig") {
    *error = "ECM replay expected EcmSessionConfig";
    return false;
  }
  config::EcmSessionConfig config;
  if (!DecodeEcmSessionConfig(event.payload_bytes, &config)) {
    *error = "ECM replay failed to decode session config";
    return false;
  }
  ReplayState* state = static_cast<ReplayState*>(user_data);
  state->session.reset(new EcmSession(EcmSession::Create(config)));
  return true;
}

bool OnCycleInput(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                  std::string* error) {
  ReplayState* state = static_cast<ReplayState*>(user_data);
  if (!state->session || state->has_pending_input || event.payload_type != "EcmCycleInput" ||
      !DecodeEcmCycleInput(event.payload_bytes, &state->pending_input)) {
    *error = "ECM replay rejected cycle input ordering or payload";
    return false;
  }
  state->has_pending_input = true;
  return true;
}

bool OnRuntimeConfigPatch(const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
                          std::string* error) {
  ReplayState* state = static_cast<ReplayState*>(user_data);
  config::EcmRuntimeConfigPatch patch;
  if (!state->session || event.payload_type != "EcmRuntimeConfigPatch" ||
      !DecodeEcmRuntimeConfigPatch(event.payload_bytes, &patch)) {
    *error = "ECM replay rejected runtime config patch";
    return false;
  }
  if (!state->session->ApplyRuntimeConfig(patch).applied) {
    *error = "ECM replay runtime config patch was not applied";
    return false;
  }
  return true;
}

oneq::replay::ReplayTraceOutputStatus OnCycleOutput(
    const oneq::replay::ReplayTraceReadEvent& event, void* user_data,
    std::string* actual_output, std::string* error) {
  using oneq::replay::ReplayTraceOutputStatus;
  ReplayState* state = static_cast<ReplayState*>(user_data);
  if (!state->session || !state->has_pending_input || event.payload_type != "EcmCycleResult") {
    *error = "ECM replay rejected cycle output ordering or payload type";
    return ReplayTraceOutputStatus::kOtherFailure;
  }
  EcmCycleResult expected;
  if (!DecodeEcmCycleResult(event.payload_bytes, &expected)) {
    *error = "ECM replay failed to decode cycle result";
    return ReplayTraceOutputStatus::kOtherFailure;
  }
  const EcmCycleResult actual = state->session->StepWithResult(state->pending_input);
  state->has_pending_input = false;
  *actual_output = EncodeEcmCycleResult(actual);
  if (*actual_output != event.payload_bytes) {
    *error = "ECM replay output divergence";
    return ReplayTraceOutputStatus::kDivergence;
  }
  actual_output->clear();
  return ReplayTraceOutputStatus::kHandledByModule;
}

}  // namespace

EcmReplaySessionResult ReplayEcmTrace(const std::string& trace_dir) {
  EcmReplaySessionResult result;
  oneq::replay::ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "electronic_countermeasure";
  expectation.require_module_match = true;
  result.report = oneq::replay::BuildReplayTraceReport(trace_dir, expectation);
  if (!result.report.replay_ready) {
    result.first_error = result.report.first_error;
    return result;
  }

  ReplayState state;
  oneq::replay::ReplayTracePlaybackCallbacks callbacks;
  callbacks.user_data = &state;
  callbacks.on_session_config = OnSessionConfig;
  callbacks.on_cycle_input = OnCycleInput;
  callbacks.on_runtime_config_patch = OnRuntimeConfigPatch;
  callbacks.on_cycle_output = OnCycleOutput;
  oneq::replay::ReplayTracePlaybackOptions options;
  options.require_output_callback = true;
  options.stop_on_first_divergence = true;
  result.playback = oneq::replay::PlaybackReplayTrace(trace_dir, callbacks, options);
  result.ok = result.playback.ok && !state.has_pending_input;
  if (!result.ok) {
    result.first_error = state.has_pending_input
                             ? "ECM replay ended with pending cycle input"
                             : result.playback.first_error;
  }
  return result;
}

}  // namespace session
}  // namespace electronic_countermeasure
