#include "1q/electronic_countermeasure/EcmTraceSession.h"

#include <sstream>
#include <string>
#include <utility>

#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "electronic_countermeasure/EcmReplayFlatbufferCodec.h"

namespace electronic_countermeasure {
namespace session {
namespace {

std::string BuildInputSummary(const EcmCycleInput& input) {
  std::ostringstream stream;
  stream << "{\"cycle_index\":" << input.cycle_index
         << ",\"input_mode\":" << static_cast<int>(input.input_mode)
         << ",\"sensor_observation_count\":"
         << input.sensor_observation_frame.observations.size()
         << ",\"truth_threat_count\":" << input.truth_threats.size() << "}";
  return stream.str();
}

std::string BuildResultSummary(const EcmCycleResult& result) {
  std::ostringstream stream;
  stream << "{\"cycle_index\":" << result.input_cycle_index
         << ",\"status\":" << static_cast<int>(result.status)
         << ",\"truth_assisted\":" << (result.truth_assisted ? "true" : "false")
         << ",\"emission_count\":" << result.emission_frame.emissions.size()
         << ",\"thermal_energy_j\":" << result.thermal_energy_j << "}";
  return stream.str();
}

}  // namespace

struct EcmTraceSession::Impl {
  Impl(config::EcmSessionConfig config, EcmTraceSessionOptions options)
      : session(EcmSession::Create(config)),
        sink(std::move(options.sink)),
        replay_writer(std::move(options.replay_writer)) {
    if (sink && options.trace_config_on_construct) {
      sink->Record("electronic_countermeasure", "config", "{}");
    }
    if (replay_writer && options.trace_config_on_construct) {
      WriteEvent("session_config", "EcmSessionConfig", EncodeEcmSessionConfig(config), false, 0U);
    }
  }

  void WriteEvent(const std::string& event_type, const std::string& payload_type,
                  const std::string& payload, bool has_cycle, std::uint32_t cycle_index) const {
    oneq::replay::ReplayTraceEvent event;
    event.module = "electronic_countermeasure";
    event.event_type = event_type;
    event.payload_type = payload_type;
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = payload;
    event.has_cycle_index = has_cycle;
    event.cycle_index = cycle_index;
    replay_writer->WriteEvent(event);
  }

  EcmSession session;
  std::shared_ptr<oneq::trace::TraceSink> sink;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer;
};

EcmTraceSession::EcmTraceSession(config::EcmSessionConfig config,
                                 EcmTraceSessionOptions options)
    : impl_(new Impl(std::move(config), std::move(options))) {}
EcmTraceSession::~EcmTraceSession() = default;
EcmTraceSession::EcmTraceSession(EcmTraceSession&&) noexcept = default;
EcmTraceSession& EcmTraceSession::operator=(EcmTraceSession&&) noexcept = default;

EcmCycleResult EcmTraceSession::StepWithResult(const EcmCycleInput& input) {
  if (impl_->sink) {
    impl_->sink->Record("electronic_countermeasure", "input", BuildInputSummary(input));
  }
  if (impl_->replay_writer) {
    impl_->WriteEvent("cycle_input", "EcmCycleInput", EncodeEcmCycleInput(input), true,
                      input.cycle_index);
  }
  const EcmCycleResult result = impl_->session.StepWithResult(input);
  if (impl_->sink) {
    impl_->sink->Record("electronic_countermeasure", "output", BuildResultSummary(result));
  }
  if (impl_->replay_writer) {
    impl_->WriteEvent("cycle_output", "EcmCycleResult", EncodeEcmCycleResult(result), true,
                      input.cycle_index);
  }
  return result;
}

EcmRuntimeConfigApplyResult EcmTraceSession::ApplyRuntimeConfig(
    const config::EcmRuntimeConfigPatch& patch) {
  if (impl_->replay_writer) {
    impl_->WriteEvent("runtime_config_patch", "EcmRuntimeConfigPatch",
                      EncodeEcmRuntimeConfigPatch(patch), false, 0U);
  }
  return impl_->session.ApplyRuntimeConfig(patch);
}

EcmSession& EcmTraceSession::session() { return impl_->session; }
const EcmSession& EcmTraceSession::session() const { return impl_->session; }

}  // namespace session
}  // namespace electronic_countermeasure
