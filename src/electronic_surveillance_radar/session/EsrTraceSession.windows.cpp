#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

#include <cstddef>
#include <sstream>
#include <string>

#include "common/trace/JsonFormatUtils.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

template <typename T>
std::string MakeFlatbuffersPayload(const char* type_name, const T& /*value*/) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"" << type_name << "\""
         << "}";
  return stream.str();
}

void AppendPoseJson(std::ostringstream& stream, const model::EsrPoseState& pose) {
  stream << "{"
         << "\"position_m\":[" << pose.position_m.x << "," << pose.position_m.y << ","
         << pose.position_m.z << "],"
         << "\"velocity_mps\":[" << pose.velocity_mps.x << "," << pose.velocity_mps.y << ","
         << pose.velocity_mps.z << "],"
         << "\"attitude_deg\":{"
         << "\"yaw_deg\":" << pose.attitude_deg.yaw_deg << ","
         << "\"pitch_deg\":" << pose.attitude_deg.pitch_deg << ","
         << "\"roll_deg\":" << pose.attitude_deg.roll_deg << "}"
         << "}";
}

std::string MakeInputPayload(const session::EsrCycleInput& input) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EsrCycleInput\","
         << "\"cycle_index\":" << input.cycle_index << ","
         << "\"dt_sec\":" << input.dt_sec << ","
         << "\"platform_pose\":";
  AppendPoseJson(stream, input.platform_pose);
  stream << ",\"scene_emitters\":[";
  for (std::size_t i = 0; i < input.scene_emitters.size(); ++i) {
    const model::EmitterTruthState& emitter = input.scene_emitters[i];
    if (i > 0U) {
      stream << ",";
    }
    stream << "{"
           << "\"emitter_id\":"
           << oneq::trace::internal::QuoteString(emitter.emitter_id) << ","
           << "\"pose\":";
    AppendPoseJson(stream, emitter.pose);
    stream << ","
           << "\"carrier_hz\":" << emitter.carrier_hz << ","
           << "\"bandwidth_hz\":" << emitter.bandwidth_hz << ","
           << "\"tx_power_w\":" << emitter.tx_power_w << ","
           << "\"pulse_width_s\":" << emitter.pulse_width_s << ","
           << "\"pri_s\":" << emitter.pri_s << ","
           << "\"beam_state\":{"
           << "\"center_az_deg\":" << emitter.beam_state.center_az_deg << ","
           << "\"center_el_deg\":" << emitter.beam_state.center_el_deg << ","
           << "\"az_beamwidth_deg\":" << emitter.beam_state.az_beamwidth_deg << ","
           << "\"el_beamwidth_deg\":" << emitter.beam_state.el_beamwidth_deg << ","
           << "\"beam_state_valid\":"
           << (emitter.beam_state.beam_state_valid ? "true" : "false") << "},"
           << "\"is_emitting\":" << (emitter.is_emitting ? "true" : "false")
           << "}";
  }
  const environment::EsrEnvironmentObservation& env = input.environment_observation;
  stream << "],"
         << "\"environment_observation\":{"
         << "\"propagation_profile\":" << static_cast<int>(env.propagation_profile) << ","
         << "\"clutter_density\":" << static_cast<int>(env.clutter_density) << ","
         << "\"spectrum_occupancy_ratio\":" << env.spectrum_occupancy_ratio << ","
         << "\"atmospheric_observation\":{"
         << "\"relative_humidity_ratio\":"
         << env.atmospheric_observation.relative_humidity_ratio << ","
         << "\"precipitation_rate_mmph\":"
         << env.atmospheric_observation.precipitation_rate_mmph << ","
         << "\"visibility_km\":" << env.atmospheric_observation.visibility_km << "},"
         << "\"jammer_sources\":[";
  for (std::size_t i = 0; i < env.jammer_sources.size(); ++i) {
    const environment::EsrJammerSource& jammer = env.jammer_sources[i];
    if (i > 0U) {
      stream << ",";
    }
    stream << "{"
           << "\"technique\":" << static_cast<int>(jammer.technique) << ","
           << "\"active\":" << (jammer.active ? "true" : "false") << ","
           << "\"center_hz\":" << jammer.center_hz << ","
           << "\"bandwidth_hz\":" << jammer.bandwidth_hz << ","
           << "\"power_w\":" << jammer.power_w << ","
           << "\"deception_risk\":" << jammer.deception_risk << ","
           << "\"confidence\":" << jammer.confidence << "}";
  }
  stream << "]}}";
  return stream.str();
}

std::string MakeOutputPayload(const output::EsrOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EsrOutputFrame\","
         << "\"observation_count\":"
         << output.observation_output.observations.size() << ","
         << "\"hypothesis_count\":" << output.emitter_output.hypotheses.size()
         << "}";
  return stream.str();
}

std::string MakeResultPayload(const session::EsrCycleResult& result) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EsrCycleResult\","
         << "\"has_validation_error\":" << (result.has_validation_error ? "true" : "false")
         << ","
         << "\"validation_issue_count\":" << result.validation_issues.size()
         << "}";
  return stream.str();
}

}  // namespace

EsrTraceSession::EsrTraceSession(session::EsrSessionConfig config, EsrTraceSessionOptions options)
    : session_(config),
      sink_(std::move(options.sink)),
      replay_writer_(std::move(options.replay_writer)) {
  if (replay_writer_ && options.trace_config_on_construct) {
    RecordReplay("session_config", "EsrSessionConfig",
                 MakeFlatbuffersPayload("EsrSessionConfig", config));
  }
  if (sink_ && options.trace_config_on_construct) {
    Record("config", MakeFlatbuffersPayload("EsrSessionConfig", config));
  }
}

output::EsrOutputFrame EsrTraceSession::Step(const session::EsrCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "EsrCycleInput", MakeInputPayload(input), input.cycle_index);
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const output::EsrOutputFrame output = session_.Step(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "EsrOutputFrame", MakeOutputPayload(output),
                 output.observation_output.cycle_index);
  }
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

session::EsrCycleResult EsrTraceSession::StepWithResult(const session::EsrCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "EsrCycleInput", MakeInputPayload(input), input.cycle_index);
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const session::EsrCycleResult output = session_.StepWithResult(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "EsrCycleResult", MakeResultPayload(output),
                 output.output_frame.observation_output.cycle_index);
  }
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

void EsrTraceSession::ApplyRuntimeConfig(const session::EsrRuntimeConfigPatch& patch) {
  if (replay_writer_) {
    RecordReplay("runtime_config_patch", "EsrRuntimeConfigPatch",
                 MakeFlatbuffersPayload("EsrRuntimeConfigPatch", patch));
  }
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", MakeFlatbuffersPayload("EsrRuntimeConfigPatch", patch));
  }
}

session::EsrSession& EsrTraceSession::session() { return session_; }

const session::EsrSession& EsrTraceSession::session() const { return session_; }

void EsrTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electronic_surveillance_radar", phase, payload_json);
}

void EsrTraceSession::RecordReplay(const std::string& event_type,
                                   const std::string& payload_type,
                                   const std::string& payload_json) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "electronic_surveillance_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  replay_writer_->WriteEvent(event);
}

void EsrTraceSession::RecordReplay(const std::string& event_type,
                                   const std::string& payload_type,
                                   const std::string& payload_json,
                                   std::uint32_t cycle_index) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "electronic_surveillance_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  event.has_cycle_index = true;
  event.cycle_index = cycle_index;
  replay_writer_->WriteEvent(event);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
