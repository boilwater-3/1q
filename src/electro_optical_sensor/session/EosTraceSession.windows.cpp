#include "1q/electro_optical_sensor/session/EosTraceSession.h"

#include <cstddef>
#include <sstream>
#include <string>

namespace electro_optical_sensor {
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

std::string MakeInputPayload(const EosCycleInput& input) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EosCycleInput\","
         << "\"cycle_index\":" << input.cycle_index << ","
         << "\"dt_sec\":" << input.dt_sec << ","
         << "\"platform_pose\":{"
         << "\"position_m\":[" << input.platform_pose.position_m.x << ","
         << input.platform_pose.position_m.y << "," << input.platform_pose.position_m.z << "],"
         << "\"velocity_mps\":[" << input.platform_pose.velocity_mps.x << ","
         << input.platform_pose.velocity_mps.y << "," << input.platform_pose.velocity_mps.z
         << "],"
         << "\"attitude_deg\":{"
         << "\"yaw_deg\":" << input.platform_pose.attitude_deg.yaw_deg << ","
         << "\"pitch_deg\":" << input.platform_pose.attitude_deg.pitch_deg << ","
         << "\"roll_deg\":" << input.platform_pose.attitude_deg.roll_deg << "}"
         << "},"
         << "\"solar_altitude_deg\":" << input.solar_altitude_deg << ","
         << "\"solar_azimuth_deg\":" << input.solar_azimuth_deg << ","
         << "\"solar_irradiance_w_m2\":" << input.solar_irradiance_w_m2 << ","
         << "\"cloud_coverage_ratio\":" << input.cloud_coverage_ratio << ","
         << "\"ambient_wind_speed_mps\":" << input.ambient_wind_speed_mps << ","
         << "\"day_night_type\":" << static_cast<int>(input.day_night_type) << ","
         << "\"background_temperature_k\":" << input.background_temperature_k << ","
         << "\"scene_targets\":[";
  for (std::size_t i = 0; i < input.scene_targets.size(); ++i) {
    const EosTargetState& target = input.scene_targets[i];
    if (i > 0U) {
      stream << ",";
    }
    stream << "{"
           << "\"target_id\":" << target.target_id << ","
           << "\"range_m\":" << target.range_m << ","
           << "\"azimuth_deg\":" << target.azimuth_deg << ","
           << "\"elevation_deg\":" << target.elevation_deg << ","
           << "\"apparent_temperature_k\":" << target.apparent_temperature_k << ","
           << "\"emissivity\":" << target.emissivity << ","
           << "\"reflectance\":" << target.reflectance << ","
           << "\"projected_area_m2\":" << target.projected_area_m2 << "}";
  }
  stream << "]}";
  return stream.str();
}

std::string MakeOutputPayload(const output::EosOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EosOutputFrame\","
         << "\"cycle_index\":" << output.cycle_index << ","
         << "\"detection_count\":" << output.detections.size()
         << "}";
  return stream.str();
}

std::string MakeResultPayload(const model::EosCycleResult& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"EosCycleResult\","
         << "\"has_validation_error\":" << (output.has_validation_error ? "true" : "false")
         << ","
         << "\"executed_this_cycle\":" << (output.executed_this_cycle ? "true" : "false")
         << "}";
  return stream.str();
}

}  // namespace

EosTraceSession::EosTraceSession(EosSessionConfig config, EosTraceSessionOptions options)
    : session_(EosSessionFactory::Create(config)),
      sink_(std::move(options.sink)),
      replay_writer_(std::move(options.replay_writer)) {
  if (replay_writer_ && options.trace_config_on_construct) {
    RecordReplay("session_config", "EosSessionConfig",
                 MakeFlatbuffersPayload("EosSessionConfig", config));
  }
  if (sink_ && options.trace_config_on_construct) {
    Record("config", MakeFlatbuffersPayload("EosSessionConfig", config));
  }
}

output::EosOutputFrame EosTraceSession::Step(const EosCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "EosCycleInput", MakeInputPayload(input), input.cycle_index);
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const output::EosOutputFrame output = session_.Step(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "EosOutputFrame", MakeOutputPayload(output),
                 output.cycle_index);
  }
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

model::EosCycleResult EosTraceSession::StepWithResult(const EosCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "EosCycleInput", MakeInputPayload(input), input.cycle_index);
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const model::EosCycleResult output = session_.StepWithResult(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "EosCycleResult", MakeResultPayload(output),
                 output.output_frame.cycle_index);
  }
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

void EosTraceSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  if (replay_writer_) {
    RecordReplay("runtime_config_patch", "EosRuntimeConfigPatch",
                 MakeFlatbuffersPayload("EosRuntimeConfigPatch", patch));
  }
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", MakeFlatbuffersPayload("EosRuntimeConfigPatch", patch));
  }
}

EosSession& EosTraceSession::session() { return session_; }

const EosSession& EosTraceSession::session() const { return session_; }

void EosTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electro_optical_sensor", phase, payload_json);
}

void EosTraceSession::RecordReplay(const std::string& event_type,
                                   const std::string& payload_type,
                                   const std::string& payload_json) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "electro_optical_sensor";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  replay_writer_->WriteEvent(event);
}

void EosTraceSession::RecordReplay(const std::string& event_type,
                                   const std::string& payload_type,
                                   const std::string& payload_json,
                                   std::uint32_t cycle_index) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "electro_optical_sensor";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  event.has_cycle_index = true;
  event.cycle_index = cycle_index;
  replay_writer_->WriteEvent(event);
}

}  // namespace session
}  // namespace electro_optical_sensor
