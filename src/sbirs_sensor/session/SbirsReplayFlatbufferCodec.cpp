#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "sbirs_sensor/session/generated/sbirs_replay_generated.h"
#include "sbirs_sensor/session/generated/sbirs_session_replay_generated.h"

namespace sbirs_sensor {
namespace session {
namespace {

std::string FinishToString(const flatbuffers::FlatBufferBuilder& fbb) {
  const std::uint8_t* buffer = fbb.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer), fbb.GetSize());
}

sbirs::replay::Vec3d ToFbVec3(const SbirsVector3M& v) {
  return sbirs::replay::Vec3d(v.x, v.y, v.z);
}

SbirsVector3M FromFbVec3(const sbirs::replay::Vec3d* fb) {
  SbirsVector3M out;
  if (fb != nullptr) {
    out.x = fb->x();
    out.y = fb->y();
    out.z = fb->z();
  }
  return out;
}

flatbuffers::Offset<sbirs::replay::SbirsCycleEnvironmentConfig> EncodeCycleEnvironmentConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsEnvironmentConfig& value) {
  return sbirs::replay::CreateSbirsCycleEnvironmentConfig(
      fbb, static_cast<std::int32_t>(value.weather_type),
      static_cast<std::int32_t>(value.sea_state), value.temperature_c,
      value.relative_humidity_percent, value.visibility_km, value.base_atmospheric_transmittance,
      value.humidity_visibility_interaction_weight, value.rain_humidity_interaction_weight);
}

config::SbirsEnvironmentConfig DecodeCycleEnvironmentConfig(
    const sbirs::replay::SbirsCycleEnvironmentConfig* fb) {
  config::SbirsEnvironmentConfig out;
  if (fb != nullptr) {
    out.weather_type = static_cast<config::SbirsWeatherType>(fb->weather_type());
    out.sea_state = static_cast<config::SbirsSeaState>(fb->sea_state());
    out.temperature_c = fb->temperature_c();
    out.relative_humidity_percent = fb->relative_humidity_percent();
    out.visibility_km = fb->visibility_km();
    out.base_atmospheric_transmittance = fb->base_atmospheric_transmittance();
    out.humidity_visibility_interaction_weight = fb->humidity_visibility_interaction_weight();
    out.rain_humidity_interaction_weight = fb->rain_humidity_interaction_weight();
  }
  return out;
}

flatbuffers::Offset<sbirs::replay::SbirsEnvironmentConfig> EncodeSessionEnvironmentConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsEnvironmentConfig& value) {
  return sbirs::replay::CreateSbirsEnvironmentConfig(
      fbb, static_cast<std::int32_t>(value.weather_type),
      static_cast<std::int32_t>(value.sea_state), value.temperature_c,
      value.relative_humidity_percent, value.visibility_km, value.base_atmospheric_transmittance,
      value.humidity_visibility_interaction_weight, value.rain_humidity_interaction_weight);
}

config::SbirsEnvironmentConfig DecodeSessionEnvironmentConfig(
    const sbirs::replay::SbirsEnvironmentConfig* fb) {
  config::SbirsEnvironmentConfig out;
  if (fb != nullptr) {
    out.weather_type = static_cast<config::SbirsWeatherType>(fb->weather_type());
    out.sea_state = static_cast<config::SbirsSeaState>(fb->sea_state());
    out.temperature_c = fb->temperature_c();
    out.relative_humidity_percent = fb->relative_humidity_percent();
    out.visibility_km = fb->visibility_km();
    out.base_atmospheric_transmittance = fb->base_atmospheric_transmittance();
    out.humidity_visibility_interaction_weight = fb->humidity_visibility_interaction_weight();
    out.rain_humidity_interaction_weight = fb->rain_humidity_interaction_weight();
  }
  return out;
}

flatbuffers::Offset<sbirs::replay::SbirsDetectionRecord> EncodeOneDetection(
    flatbuffers::FlatBufferBuilder& fbb, const output::SbirsDetectionRecord& value) {
  return sbirs::replay::CreateSbirsDetectionRecord(
      fbb, value.detection_id, value.azimuth_deg, value.elevation_deg, value.infrared_snr_linear,
      static_cast<std::int32_t>(value.observation_stage), value.detected);
}

void DecodeOneDetection(const sbirs::replay::SbirsDetectionRecord& fb,
                        output::SbirsDetectionRecord* out) {
  out->detection_id = fb.detection_id();
  out->azimuth_deg = fb.azimuth_deg();
  out->elevation_deg = fb.elevation_deg();
  out->infrared_snr_linear = fb.infrared_snr_linear();
  out->observation_stage = static_cast<output::SbirsObservationStage>(fb.observation_stage());
  out->detected = fb.detected();
}

flatbuffers::Offset<sbirs::replay::SbirsOutputFrame> EncodeOutputFrameTable(
    flatbuffers::FlatBufferBuilder& fbb, const SbirsOutputFrame& value) {
  std::vector<flatbuffers::Offset<sbirs::replay::SbirsDetectionRecord>> detections;
  detections.reserve(value.detections.size());
  for (const output::SbirsDetectionRecord& detection : value.detections) {
    detections.push_back(EncodeOneDetection(fbb, detection));
  }
  return sbirs::replay::CreateSbirsOutputFrame(fbb, value.cycle_index, value.scan_azimuth_deg,
                                               fbb.CreateVector(detections));
}

void DecodeOutputFrameTable(const sbirs::replay::SbirsOutputFrame* fb, SbirsOutputFrame* out) {
  if (fb == nullptr) {
    return;
  }
  out->cycle_index = fb->cycle_index();
  out->scan_azimuth_deg = fb->scan_azimuth_deg();
  out->detections.clear();
  if (fb->detections() != nullptr) {
    for (const sbirs::replay::SbirsDetectionRecord* detection : *fb->detections()) {
      output::SbirsDetectionRecord record;
      DecodeOneDetection(*detection, &record);
      out->detections.push_back(record);
    }
  }
}

flatbuffers::Offset<sbirs::replay::SbirsHardwareConfig> EncodeHardwareConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsHardwareConfig& value) {
  return sbirs::replay::CreateSbirsHardwareConfig(
      fbb, value.wavelength_lower_um, value.wavelength_upper_um, value.optical_aperture_m,
      value.detector_area_m2, value.optical_transmission, value.detector_quantum_efficiency,
      value.integration_time_sec, value.noise_equivalent_power_w,
      value.background_radiance_w_sr_m2, value.detector_temperature_k, value.readout_noise_rms_w);
}

void DecodeHardwareConfig(const sbirs::replay::SbirsHardwareConfig* fb,
                          config::SbirsHardwareConfig* out) {
  if (fb == nullptr) {
    return;
  }
  out->wavelength_lower_um = fb->wavelength_lower_um();
  out->wavelength_upper_um = fb->wavelength_upper_um();
  out->optical_aperture_m = fb->optical_aperture_m();
  out->detector_area_m2 = fb->detector_area_m2();
  out->optical_transmission = fb->optical_transmission();
  out->detector_quantum_efficiency = fb->detector_quantum_efficiency();
  out->integration_time_sec = fb->integration_time_sec();
  out->noise_equivalent_power_w = fb->noise_equivalent_power_w();
  out->background_radiance_w_sr_m2 = fb->background_radiance_w_sr_m2();
  out->detector_temperature_k = fb->detector_temperature_k();
  out->readout_noise_rms_w = fb->readout_noise_rms_w();
}

flatbuffers::Offset<sbirs::replay::SbirsMissionConfig> EncodeMissionConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsMissionConfig& value) {
  return sbirs::replay::CreateSbirsMissionConfig(
      fbb, static_cast<std::int32_t>(value.work_mode), value.sensor_enabled,
      value.wide_field_fov_az_deg, value.wide_field_fov_el_deg, value.narrow_field_fov_az_deg,
      value.narrow_field_fov_el_deg, value.scan_start_az_deg, value.scan_end_az_deg,
      value.scan_center_el_deg, value.scan_rate_deg_per_sec, value.min_range_m, value.max_range_m,
      value.frame_rate_hz, value.narrow_cue_latency_s, value.narrow_pointing_settle_error_deg);
}

void DecodeMissionConfig(const sbirs::replay::SbirsMissionConfig* fb,
                         config::SbirsMissionConfig* out) {
  if (fb == nullptr) {
    return;
  }
  out->work_mode = static_cast<config::SbirsWorkMode>(fb->work_mode());
  out->sensor_enabled = fb->sensor_enabled();
  out->wide_field_fov_az_deg = fb->wide_field_fov_az_deg();
  out->wide_field_fov_el_deg = fb->wide_field_fov_el_deg();
  out->narrow_field_fov_az_deg = fb->narrow_field_fov_az_deg();
  out->narrow_field_fov_el_deg = fb->narrow_field_fov_el_deg();
  out->scan_start_az_deg = fb->scan_start_az_deg();
  out->scan_end_az_deg = fb->scan_end_az_deg();
  out->scan_center_el_deg = fb->scan_center_el_deg();
  out->scan_rate_deg_per_sec = fb->scan_rate_deg_per_sec();
  out->min_range_m = fb->min_range_m();
  out->max_range_m = fb->max_range_m();
  out->frame_rate_hz = fb->frame_rate_hz();
  out->narrow_cue_latency_s = fb->narrow_cue_latency_s();
  out->narrow_pointing_settle_error_deg = fb->narrow_pointing_settle_error_deg();
}

flatbuffers::Offset<sbirs::replay::SbirsPolicyConfig> EncodePolicyConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SbirsPolicyConfig& value) {
  const flatbuffers::Offset<sbirs::replay::SbirsDetectionPolicyConfig> detection =
      sbirs::replay::CreateSbirsDetectionPolicyConfig(fbb, value.detection.wide_min_snr_linear,
                                                      value.detection.narrow_min_snr_linear);
  const flatbuffers::Offset<sbirs::replay::SbirsErrorModelConfig> error =
      sbirs::replay::CreateSbirsErrorModelConfig(
          fbb, value.error_model.angular_sigma_deg, value.error_model.range_fraction_sigma,
          value.error_model.random_seed, value.error_model.orbit_sigma_deg,
          value.error_model.attitude_sigma_deg, value.error_model.fov_sigma_deg,
          value.error_model.detector_bandwidth_hz);
  const flatbuffers::Offset<sbirs::replay::SbirsSchedulerConfig> scheduler =
      sbirs::replay::CreateSbirsSchedulerConfig(fbb, value.scheduler.single_narrow_resource);
  return sbirs::replay::CreateSbirsPolicyConfig(fbb, detection, error, scheduler);
}

void DecodePolicyConfig(const sbirs::replay::SbirsPolicyConfig* fb,
                        config::SbirsPolicyConfig* out) {
  if (fb == nullptr) {
    return;
  }
  if (fb->detection() != nullptr) {
    out->detection.wide_min_snr_linear = fb->detection()->wide_min_snr_linear();
    out->detection.narrow_min_snr_linear = fb->detection()->narrow_min_snr_linear();
  }
  if (fb->error_model() != nullptr) {
    out->error_model.angular_sigma_deg = fb->error_model()->angular_sigma_deg();
    out->error_model.range_fraction_sigma = fb->error_model()->range_fraction_sigma();
    out->error_model.random_seed = fb->error_model()->random_seed();
    out->error_model.orbit_sigma_deg = fb->error_model()->orbit_sigma_deg();
    out->error_model.attitude_sigma_deg = fb->error_model()->attitude_sigma_deg();
    out->error_model.fov_sigma_deg = fb->error_model()->fov_sigma_deg();
    out->error_model.detector_bandwidth_hz = fb->error_model()->detector_bandwidth_hz();
  }
  if (fb->scheduler() != nullptr) {
    out->scheduler.single_narrow_resource = fb->scheduler()->single_narrow_resource();
  }
}

}  // namespace

std::string EncodeSbirsCycleInput(const SbirsCycleInput& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  std::vector<flatbuffers::Offset<sbirs::replay::SbirsSceneTarget>> targets;
  targets.reserve(value.scene.size());
  for (const SbirsSceneTarget& target : value.scene) {
    const flatbuffers::Offset<flatbuffers::String> name = fbb.CreateString(target.target_name);
    const sbirs::replay::Vec3d position = ToFbVec3(target.position_ecef_m);
    targets.push_back(sbirs::replay::CreateSbirsSceneTarget(
        fbb, target.target_id, name, &position, target.temperature_k, target.emissivity,
        target.projected_area_m2, target.active));
  }

  const sbirs::replay::Vec3d satellite = ToFbVec3(value.satellite_position_ecef_m);
  const flatbuffers::Offset<sbirs::replay::SbirsEnvironmentInput> environment =
      sbirs::replay::CreateSbirsEnvironmentInput(
          fbb, value.environment.has_environment_override,
          EncodeCycleEnvironmentConfig(fbb, value.environment.environment));
  fbb.Finish(sbirs::replay::CreateSbirsCycleInput(fbb, value.cycle_index, value.dt_sec,
                                                  value.has_satellite_position, &satellite,
                                                  environment, fbb.CreateVector(targets)));
  return FinishToString(fbb);
}

bool DecodeSbirsCycleInput(const std::string& bytes, SbirsCycleInput* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr || !verifier.VerifyBuffer<sbirs::replay::SbirsCycleInput>()) {
    return false;
  }
  const sbirs::replay::SbirsCycleInput* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsCycleInput>(bytes.data());
  out->cycle_index = fb->cycle_index();
  out->dt_sec = fb->dt_sec();
  out->has_satellite_position = fb->has_satellite_position();
  out->satellite_position_ecef_m = FromFbVec3(fb->satellite_position_ecef_m());
  out->environment = {};
  if (fb->environment() != nullptr) {
    out->environment.has_environment_override = fb->environment()->has_environment_override();
    out->environment.environment = DecodeCycleEnvironmentConfig(fb->environment()->environment());
  }
  out->scene.clear();
  if (fb->scene_targets() != nullptr) {
    for (const sbirs::replay::SbirsSceneTarget* target : *fb->scene_targets()) {
      SbirsSceneTarget item;
      item.target_id = target->target_id();
      item.target_name = target->target_name() ? target->target_name()->str() : std::string();
      item.position_ecef_m = FromFbVec3(target->position_ecef_m());
      item.temperature_k = target->temperature_k();
      item.emissivity = target->emissivity();
      item.projected_area_m2 = target->projected_area_m2();
      item.active = target->active();
      out->scene.push_back(item);
    }
  }
  return true;
}

std::string EncodeSbirsOutputFrame(const SbirsOutputFrame& value) {
  flatbuffers::FlatBufferBuilder fbb(256);
  fbb.Finish(EncodeOutputFrameTable(fbb, value));
  return FinishToString(fbb);
}

bool DecodeSbirsOutputFrame(const std::string& bytes, SbirsOutputFrame* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr || !verifier.VerifyBuffer<sbirs::replay::SbirsOutputFrame>()) {
    return false;
  }
  DecodeOutputFrameTable(flatbuffers::GetRoot<sbirs::replay::SbirsOutputFrame>(bytes.data()), out);
  return true;
}

std::string EncodeSbirsCycleResult(const SbirsCycleResult& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  const flatbuffers::Offset<sbirs::replay::SbirsOutputFrame> frame =
      EncodeOutputFrameTable(fbb, value.output_frame);

  std::vector<flatbuffers::Offset<sbirs::replay::SbirsDetectionAttributionRecord>> attributions;
  attributions.reserve(value.detection_attributions.size());
  for (const attribution::SbirsDetectionAttributionRecord& attribution :
       value.detection_attributions) {
    attributions.push_back(sbirs::replay::CreateSbirsDetectionAttributionRecord(
        fbb, attribution.detection_id, attribution.target_id,
        fbb.CreateString(attribution.target_name), attribution.estimated_range_m,
        attribution.used_truth_assist));
  }

  std::vector<flatbuffers::Offset<sbirs::replay::ValidationIssue>> issues;
  issues.reserve(value.validation_issues.size());
  for (const ValidationIssue& issue : value.validation_issues) {
    std::int64_t entity_index = -1;
    if (issue.location.entity_index != std::numeric_limits<std::size_t>::max()) {
      entity_index = static_cast<std::int64_t>(issue.location.entity_index);
    }
    issues.push_back(
        sbirs::replay::CreateValidationIssue(fbb, static_cast<std::int32_t>(issue.severity),
                                             static_cast<std::int32_t>(issue.location.kind),
                                             entity_index, fbb.CreateString(issue.message)));
  }

  fbb.Finish(sbirs::replay::CreateSbirsCycleResult(
      fbb, value.input_cycle_index, frame, fbb.CreateVector(attributions), fbb.CreateVector(issues),
      value.has_validation_error, value.executed_this_cycle, value.reused_previous_output,
      static_cast<std::int32_t>(value.abort_reason)));
  return FinishToString(fbb);
}

bool DecodeSbirsCycleResult(const std::string& bytes, SbirsCycleResult* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr || !verifier.VerifyBuffer<sbirs::replay::SbirsCycleResult>()) {
    return false;
  }
  const sbirs::replay::SbirsCycleResult* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsCycleResult>(bytes.data());
  out->input_cycle_index = fb->input_cycle_index();
  DecodeOutputFrameTable(fb->output_frame(), &out->output_frame);
  out->detection_attributions.clear();
  if (fb->detection_attributions() != nullptr) {
    for (const sbirs::replay::SbirsDetectionAttributionRecord* attr :
         *fb->detection_attributions()) {
      attribution::SbirsDetectionAttributionRecord record;
      record.detection_id = attr->detection_id();
      record.target_id = attr->target_id();
      record.target_name = attr->target_name() ? attr->target_name()->str() : std::string();
      record.estimated_range_m = attr->estimated_range_m();
      record.used_truth_assist = attr->used_truth_assist();
      out->detection_attributions.push_back(record);
    }
  }
  out->validation_issues.clear();
  if (fb->validation_issues() != nullptr) {
    for (const sbirs::replay::ValidationIssue* issue : *fb->validation_issues()) {
      ValidationIssue item;
      item.severity = static_cast<ValidationSeverity>(issue->severity());
      item.location.kind = static_cast<ValidationLocationKind>(issue->location_kind());
      item.location.entity_index = issue->entity_index() < 0
                                       ? std::numeric_limits<std::size_t>::max()
                                       : static_cast<std::size_t>(issue->entity_index());
      item.message = issue->message() ? issue->message()->str() : std::string();
      out->validation_issues.push_back(item);
    }
  }
  out->has_validation_error = fb->has_validation_error();
  out->executed_this_cycle = fb->executed_this_cycle();
  out->reused_previous_output = fb->reused_previous_output();
  out->abort_reason = static_cast<SbirsPipelineAbortReason>(fb->abort_reason());
  return true;
}

std::string EncodeSbirsSessionConfig(const config::SbirsSessionConfig& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  fbb.Finish(sbirs::replay::CreateSbirsSessionConfig(
      fbb, EncodeHardwareConfig(fbb, value.hardware), EncodeMissionConfig(fbb, value.mission),
      EncodePolicyConfig(fbb, value.policy),
      EncodeSessionEnvironmentConfig(fbb, value.environment)));
  return FinishToString(fbb);
}

bool DecodeSbirsSessionConfig(const std::string& bytes, config::SbirsSessionConfig* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr || !verifier.VerifyBuffer<sbirs::replay::SbirsSessionConfig>()) {
    return false;
  }
  const sbirs::replay::SbirsSessionConfig* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsSessionConfig>(bytes.data());
  DecodeHardwareConfig(fb->hardware(), &out->hardware);
  DecodeMissionConfig(fb->mission(), &out->mission);
  DecodePolicyConfig(fb->policy(), &out->policy);
  out->environment = DecodeSessionEnvironmentConfig(fb->environment());
  return true;
}

std::string EncodeSbirsRuntimeConfigPatch(const config::SbirsRuntimeConfigPatch& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  fbb.Finish(sbirs::replay::CreateSbirsRuntimeConfigPatch(
      fbb, value.has_mission, EncodeMissionConfig(fbb, value.mission), value.has_policy,
      EncodePolicyConfig(fbb, value.policy), value.has_environment,
      EncodeSessionEnvironmentConfig(fbb, value.environment), value.has_work_mode,
      static_cast<std::int32_t>(value.work_mode), value.has_scan_rate_deg_per_sec,
      value.scan_rate_deg_per_sec, value.has_sensor_enabled, value.sensor_enabled));
  return FinishToString(fbb);
}

bool DecodeSbirsRuntimeConfigPatch(const std::string& bytes, config::SbirsRuntimeConfigPatch* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (out == nullptr || !verifier.VerifyBuffer<sbirs::replay::SbirsRuntimeConfigPatch>()) {
    return false;
  }
  const sbirs::replay::SbirsRuntimeConfigPatch* fb =
      flatbuffers::GetRoot<sbirs::replay::SbirsRuntimeConfigPatch>(bytes.data());
  out->has_mission = fb->has_mission();
  out->has_policy = fb->has_policy();
  out->has_environment = fb->has_environment();
  out->has_work_mode = fb->has_work_mode();
  out->work_mode = static_cast<config::SbirsWorkMode>(fb->work_mode());
  out->has_scan_rate_deg_per_sec = fb->has_scan_rate_deg_per_sec();
  out->scan_rate_deg_per_sec = fb->scan_rate_deg_per_sec();
  out->has_sensor_enabled = fb->has_sensor_enabled();
  out->sensor_enabled = fb->sensor_enabled();
  DecodeMissionConfig(fb->mission(), &out->mission);
  DecodePolicyConfig(fb->policy(), &out->policy);
  out->environment = DecodeSessionEnvironmentConfig(fb->environment());
  return true;
}

std::string EncodeSbirsFailureMarker(const oneq::replay::ReplayTraceFailure& failure) {
  flatbuffers::FlatBufferBuilder fbb(256);
  fbb.Finish(sbirs::replay::CreateFailureMarkerDirect(
      fbb, failure.error_code.c_str(), failure.message.c_str(), failure.location.c_str(),
      failure.has_cycle_index, failure.cycle_index, failure.has_sim_time_sec, failure.sim_time_sec,
      failure.diagnostics_payload.c_str(), false, 0U));
  return FinishToString(fbb);
}

bool DecodeSbirsFailureMarker(const std::string& bytes, oneq::replay::ReplayTraceFailure* failure,
                              std::string* error) {
  if (failure == nullptr) {
    if (error != nullptr) {
      *error = "null FailureMarker output";
    }
    return false;
  }
  if (bytes.empty()) {
    if (error != nullptr) {
      *error = "empty FailureMarker flatbuffers payload";
    }
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<sbirs::replay::FailureMarker>()) {
    if (error != nullptr) {
      *error = "invalid FailureMarker flatbuffers payload";
    }
    return false;
  }
  const sbirs::replay::FailureMarker* fb =
      flatbuffers::GetRoot<sbirs::replay::FailureMarker>(bytes.data());
  failure->error_code = fb->error_code() ? fb->error_code()->str() : "";
  failure->message = fb->message() ? fb->message()->str() : "";
  failure->location = fb->location() ? fb->location()->str() : "";
  failure->has_cycle_index = fb->has_cycle_index();
  failure->cycle_index = fb->cycle_index();
  failure->has_sim_time_sec = fb->has_sim_time_sec();
  failure->sim_time_sec = fb->sim_time_sec();
  failure->diagnostics_payload = fb->diagnostics() ? fb->diagnostics()->str() : "{}";
  return true;
}

}  // namespace session
}  // namespace sbirs_sensor
