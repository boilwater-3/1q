#include "1q/electro_optical_sensor/session/EosTraceSession.h"

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace electro_optical_sensor {
namespace session {
namespace {

using Json = nlohmann::ordered_json;

template <typename T, typename Serializer>
Json SerializeArray(const std::vector<T>& values, const Serializer& serializer) {
  Json result = Json::array();
  for (std::size_t i = 0; i < values.size(); ++i) {
    result.push_back(serializer(values[i]));
  }
  return result;
}

Json BuildJson(const oneq::foundation::Vector3f& value) {
  Json json = Json::array();
  json.push_back(value.x);
  json.push_back(value.y);
  json.push_back(value.z);
  return json;
}

Json BuildJson(const oneq::foundation::EulerAnglesDeg& value) {
  Json json;
  json["yaw_deg"] = value.yaw_deg;
  json["pitch_deg"] = value.pitch_deg;
  json["roll_deg"] = value.roll_deg;
  return json;
}

Json BuildJson(const oneq::foundation::PoseState& value) {
  Json json;
  json["position_m"] = BuildJson(value.position_m);
  json["velocity_mps"] = BuildJson(value.velocity_mps);
  json["attitude_deg"] = BuildJson(value.attitude_deg);
  return json;
}

Json BuildJson(const EosTargetState& value) {
  Json json;
  json["target_id"] = value.target_id;
  json["range_m"] = value.range_m;
  json["azimuth_deg"] = value.azimuth_deg;
  json["elevation_deg"] = value.elevation_deg;
  json["apparent_temperature_k"] = value.apparent_temperature_k;
  json["emissivity"] = value.emissivity;
  json["reflectance"] = value.reflectance;
  json["projected_area_m2"] = value.projected_area_m2;
  return json;
}

Json BuildJson(const EosCycleInput& value) {
  Json json;
  json["cycle_index"] = value.cycle_index;
  json["dt_sec"] = value.dt_sec;
  json["platform_pose"] = BuildJson(value.platform_pose);
  json["solar_altitude_deg"] = value.solar_altitude_deg;
  json["solar_azimuth_deg"] = value.solar_azimuth_deg;
  json["solar_irradiance_w_m2"] = value.solar_irradiance_w_m2;
  json["atmospheric_transmittance"] = value.atmospheric_transmittance;
  json["cloud_coverage_ratio"] = value.cloud_coverage_ratio;
  json["day_night_type"] = static_cast<int>(value.day_night_type);
  json["background_temperature_k"] = value.background_temperature_k;
  json["scene_targets"] =
      SerializeArray(value.scene_targets, [](const EosTargetState& target) {
        return BuildJson(target);
      });
  return json;
}

Json BuildJson(const output::EosDetectionRecord& value) {
  Json json;
  json["target_id"] = value.target_id;
  json["range_m"] = value.range_m;
  json["azimuth_deg"] = value.azimuth_deg;
  json["elevation_deg"] = value.elevation_deg;
  json["infrared_snr_linear"] = value.infrared_snr_linear;
  json["visible_snr_linear"] = value.visible_snr_linear;
  json["fused_snr_linear"] = value.fused_snr_linear;
  json["fused_snr_db"] = value.fused_snr_db;
  json["detected"] = value.detected;
  return json;
}

Json BuildJson(const output::EosOutputFrame& value) {
  Json json;
  json["cycle_index"] = value.cycle_index;
  json["scan_azimuth_deg"] = value.scan_azimuth_deg;
  json["detections"] =
      SerializeArray(value.detections, [](const output::EosDetectionRecord& detection) {
        return BuildJson(detection);
      });
  return json;
}

Json BuildJson(const model::EosValidationIssue& value) {
  Json json;
  json["severity"] = static_cast<int>(value.severity);
  json["code"] = static_cast<int>(value.code);
  json["target_index"] = value.target_index;
  json["message"] = value.message;
  return json;
}

Json BuildJson(const model::EosCycleResult& value) {
  Json json;
  json["output_frame"] = BuildJson(value.output_frame);
  json["validation_issues"] =
      SerializeArray(value.validation_issues, [](const model::EosValidationIssue& issue) {
        return BuildJson(issue);
      });
  json["has_validation_error"] = value.has_validation_error;
  json["executed_this_cycle"] = value.executed_this_cycle;
  json["reused_previous_output"] = value.reused_previous_output;
  json["abort_reason"] = static_cast<int>(value.abort_reason);
  return json;
}

Json BuildJson(const EosSessionConfig& value) {
  Json json;
  json["wavelength_lower_um"] = value.wavelength_lower_um;
  json["wavelength_upper_um"] = value.wavelength_upper_um;
  json["optical_aperture_m"] = value.optical_aperture_m;
  json["focal_length_m"] = value.focal_length_m;
  json["work_mode"] = static_cast<int>(value.work_mode);
  json["horizontal_fov_deg"] = value.horizontal_fov_deg;
  json["vertical_fov_deg"] = value.vertical_fov_deg;
  json["scan_rate_deg_per_sec"] = value.scan_rate_deg_per_sec;
  json["frame_rate_hz"] = value.frame_rate_hz;
  json["minimum_snr_db"] = value.minimum_snr_db;
  json["detection_sensitivity_w"] = value.detection_sensitivity_w;
  json["scan_start_az_deg"] = value.scan_start_az_deg;
  json["scan_end_az_deg"] = value.scan_end_az_deg;
  json["scan_center_el_deg"] = value.scan_center_el_deg;
  json["boresight_depression_deg"] = value.boresight_depression_deg;
  json["min_detection_depression_deg"] = value.min_detection_depression_deg;
  json["max_detection_depression_deg"] = value.max_detection_depression_deg;
  json["visible_reference_irradiance_w_m2"] = value.visible_reference_irradiance_w_m2;
  return json;
}

Json BuildJson(const EosRuntimeConfigPatch& value) {
  Json json;
  json["has_work_mode"] = value.has_work_mode;
  json["work_mode"] = static_cast<int>(value.work_mode);
  json["has_scan_rate_deg_per_sec"] = value.has_scan_rate_deg_per_sec;
  json["scan_rate_deg_per_sec"] = value.scan_rate_deg_per_sec;
  json["has_frame_rate_hz"] = value.has_frame_rate_hz;
  json["frame_rate_hz"] = value.frame_rate_hz;
  json["has_minimum_snr_db"] = value.has_minimum_snr_db;
  json["minimum_snr_db"] = value.minimum_snr_db;
  json["has_enable_straylight_filter"] = value.has_enable_straylight_filter;
  json["enable_straylight_filter"] = value.enable_straylight_filter;
  json["has_visible_reference_irradiance_w_m2"] = value.has_visible_reference_irradiance_w_m2;
  json["visible_reference_irradiance_w_m2"] = value.visible_reference_irradiance_w_m2;
  return json;
}

template <typename T>
std::string ToJson(const T& value) {
  return BuildJson(value).dump();
}

}  // namespace

EosTraceSession::EosTraceSession(EosSessionConfig config,
                                 EosTraceSessionOptions options)
    : session_(EosSessionFactory::Create(config)),
      sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", ToJson(config));
  }
}

output::EosOutputFrame EosTraceSession::Step(const EosCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const output::EosOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

model::EosCycleResult EosTraceSession::StepWithResult(const EosCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const model::EosCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

void EosTraceSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", ToJson(patch));
  }
}

EosSession& EosTraceSession::session() { return session_; }

const EosSession& EosTraceSession::session() const { return session_; }

void EosTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electro_optical_sensor", phase, payload_json);
}

}  // namespace session
}  // namespace electro_optical_sensor
