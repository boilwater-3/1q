#include "1q/electro_optical_sensor/tools/EosTraceSession.h"

#include <sstream>
#include <string>

#include "common/trace/JsonFormatUtils.h"

namespace electro_optical_sensor {
namespace tools {
namespace {

using oneq::common::trace::internal::BoolToJson;

std::string ToJson(const oneq::common::Vector3f& value) {
  std::ostringstream stream;
  stream << "[" << value.x << "," << value.y << "," << value.z << "]";
  return stream.str();
}

std::string ToJson(const oneq::common::EulerAnglesDeg& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"yaw_deg\":" << value.yaw_deg << ",";
  stream << "\"pitch_deg\":" << value.pitch_deg << ",";
  stream << "\"roll_deg\":" << value.roll_deg;
  stream << "}";
  return stream.str();
}

std::string ToJson(const oneq::common::PoseState& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"position_m\":" << ToJson(value.position_m) << ",";
  stream << "\"velocity_mps\":" << ToJson(value.velocity_mps) << ",";
  stream << "\"attitude_deg\":" << ToJson(value.attitude_deg);
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::context::EosTargetState& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"target_id\":" << value.target_id << ",";
  stream << "\"range_m\":" << value.range_m << ",";
  stream << "\"azimuth_deg\":" << value.azimuth_deg << ",";
  stream << "\"elevation_deg\":" << value.elevation_deg << ",";
  stream << "\"apparent_temperature_k\":" << value.apparent_temperature_k << ",";
  stream << "\"emissivity\":" << value.emissivity << ",";
  stream << "\"reflectance\":" << value.reflectance << ",";
  stream << "\"projected_area_m2\":" << value.projected_area_m2;
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::context::EosCycleInput& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"cycle_index\":" << value.cycle_index << ",";
  stream << "\"dt_sec\":" << value.dt_sec << ",";
  stream << "\"platform_pose\":" << ToJson(value.platform_pose) << ",";
  stream << "\"solar_altitude_deg\":" << value.solar_altitude_deg << ",";
  stream << "\"solar_azimuth_deg\":" << value.solar_azimuth_deg << ",";
  stream << "\"solar_irradiance_w_m2\":" << value.solar_irradiance_w_m2 << ",";
  stream << "\"atmospheric_transmittance\":" << value.atmospheric_transmittance << ",";
  stream << "\"cloud_coverage_ratio\":" << value.cloud_coverage_ratio << ",";
  stream << "\"day_night_type\":" << static_cast<int>(value.day_night_type) << ",";
  stream << "\"background_temperature_k\":" << value.background_temperature_k << ",";
  stream << "\"scene_targets\":[";
  for (std::size_t i = 0; i < value.scene_targets.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.scene_targets[i]);
  }
  stream << "]";
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EosDetectionRecord& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"target_id\":" << value.target_id << ",";
  stream << "\"range_m\":" << value.range_m << ",";
  stream << "\"azimuth_deg\":" << value.azimuth_deg << ",";
  stream << "\"elevation_deg\":" << value.elevation_deg << ",";
  stream << "\"infrared_snr_linear\":" << value.infrared_snr_linear << ",";
  stream << "\"visible_snr_linear\":" << value.visible_snr_linear << ",";
  stream << "\"fused_snr_linear\":" << value.fused_snr_linear << ",";
  stream << "\"fused_snr_db\":" << value.fused_snr_db << ",";
  stream << "\"detected\":" << BoolToJson(value.detected);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EosOutputFrame& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"cycle_index\":" << value.cycle_index << ",";
  stream << "\"scan_azimuth_deg\":" << value.scan_azimuth_deg << ",";
  stream << "\"detections\":[";
  for (std::size_t i = 0; i < value.detections.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.detections[i]);
  }
  stream << "]";
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::context::EosValidationIssue& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"severity\":" << static_cast<int>(value.severity) << ",";
  stream << "\"code\":" << static_cast<int>(value.code) << ",";
  stream << "\"target_index\":" << value.target_index << ",";
  stream << "\"message\":" << oneq::common::trace::internal::QuoteString(value.message);
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::EosCycleResult& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"output_frame\":" << ToJson(value.output_frame) << ",";
  stream << "\"validation_issues\":[";
  for (std::size_t i = 0; i < value.validation_issues.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.validation_issues[i]);
  }
  stream << "],";
  stream << "\"has_validation_error\":" << BoolToJson(value.has_validation_error);
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::EosSessionConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"wavelength_lower_um\":" << value.wavelength_lower_um << ",";
  stream << "\"wavelength_upper_um\":" << value.wavelength_upper_um << ",";
  stream << "\"optical_aperture_m\":" << value.optical_aperture_m << ",";
  stream << "\"focal_length_m\":" << value.focal_length_m << ",";
  stream << "\"work_mode\":" << static_cast<int>(value.work_mode) << ",";
  stream << "\"horizontal_fov_deg\":" << value.horizontal_fov_deg << ",";
  stream << "\"vertical_fov_deg\":" << value.vertical_fov_deg << ",";
  stream << "\"scan_rate_deg_per_sec\":" << value.scan_rate_deg_per_sec << ",";
  stream << "\"frame_rate_hz\":" << value.frame_rate_hz << ",";
  stream << "\"minimum_snr_db\":" << value.minimum_snr_db << ",";
  stream << "\"detection_sensitivity_w\":" << value.detection_sensitivity_w << ",";
  stream << "\"scan_start_az_deg\":" << value.scan_start_az_deg << ",";
  stream << "\"scan_end_az_deg\":" << value.scan_end_az_deg << ",";
  stream << "\"scan_center_el_deg\":" << value.scan_center_el_deg << ",";
  stream << "\"boresight_depression_deg\":" << value.boresight_depression_deg << ",";
  stream << "\"min_detection_depression_deg\":" << value.min_detection_depression_deg << ",";
  stream << "\"max_detection_depression_deg\":" << value.max_detection_depression_deg << ",";
  stream << "\"visible_reference_irradiance_w_m2\":" << value.visible_reference_irradiance_w_m2;
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::EosRuntimeConfigPatch& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"has_work_mode\":" << BoolToJson(value.has_work_mode) << ",";
  stream << "\"work_mode\":" << static_cast<int>(value.work_mode) << ",";
  stream << "\"has_scan_rate_deg_per_sec\":" << BoolToJson(value.has_scan_rate_deg_per_sec)
         << ",";
  stream << "\"scan_rate_deg_per_sec\":" << value.scan_rate_deg_per_sec << ",";
  stream << "\"has_frame_rate_hz\":" << BoolToJson(value.has_frame_rate_hz) << ",";
  stream << "\"frame_rate_hz\":" << value.frame_rate_hz << ",";
  stream << "\"has_minimum_snr_db\":" << BoolToJson(value.has_minimum_snr_db) << ",";
  stream << "\"minimum_snr_db\":" << value.minimum_snr_db << ",";
  stream << "\"has_enable_straylight_filter\":"
         << BoolToJson(value.has_enable_straylight_filter) << ",";
  stream << "\"enable_straylight_filter\":" << BoolToJson(value.enable_straylight_filter) << ",";
  stream << "\"has_visible_reference_irradiance_w_m2\":"
         << BoolToJson(value.has_visible_reference_irradiance_w_m2) << ",";
  stream << "\"visible_reference_irradiance_w_m2\":"
         << value.visible_reference_irradiance_w_m2;
  stream << "}";
  return stream.str();
}

}  // namespace

EosTraceSession::EosTraceSession(core::session::EosSessionConfig config,
                                 EosTraceSessionOptions options)
    : session_(config), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", ToJson(config));
  }
}

common::EosOutputFrame EosTraceSession::Step(const core::context::EosCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const common::EosOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

core::session::EosCycleResult EosTraceSession::StepWithResult(const core::context::EosCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const core::session::EosCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

void EosTraceSession::ApplyRuntimeConfig(const core::session::EosRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", ToJson(patch));
  }
}

core::session::EosSession& EosTraceSession::session() { return session_; }

const core::session::EosSession& EosTraceSession::session() const { return session_; }

void EosTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electro_optical_sensor", phase, payload_json);
}

}  // namespace tools
}  // namespace electro_optical_sensor
