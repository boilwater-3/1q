#include "1q/electro_optical_sensor/core/session/EosSession.h"

#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "electro_optical_sensor/core/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace core {
namespace session {

namespace {

pipeline::EosPipelineWorkMode ToPipelineWorkMode(EosWorkMode mode) {
  if (mode == EosWorkMode::kInfraredOnly) {
    return pipeline::EosPipelineWorkMode::kInfraredOnly;
  }
  if (mode == EosWorkMode::kVisibleOnly) {
    return pipeline::EosPipelineWorkMode::kVisibleOnly;
  }
  return pipeline::EosPipelineWorkMode::kFused;
}

pipeline::EosPipelineEnvironmentModelType ToPipelineEnvironmentModelType(
    EosEnvironmentModelType model_type) {
  if (model_type == EosEnvironmentModelType::kAdvanced) {
    return pipeline::EosPipelineEnvironmentModelType::kAdvanced;
  }
  return pipeline::EosPipelineEnvironmentModelType::kSimplified;
}

pipeline::EosPipelineConfig BuildPipelineConfig(const EosSessionConfig& config) {
  pipeline::EosPipelineConfig pipeline_config;
  pipeline_config.wavelength_lower_um = config.wavelength_lower_um;
  pipeline_config.wavelength_upper_um = config.wavelength_upper_um;
  pipeline_config.optical_aperture_m = config.optical_aperture_m;
  pipeline_config.focal_length_m = config.focal_length_m;
  pipeline_config.work_mode = ToPipelineWorkMode(config.work_mode);
  pipeline_config.horizontal_fov_deg = config.horizontal_fov_deg;
  pipeline_config.vertical_fov_deg = config.vertical_fov_deg;
  pipeline_config.scan_rate_deg_per_sec = config.scan_rate_deg_per_sec;
  pipeline_config.minimum_snr_db = config.minimum_snr_db;
  pipeline_config.detection_sensitivity_w = config.detection_sensitivity_w;
  pipeline_config.scan_start_az_deg = config.scan_start_az_deg;
  pipeline_config.scan_end_az_deg = config.scan_end_az_deg;
  pipeline_config.scan_center_el_deg = config.scan_center_el_deg;
  pipeline_config.boresight_depression_deg = config.boresight_depression_deg;
  pipeline_config.min_detection_depression_deg = config.min_detection_depression_deg;
  pipeline_config.max_detection_depression_deg = config.max_detection_depression_deg;
  pipeline_config.visible_reference_irradiance_w_m2 = config.visible_reference_irradiance_w_m2;
  pipeline_config.enable_straylight_filter = config.enable_straylight_filter;
  pipeline_config.hood_inner_half_angle_deg = config.hood_inner_half_angle_deg;
  pipeline_config.hood_outer_half_angle_deg = config.hood_outer_half_angle_deg;
  pipeline_config.hood_min_suppression_ratio = config.hood_min_suppression_ratio;
  pipeline_config.hood_max_suppression_ratio = config.hood_max_suppression_ratio;
  pipeline_config.radiative_transfer_model = config.radiative_transfer_model;
  pipeline_config.aerosol_density_factor = config.aerosol_density_factor;
  pipeline_config.turbulence_factor = config.turbulence_factor;
  pipeline_config.environment_model_type =
      ToPipelineEnvironmentModelType(config.environment_model_type);
  return pipeline_config;
}

}  // namespace

struct EosSession::Impl {
  explicit Impl(const EosSessionConfig& config) : pipeline(BuildPipelineConfig(config)) {}

  pipeline::EosPipeline pipeline;
};

EosSession::EosSession(EosSessionConfig config) : impl_(new Impl(config)) {}

EosSession::~EosSession() = default;

common::EosOutputFrame EosSession::Step(const context::EosCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EosCycleResult EosSession::StepWithResult(const context::EosCycleInput& input) {
  EosCycleResult result;
  result.validation_issues = context::ValidateEosCycleInput(input);
  result.has_validation_error = context::HasEosValidationError(result.validation_issues);
  if (result.has_validation_error) {
    result.output_frame.cycle_index = input.cycle_index;
    return result;
  }

  result.output_frame = impl_->pipeline.Execute(input);
  return result;
}

}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor
