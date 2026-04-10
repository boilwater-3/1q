#include "electro_optical_sensor/runtime/EosCycleOrchestrator.h"

#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/session/EosRuntimeConfigResolver.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

::electro_optical_sensor::extension::EosPipelineWorkMode ToPipelineWorkMode(EosWorkMode mode) {
  if (mode == EosWorkMode::kInfraredOnly) {
    return ::electro_optical_sensor::extension::EosPipelineWorkMode::kInfraredOnly;
  }
  if (mode == EosWorkMode::kVisibleOnly) {
    return ::electro_optical_sensor::extension::EosPipelineWorkMode::kVisibleOnly;
  }
  return ::electro_optical_sensor::extension::EosPipelineWorkMode::kFused;
}

::electro_optical_sensor::extension::EosPipelineEnvironmentModelType ToPipelineEnvironmentModelType(
    environment::EosEnvironmentModelType model_type) {
  if (model_type == environment::EosEnvironmentModelType::kAdvanced) {
    return ::electro_optical_sensor::extension::EosPipelineEnvironmentModelType::kAdvanced;
  }
  return ::electro_optical_sensor::extension::EosPipelineEnvironmentModelType::kSimplified;
}

::electro_optical_sensor::extension::EosPipelineConfig BuildPipelineConfig(
    const EosSessionConfig& config) {
  ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config;
  pipeline_config.wavelength_lower_um = config.wavelength_lower_um;
  pipeline_config.wavelength_upper_um = config.wavelength_upper_um;
  pipeline_config.optical_aperture_m = config.optical_aperture_m;
  pipeline_config.focal_length_m = config.focal_length_m;
  pipeline_config.work_mode = ToPipelineWorkMode(config.work_mode);
  pipeline_config.horizontal_fov_deg = config.horizontal_fov_deg;
  pipeline_config.vertical_fov_deg = config.vertical_fov_deg;
  pipeline_config.scan_rate_deg_per_sec = config.scan_rate_deg_per_sec;
  pipeline_config.frame_rate_hz = config.frame_rate_hz;
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
  pipeline_config.radiative_transfer_model =
      config.environment_default_config.radiative_transfer_model;
  pipeline_config.aerosol_density_factor = config.environment_default_config.aerosol_density_factor;
  pipeline_config.turbulence_factor = config.environment_default_config.turbulence_factor;
  pipeline_config.environment_model_type =
      ToPipelineEnvironmentModelType(config.environment_default_config.model_type);
  return pipeline_config;
}

}  // namespace

EosCycleOrchestrator::EosCycleOrchestrator(
    const EosSessionConfig& config,
    ::electro_optical_sensor::extension::IEosPipeline& pipeline,
    ::electro_optical_sensor::extension::EosController& controller)
    : runtime_config_(config), pipeline_(pipeline), controller_(controller) {
  pipeline_.UpdateConfig(BuildPipelineConfig(runtime_config_), true);
}

EosCycleResult EosCycleOrchestrator::BuildResult(const EosCycleInput& input) const {
  EosCycleResult result;
  result.validation_issues = controller_.GetLastValidationIssues();
  result.has_validation_error = controller_.HasValidationError();
  result.executed_this_cycle = controller_.ExecutedLatestCycle();
  result.reused_previous_output = controller_.ReusedPreviousOutputLatestCycle();
  if (controller_.HasLatestOutputFrame()) {
    result.output_frame = controller_.GetLatestOutputFrame();
  } else {
    result.output_frame.cycle_index = input.cycle_index;
  }
  return result;
}

EosCycleResult EosCycleOrchestrator::Step(const EosCycleInput& input) {
  controller_.RunOnce(input);
  return BuildResult(input);
}

void EosCycleOrchestrator::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  const ::electro_optical_sensor::session::internal::EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(runtime_config_, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return;
  }
  runtime_config_ = resolved.next_config;
  pipeline_.UpdateConfig(BuildPipelineConfig(runtime_config_), resolved.reset_scan_phase);
}

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
