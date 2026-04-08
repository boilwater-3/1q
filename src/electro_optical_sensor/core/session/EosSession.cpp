#include "1q/electro_optical_sensor/core/session/EosSession.h"

#include <utility>

#include "1q/electro_optical_sensor/core/controller/EosController.h"
#include "1q/electro_optical_sensor/pipeline/IEosPipeline.h"
#include "electro_optical_sensor/core/pipeline/EosPipeline.h"
#include "electro_optical_sensor/core/session/EosRuntimeConfigResolver.h"

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
    environment::EosEnvironmentModelType model_type) {
  if (model_type == environment::EosEnvironmentModelType::kAdvanced) {
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

struct EosSession::Impl {
  explicit Impl(const EosSessionConfig& config)
      : runtime_config(config),
        owned_pipeline(new core::pipeline::EosPipeline(BuildPipelineConfig(config))),
        owned_controller(new core::controller::EosController(*owned_pipeline)),
        pipeline(*owned_pipeline),
        controller(*owned_controller) {}

  Impl(const EosSessionConfig& config, ::electro_optical_sensor::pipeline::IEosPipeline& pipeline_ref)
      : runtime_config(config),
        owned_controller(new core::controller::EosController(pipeline_ref)),
        pipeline(pipeline_ref),
        controller(*owned_controller) {
    pipeline.UpdateConfig(BuildPipelineConfig(runtime_config), true);
  }

  Impl(const EosSessionConfig& config, core::controller::EosController& controller_ref)
      : runtime_config(config), pipeline(controller_ref.GetPipeline()), controller(controller_ref) {
    pipeline.UpdateConfig(BuildPipelineConfig(runtime_config), true);
  }

  Impl(const EosSessionConfig& config, ::electro_optical_sensor::pipeline::IEosPipeline& pipeline_ref,
       core::controller::EosController& controller_ref)
      : runtime_config(config), pipeline(pipeline_ref), controller(controller_ref) {
    pipeline.UpdateConfig(BuildPipelineConfig(runtime_config), true);
  }

  EosSessionConfig runtime_config{};
  std::unique_ptr<::electro_optical_sensor::pipeline::IEosPipeline> owned_pipeline;
  std::unique_ptr<core::controller::EosController> owned_controller;
  ::electro_optical_sensor::pipeline::IEosPipeline& pipeline;
  core::controller::EosController& controller;
};

EosSession::EosSession(EosSessionConfig config) : impl_(new Impl(config)) {}
EosSession::EosSession(EosSessionConfig config, ::electro_optical_sensor::pipeline::IEosPipeline& pipeline)
    : impl_(new Impl(config, pipeline)) {}
EosSession::EosSession(EosSessionConfig config, core::controller::EosController& controller)
    : impl_(new Impl(config, controller)) {}
EosSession::EosSession(EosSessionConfig config, ::electro_optical_sensor::pipeline::IEosPipeline& pipeline,
                       core::controller::EosController& controller)
    : impl_(new Impl(config, pipeline, controller)) {}

EosSession::~EosSession() noexcept = default;

common::EosOutputFrame EosSession::Step(const context::EosCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EosCycleResult EosSession::StepWithResult(const context::EosCycleInput& input) {
  impl_->controller.RunOnce(input);
  EosCycleResult result;
  result.validation_issues = impl_->controller.GetLastValidationIssues();
  result.has_validation_error = impl_->controller.HasValidationError();
  if (impl_->controller.HasLatestOutputFrame()) {
    result.output_frame = impl_->controller.GetLatestOutputFrame();
  } else {
    result.output_frame.cycle_index = input.cycle_index;
  }
  return result;
}

void EosSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  const internal::EosRuntimeConfigResolveResult resolved =
      internal::ResolveEosRuntimeConfigPatch(impl_->runtime_config, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return;
  }
  impl_->runtime_config = resolved.next_config;
  impl_->pipeline.UpdateConfig(BuildPipelineConfig(impl_->runtime_config), resolved.reset_scan_phase);
}

}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor
