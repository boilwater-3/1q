#include "1q/electro_optical_sensor/session/EosSession.h"

#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "electro_optical_sensor/core/session/EosRuntimeConfigResolver.h"
#include "electro_optical_sensor/core/session/EosSessionCompositionRoot.h"

namespace electro_optical_sensor {
namespace session {

namespace {

extension::EosPipelineWorkMode ToPipelineWorkMode(EosWorkMode mode) {
  if (mode == EosWorkMode::kInfraredOnly) {
    return extension::EosPipelineWorkMode::kInfraredOnly;
  }
  if (mode == EosWorkMode::kVisibleOnly) {
    return extension::EosPipelineWorkMode::kVisibleOnly;
  }
  return extension::EosPipelineWorkMode::kFused;
}

extension::EosPipelineEnvironmentModelType ToPipelineEnvironmentModelType(
    environment::EosEnvironmentModelType model_type) {
  if (model_type == environment::EosEnvironmentModelType::kAdvanced) {
    return extension::EosPipelineEnvironmentModelType::kAdvanced;
  }
  return extension::EosPipelineEnvironmentModelType::kSimplified;
}

extension::EosPipelineConfig BuildPipelineConfig(const EosSessionConfig& config) {
  extension::EosPipelineConfig pipeline_config;
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
  Impl(internal::EosSessionComposition composition, const EosSessionConfig& config)
      : runtime_config(config),
        owned_pipeline(std::move(composition.owned_pipeline)),
        owned_controller(std::move(composition.owned_controller)),
        pipeline(*composition.pipeline),
        controller(*composition.controller) {
    pipeline.UpdateConfig(BuildPipelineConfig(runtime_config), true);
  }

  EosSessionConfig runtime_config{};
  std::unique_ptr<::electro_optical_sensor::extension::IEosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;
  ::electro_optical_sensor::extension::IEosPipeline& pipeline;
  extension::EosController& controller;
};

EosSession::EosSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EosSession::~EosSession() noexcept = default;
EosSession::EosSession(EosSession&&) noexcept = default;
EosSession& EosSession::operator=(EosSession&&) noexcept = default;

EosSession EosSessionFactory::Create(const EosSessionConfig& config) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeDefault(), config)));
}

EosSession EosSessionFactory::CreateWithPipeline(
    const EosSessionConfig& config, ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeWithPipeline(pipeline), config)));
}

EosSession EosSessionFactory::CreateWithEnvironmentService(
    const EosSessionConfig& config,
    extension::IEosEnvironmentService& environment_service) {
  return EosSession(std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
      internal::EosSessionCompositionRoot::ComposeWithEnvironmentService(environment_service),
      config)));
}

EosSession EosSessionFactory::CreateWithController(
    const EosSessionConfig& config, extension::EosController& controller) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeWithController(controller), config)));
}

common::EosOutputFrame EosSession::Step(const EosCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EosCycleResult EosSession::StepWithResult(const EosCycleInput& input) {
  impl_->controller.RunOnce(input);
  EosCycleResult result;
  result.validation_issues = impl_->controller.GetLastValidationIssues();
  result.has_validation_error = impl_->controller.HasValidationError();
  result.executed_this_cycle = impl_->controller.ExecutedLatestCycle();
  result.reused_previous_output = impl_->controller.ReusedPreviousOutputLatestCycle();
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
}  // namespace electro_optical_sensor
