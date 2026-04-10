/**
 * @file EosSessionCompositionRoot.cpp
 * @brief 实现 EOS 会话组合根，统一装配 pipeline 与 controller 依赖。
 */

#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"

#include <cstdlib>
#include <memory>
#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

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

extension::EosPipelineConfig BuildPipelineConfigFromSessionConfig(
    const EosSessionConfig& config) {
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

EosSessionComposition BuildInitialCompositionRuntime(const EosSessionConfig& config,
                                                     EosSessionComposition composition) {
  composition.runtime_config = config;
  composition.pipeline_config = BuildPipelineConfigFromSessionConfig(config);
  composition.initial_reset_scan_phase = true;
  return composition;
}

/**
 * @brief 校验组合结果中的关键依赖非空。
 * @param[in] ptr 待校验指针。
 * @param[in] dependency_name 依赖名称，用于日志与故障定位。
 * @return 非空依赖引用。
 * @warning 若装配输出为空指针，将终止进程避免未定义行为。
 */
template <typename T>
T& RequireComposedDependency(T* ptr, const char* dependency_name) {
  if (ptr != nullptr) {
    return *ptr;
  }
  PROJECT_LOG_ERROR("[EosSessionCompositionRoot] Dependency '{}' is null.", dependency_name);
  std::abort();
}

/**
 * @brief 统一收尾并校验组合结果。
 * @param[in] composition 待校验组合结果。
 * @return 校验后的组合结果。
 */
EosSessionComposition FinalizeComposition(EosSessionComposition composition) {
  static_cast<void>(RequireComposedDependency(composition.pipeline, "pipeline"));
  static_cast<void>(RequireComposedDependency(composition.controller, "controller"));
  return composition;
}

std::shared_ptr<extension::IEosEnvironmentService> MakeNonOwningEnvironmentServiceHandle(
    extension::IEosEnvironmentService& environment_service) {
  return std::shared_ptr<extension::IEosEnvironmentService>(
      &environment_service, [](extension::IEosEnvironmentService*) {});
}

EosSessionComposition MakeCompositionWithOwnedPipeline(
    std::unique_ptr<extension::IEosPipeline> owned_pipeline,
    const EosSessionConfig& config) {
  EosSessionComposition composition;
  composition.owned_pipeline = std::move(owned_pipeline);
  composition.owned_controller.reset(new extension::EosController(*composition.owned_pipeline));
  composition.pipeline = composition.owned_pipeline.get();
  composition.controller = composition.owned_controller.get();
  return BuildInitialCompositionRuntime(config, std::move(composition));
}

EosSessionComposition MakeCompositionWithExternalPipeline(
    extension::IEosPipeline& pipeline,
    const EosSessionConfig& config) {
  EosSessionComposition composition;
  composition.owned_controller.reset(new extension::EosController(pipeline));
  composition.pipeline = &pipeline;
  composition.controller = composition.owned_controller.get();
  composition = BuildInitialCompositionRuntime(config, std::move(composition));
  composition.pipeline->UpdateConfig(composition.pipeline_config,
                                     composition.initial_reset_scan_phase);
  return composition;
}

EosSessionComposition ComposeWithOwnedPipeline(
    std::unique_ptr<extension::IEosPipeline> owned_pipeline,
    const EosSessionConfig& config) {
  return FinalizeComposition(MakeCompositionWithOwnedPipeline(std::move(owned_pipeline), config));
}

EosSessionComposition ComposeWithExternalPipeline(extension::IEosPipeline& pipeline,
                                                  const EosSessionConfig& config) {
  return FinalizeComposition(MakeCompositionWithExternalPipeline(pipeline, config));
}

}  // namespace

EosSessionComposition EosSessionCompositionRoot::ComposeDefault(const EosSessionConfig& config) {
  return ComposeWithOwnedPipeline(std::unique_ptr<extension::IEosPipeline>(
      new core::pipeline::EosPipeline(BuildPipelineConfigFromSessionConfig(config))),
      config);
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithPipeline(
    const EosSessionConfig& config,
    ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  return ComposeWithExternalPipeline(pipeline, config);
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithEnvironmentService(
    const EosSessionConfig& config,
    extension::IEosEnvironmentService& environment_service) {
  return ComposeWithOwnedPipeline(std::unique_ptr<extension::IEosPipeline>(
      new core::pipeline::EosPipeline(BuildPipelineConfigFromSessionConfig(config),
                                       MakeNonOwningEnvironmentServiceHandle(
                                           environment_service))),
      config);
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithController(
    const EosSessionConfig& config,
    extension::EosController& controller) {
  EosSessionComposition composition;
  composition = BuildInitialCompositionRuntime(config, std::move(composition));
  composition.pipeline = &controller.GetPipeline();
  composition.controller = &controller;
  composition.pipeline->UpdateConfig(composition.pipeline_config,
                                     composition.initial_reset_scan_phase);
  return FinalizeComposition(std::move(composition));
}

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
