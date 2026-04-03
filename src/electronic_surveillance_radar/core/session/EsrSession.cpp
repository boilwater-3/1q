#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"

#include <algorithm>
#include <cmath>

#include "1q/electronic_surveillance_radar/core/controller/EsrController.h"
#include "1q/electronic_surveillance_radar/environment/IMutableEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/pipeline/IMutableInterceptPipeline.h"
#include "common/logging/ProjectLog.h"
#include "electronic_surveillance_radar/core/session/EsrSessionConfigResolver.h"
#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {

namespace {

bool IsFinite(float value) { return std::isfinite(value) != 0; }
bool IsFinite(double value) { return std::isfinite(value) != 0; }
bool IsValidReceiverWindow(double lower_hz, double upper_hz) {
  return IsFinite(lower_hz) && IsFinite(upper_hz) && upper_hz > lower_hz;
}

}  // namespace

struct EsrSession::Impl {
  explicit Impl(const EsrSessionConfig& config)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        owned_pipeline(
            new pipeline::InterceptPipeline(resolved_config.pipeline_config, resolved_config.runtime_config)),
        owned_environment_service(new environment::EsrEnvironmentService(resolved_config.environment_config)),
        owned_controller(new controller::EsrController(*owned_pipeline, *owned_environment_service)),
        pipeline(*owned_pipeline),
        environment_service(*owned_environment_service),
        controller(*owned_controller) {}

  Impl(const EsrSessionConfig& config, pipeline::IMutableInterceptPipeline& pipeline_ref,
       environment::IMutableEsrEnvironmentService& environment_service_ref,
       controller::EsrController& controller_ref)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        pipeline(pipeline_ref),
        environment_service(environment_service_ref),
        controller(controller_ref) {
    pipeline.UpdateConfig(resolved_config.pipeline_config);
    pipeline.UpdateRuntimeConfig(resolved_config.runtime_config);
    environment_service.UpdateModelConfig(resolved_config.environment_config);
  }

  // 装配当前周期会话结果。
  EsrCycleResult BuildCycleResult() const {
    EsrCycleResult result;
    if (controller.HasLatestOutputFrame()) {
      result.output_frame = controller.GetLatestOutputFrame();
    }
    result.validation_issues = controller.GetLastValidationIssues();
    result.has_validation_error = context::HasEsrValidationError(result.validation_issues);
    return result;
  }

  internal::ResolvedEsrSessionConfig resolved_config{};
  std::unique_ptr<pipeline::IMutableInterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IMutableEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<controller::EsrController> owned_controller;
  pipeline::IMutableInterceptPipeline& pipeline;
  environment::IMutableEsrEnvironmentService& environment_service;
  controller::EsrController& controller;
};

EsrSession::EsrSession(EsrSessionConfig config) : impl_(new Impl(config)) {}
EsrSession::EsrSession(EsrSessionConfig config, pipeline::IMutableInterceptPipeline& pipeline,
                       environment::IMutableEsrEnvironmentService& environment_service,
                       controller::EsrController& controller)
    : impl_(new Impl(config, pipeline, environment_service, controller)) {}

EsrSession::~EsrSession() = default;

common::EsrOutputFrame EsrSession::Step(const context::EsrCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EsrCycleResult EsrSession::StepWithResult(const context::EsrCycleInput& input) {
  impl_->controller.RunOnce(input);
  return impl_->BuildCycleResult();
}

void EsrSession::ApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch) {
  bool runtime_config_changed = false;
  const bool has_fixed_receiver_window_patch = patch.has_fixed_receiver_window_hz;
  const bool fixed_receiver_window_bounds_valid =
      IsValidReceiverWindow(patch.receiver_lower_hz, patch.receiver_upper_hz);

  if (patch.has_sensor_enabled) {
    impl_->resolved_config.runtime_config.sensor_enabled = patch.sensor_enabled;
    runtime_config_changed = true;
  }
  if (patch.has_scan_rate_hz && IsFinite(patch.scan_rate_hz) && patch.scan_rate_hz > 0.0f) {
    impl_->resolved_config.runtime_config.scan_rate_hz = patch.scan_rate_hz;
    runtime_config_changed = true;
  }
  if (patch.has_integrated_receive_loss_db && IsFinite(patch.integrated_receive_loss_db)) {
    impl_->resolved_config.runtime_config.integrated_receive_loss_db =
        std::max(0.0f, patch.integrated_receive_loss_db);
    runtime_config_changed = true;
  }
  if (has_fixed_receiver_window_patch && fixed_receiver_window_bounds_valid) {
    impl_->resolved_config.runtime_config.receiver_lower_hz = patch.receiver_lower_hz;
    impl_->resolved_config.runtime_config.receiver_upper_hz = patch.receiver_upper_hz;
    runtime_config_changed = true;
  } else if (has_fixed_receiver_window_patch) {
    PROJECT_LOG_ERROR(
        "[EsrSession] ignored invalid fixed receiver window patch: lower_hz={} upper_hz={}",
        patch.receiver_lower_hz, patch.receiver_upper_hz);
  }

  const bool current_fixed_receiver_window_bounds_valid =
      IsValidReceiverWindow(impl_->resolved_config.runtime_config.receiver_lower_hz,
                            impl_->resolved_config.runtime_config.receiver_upper_hz);
  if (patch.has_use_fixed_receiver_window) {
    if (patch.use_fixed_receiver_window) {
      if (fixed_receiver_window_bounds_valid || current_fixed_receiver_window_bounds_valid) {
        impl_->resolved_config.runtime_config.use_fixed_receiver_window = true;
        runtime_config_changed = true;
      } else {
        PROJECT_LOG_ERROR(
            "[EsrSession] ignored fixed receiver window enable request due to invalid bounds: "
            "lower_hz={} upper_hz={}",
            impl_->resolved_config.runtime_config.receiver_lower_hz,
            impl_->resolved_config.runtime_config.receiver_upper_hz);
      }
    } else {
      impl_->resolved_config.runtime_config.use_fixed_receiver_window = false;
      runtime_config_changed = true;
    }
  } else if (fixed_receiver_window_bounds_valid) {
    impl_->resolved_config.runtime_config.use_fixed_receiver_window = true;
    runtime_config_changed = true;
  }
  if (runtime_config_changed) {
    impl_->pipeline.UpdateRuntimeConfig(impl_->resolved_config.runtime_config);
  }

  bool pipeline_config_changed = false;
  if (patch.has_enable_statistical_detection) {
    impl_->resolved_config.pipeline_config.statistical_detection.enable_statistical_detection =
        patch.enable_statistical_detection;
    pipeline_config_changed = true;
  }
  if (patch.has_enable_spectral_analysis) {
    impl_->resolved_config.pipeline_config.spectral_analysis.enable =
        patch.enable_spectral_analysis;
    pipeline_config_changed = true;
  }
  if (patch.has_detection_min_snr_db && IsFinite(patch.detection_min_snr_db)) {
    impl_->resolved_config.pipeline_config.detection.min_detect_snr_db = patch.detection_min_snr_db;
    pipeline_config_changed = true;
  }
  if (pipeline_config_changed) {
    impl_->pipeline.UpdateConfig(impl_->resolved_config.pipeline_config);
  }

  if (patch.has_jamming_detection_threshold_w && IsFinite(patch.jamming_detection_threshold_w)) {
    impl_->resolved_config.environment_config.jamming_detection_threshold_w =
        std::max(0.0f, patch.jamming_detection_threshold_w);
    impl_->environment_service.UpdateModelConfig(impl_->resolved_config.environment_config);
  }

  if (patch.has_observation_jam_mark_threshold_w &&
      IsFinite(patch.observation_jam_mark_threshold_w)) {
    impl_->resolved_config.pipeline_config.suppression_model.suppression_mark_threshold_w =
        std::max(0.0f, patch.observation_jam_mark_threshold_w);
    impl_->pipeline.UpdateConfig(impl_->resolved_config.pipeline_config);
  }
}

}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar
