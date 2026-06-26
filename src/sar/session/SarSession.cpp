#include "1q/sar/session/SarSession.h"

#include <algorithm>
#include <deque>
#include <cmath>
#include <memory>
#include <utility>

#include "sar/geometry/SarGeometry.h"
#include "sar/runtime/PulseRingBuffer.h"
#include "sar/session/SarFocusedImageAssembler.h"
#include "sar/session/SarImagingExecutor.h"
#include "sar/session/SarDiagnosticUtils.h"
#include "sar/session/SarRawHistoryBuilder.h"
#include "sar/session/SarRuntimeConfigValidation.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

namespace {

bool HasRequestedUpdate(const config::SarRuntimeConfigPatch& patch) {
  return patch.has_enable_raw_echo_generation || patch.has_enable_range_compression ||
         patch.has_enable_l1_rda_imaging || patch.has_retain_raw_phase_history ||
         patch.has_retain_focused_image || patch.has_minimum_snr_db;
}

void ApplyPatchToConfig(config::SarSessionConfig* config,
                        const config::SarRuntimeConfigPatch& patch) {
  if (patch.has_enable_raw_echo_generation) {
    config->policy.enable_raw_echo_generation = patch.enable_raw_echo_generation;
  }
  if (patch.has_enable_range_compression) {
    config->policy.enable_range_compression = patch.enable_range_compression;
  }
  if (patch.has_enable_l1_rda_imaging) {
    config->policy.enable_l1_rda_imaging = patch.enable_l1_rda_imaging;
  }
  if (patch.has_retain_raw_phase_history) {
    config->policy.retain_raw_phase_history = patch.retain_raw_phase_history;
  }
  if (patch.has_retain_focused_image) {
    config->policy.retain_focused_image = patch.retain_focused_image;
  }
  if (patch.has_minimum_snr_db) {
    config->policy.minimum_snr_db = patch.minimum_snr_db;
  }
}

void ApplyDiagnosticsPolicy(const config::SarPolicyConfig& policy, SarCycleResult* result) {
  if (policy.enable_diagnostics || result == nullptr) {
    return;
  }
  SarDiagnosticIssueList errors;
  for (const SarDiagnosticIssue& issue : result->diagnostics) {
    if (issue.severity == SarDiagnosticSeverity::kError) {
      errors.push_back(issue);
    }
  }
  result->diagnostics.swap(errors);
}

}  // namespace

struct SarSession::Impl {
  explicit Impl(const config::SarSessionConfig& initial_config)
      : runtime_config(initial_config),
        raw_pulse_buffer(std::max<std::size_t>(initial_config.mission.azimuth_pulse_count, 1U)) {}

  config::SarSessionConfig runtime_config;
  runtime::PulseRingBuffer raw_pulse_buffer;
  std::deque<geometry::PlatformPulseState> ideal_trajectory_buffer;
  std::deque<geometry::PlatformPulseState> actual_trajectory_buffer;
  std::uint64_t next_pulse_id{0U};
  double pulse_fraction_carry{0.0};
  SarOutputFrame previous_output{};
  bool has_previous_output{false};
};

SarSession::SarSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

SarSession::SarSession() : impl_(new Impl(config::SarSessionConfig{})) {}

SarSession::~SarSession() noexcept = default;
SarSession::SarSession(SarSession&&) noexcept = default;
SarSession& SarSession::operator=(SarSession&&) noexcept = default;

SarSession SarSession::Create(const config::SarSessionConfig& config) {
  return SarSession(std::unique_ptr<SarSession::Impl>(new SarSession::Impl(config)));
}

SarOutputFrame SarSession::Step(const SarCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SarCycleResult SarSession::StepWithResult(const SarCycleInput& input) {
  SarCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.output_frame.cycle_index = input.cycle_index;
  const auto finish = [this, &result]() -> SarCycleResult {
    ApplyDiagnosticsPolicy(impl_->runtime_config.policy, &result);
    return result;
  };

  if (input.dt_sec <= 0.0) {
    RecordAbort(&result, "invalid_dt_sec", "SAR cycle dt_sec must be positive.");
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return finish();
  }

  InitializeOutputFrameMetadata(impl_->runtime_config, &result.output_frame);

  const bool has_external_raw_iq = HasExternalRawIq(input);
  if (!ValidateRuntimeConfigForStep(impl_->runtime_config, has_external_raw_iq, &result)) {
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return finish();
  }

  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!BuildWaveformAndFilter(impl_->runtime_config, &waveform, &matched_filter)) {
    RecordAbort(&result, "waveform_generation_failed", "SAR failed to generate LFM waveform.");
    return finish();
  }

  signal::ComplexMatrix raw_history;
  if (impl_->runtime_config.policy.enable_raw_echo_generation) {
    if (has_external_raw_iq) {
      if (!BuildExternalRawIqHistory(impl_->runtime_config, input, &raw_history,
                                     &impl_->ideal_trajectory_buffer,
                                     &impl_->actual_trajectory_buffer, &result)) {
        return finish();
      }
    } else {
      if (!BuildRawPulseHistory(
              impl_->runtime_config, input, waveform.samples, &impl_->raw_pulse_buffer,
              &impl_->next_pulse_id, &impl_->pulse_fraction_carry, &raw_history,
              &impl_->ideal_trajectory_buffer, &impl_->actual_trajectory_buffer, &result)) {
        return finish();
      }
    }
    const double estimated_snr_db = EstimateRawHistorySnrDb(raw_history);
    MarkRawEchoStage(&result.output_frame, estimated_snr_db);
    if (std::isfinite(result.output_frame.estimated_snr_db) &&
        result.output_frame.estimated_snr_db < impl_->runtime_config.policy.minimum_snr_db) {
      RecordAbort(&result, "snr_below_minimum",
                  "SAR estimated SNR is below the configured minimum valid SNR.");
      return finish();
    }
  }
  if (impl_->runtime_config.policy.enable_range_compression) {
    MarkRangeCompressionStage(&result.output_frame);
  }
  if (impl_->runtime_config.policy.enable_l1_rda_imaging) {
    if (!ExecuteL1RdaImaging(impl_->runtime_config, raw_history, matched_filter,
                             impl_->ideal_trajectory_buffer, impl_->actual_trajectory_buffer,
                             &result)) {
      return finish();
    }
  }
  if (impl_->runtime_config.policy.enable_l3_bp_imaging) {
    if (!ExecuteL3BpImaging(impl_->runtime_config, raw_history, matched_filter,
                            impl_->actual_trajectory_buffer, &result)) {
      return finish();
    }
  }

  result.executed_this_cycle = true;
  impl_->previous_output = result.output_frame;
  impl_->has_previous_output = true;
  return finish();
}

void SarSession::ApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool SarSession::TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  if (!HasRequestedUpdate(patch)) {
    return false;
  }
  ApplyPatchToConfig(&impl_->runtime_config, patch);
  return true;
}

}  // namespace session
}  // namespace sar
