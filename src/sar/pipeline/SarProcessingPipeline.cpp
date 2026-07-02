#include "sar/pipeline/SarProcessingPipeline.h"

#include <algorithm>
#include <cmath>

#include "sar/session/SarDiagnosticUtils.h"
#include "sar/session/SarFocusedImageAssembler.h"
#include "sar/session/SarImagingExecutor.h"
#include "sar/session/SarRawHistoryBuilder.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace pipeline {

namespace {

constexpr std::uint32_t kPipelineRuntimeStateSchemaVersion = 1U;

bool HasDegenerateImagePeak(const session::SarCycleResult& result,
                            const session::SarCycleInput& input) {
  if (result.focused_image.is_placeholder) {
    return false;
  }
  if (input.point_targets.empty()) {
    return false;
  }
  if (result.output_frame.has_l1_image || result.output_frame.has_l3_bp_image) {
    for (std::size_t i = 0U; i < result.focused_image.real_values.size(); ++i) {
      const double power =
          result.focused_image.real_values[i] * result.focused_image.real_values[i] +
          result.focused_image.imaginary_values[i] * result.focused_image.imaginary_values[i];
      if (power > 0.0) {
        return false;
      }
    }
    return true;
  }
  return false;
}

}  // namespace

struct SarProcessingPipeline::Impl {
  explicit Impl(const config::SarSessionConfig& initial_config)
      : raw_pulse_buffer(std::max<std::size_t>(initial_config.mission.azimuth_pulse_count, 1U)) {}

  runtime::PulseRingBuffer raw_pulse_buffer;
  std::deque<geometry::PlatformPulseState> ideal_trajectory_buffer;
  std::deque<geometry::PlatformPulseState> actual_trajectory_buffer;
  std::uint64_t next_pulse_id{0U};
  double pulse_fraction_carry{0.0};
};

SarProcessingPipeline::SarProcessingPipeline(const config::SarSessionConfig& initial_config)
    : impl_(new Impl(initial_config)) {}

SarProcessingPipeline::~SarProcessingPipeline() = default;

bool SarProcessingPipeline::RunCycle(const config::SarSessionConfig& config,
                                     const session::SarCycleInput& input,
                                     session::SarCycleResult* result) {
  if (result == nullptr) {
    return false;
  }

  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!session::BuildWaveformAndFilter(config, &waveform, &matched_filter)) {
    session::RecordAbort(result, "waveform_generation_failed",
                         "SAR failed to generate LFM waveform.");
    return false;
  }

  signal::ComplexMatrix raw_history;
  if (config.policy.enable_raw_echo_generation) {
    if (session::HasExternalRawIq(input)) {
      if (!session::BuildExternalRawIqHistory(config, input, &raw_history,
                                              &impl_->ideal_trajectory_buffer,
                                              &impl_->actual_trajectory_buffer, result)) {
        return false;
      }
    } else {
      if (!session::BuildRawPulseHistory(config, input, waveform.samples, &impl_->raw_pulse_buffer,
                                         &impl_->next_pulse_id, &impl_->pulse_fraction_carry,
                                         &raw_history, &impl_->ideal_trajectory_buffer,
                                         &impl_->actual_trajectory_buffer, result)) {
        return false;
      }
    }
    const double estimated_snr_db = session::EstimateRawHistorySnrDb(raw_history);
    session::MarkRawEchoStage(&result->output_frame, estimated_snr_db);
    if (std::isfinite(result->output_frame.estimated_snr_db) &&
        result->output_frame.estimated_snr_db < config.policy.minimum_snr_db) {
      session::RecordAbort(result, "snr_below_minimum",
                           "SAR estimated SNR is below the configured minimum valid SNR.");
      return false;
    }
  }
  if (config.policy.enable_range_compression) {
    session::MarkRangeCompressionStage(&result->output_frame);
  }
  if (config.policy.enable_l1_rda_imaging) {
    if (!session::ExecuteL1RdaImaging(config, raw_history, matched_filter,
                                      impl_->ideal_trajectory_buffer,
                                      impl_->actual_trajectory_buffer, result)) {
      return false;
    }
  }
  if (config.policy.enable_l3_bp_imaging) {
    if (!session::ExecuteL3BpImaging(config, raw_history, matched_filter,
                                     impl_->actual_trajectory_buffer, result)) {
      return false;
    }
  }

  if (HasDegenerateImagePeak(*result, input)) {
    session::RecordAbort(
        result, "degenerate_image_peak",
        "SAR focused image has zero peak power; the echo/focusing pipeline produced no "
        "signal. Check sar.raw_echo_clipping and sar.slant_range_mismatch diagnostics.");
    return false;
  }

  result->executed_this_cycle = true;
  return true;
}

SarProcessingPipelineRuntimeState SarProcessingPipeline::CaptureRuntimeState() const {
  SarProcessingPipelineRuntimeState state;
  state.owner_identity = this;
  state.schema_version = kPipelineRuntimeStateSchemaVersion;
  state.raw_pulse_buffer_state = impl_->raw_pulse_buffer.CaptureRuntimeState();
  state.ideal_trajectory_buffer = impl_->ideal_trajectory_buffer;
  state.actual_trajectory_buffer = impl_->actual_trajectory_buffer;
  state.next_pulse_id = impl_->next_pulse_id;
  state.pulse_fraction_carry = impl_->pulse_fraction_carry;
  return state;
}

bool SarProcessingPipeline::RestoreRuntimeState(const SarProcessingPipelineRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != kPipelineRuntimeStateSchemaVersion ||
      !impl_->raw_pulse_buffer.RestoreRuntimeState(state.raw_pulse_buffer_state)) {
    return false;
  }
  impl_->ideal_trajectory_buffer = state.ideal_trajectory_buffer;
  impl_->actual_trajectory_buffer = state.actual_trajectory_buffer;
  impl_->next_pulse_id = state.next_pulse_id;
  impl_->pulse_fraction_carry = state.pulse_fraction_carry;
  return true;
}

}  // namespace pipeline
}  // namespace sar
