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
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kRadiansToDegrees = 180.0 / 3.141592653589793238462643383279502884;

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

void ExportRawPhaseHistory(const signal::ComplexMatrix& raw_history,
                           session::SarRawPhaseHistorySource source,
                           session::SarRawPhaseHistory* output) {
  if (output == nullptr) {
    return;
  }
  output->source = source;
  output->pulse_count = static_cast<std::uint32_t>(raw_history.rows);
  output->samples_per_pulse = static_cast<std::uint32_t>(raw_history.cols);
  output->i_values.reserve(raw_history.values.size());
  output->q_values.reserve(raw_history.values.size());
  for (const signal::ComplexSample& sample : raw_history.values) {
    output->i_values.push_back(sample.real());
    output->q_values.push_back(sample.imag());
  }
}

double ComputeSquintAngleDeg(const geometry::PlatformPulseState& pulse) {
  const double speed = std::sqrt(pulse.velocity_x_mps * pulse.velocity_x_mps +
                                 pulse.velocity_y_mps * pulse.velocity_y_mps +
                                 pulse.velocity_z_mps * pulse.velocity_z_mps);
  const double los_x = -pulse.position_m.x_m;
  const double los_y = -pulse.position_m.y_m;
  const double los_z = -pulse.position_m.z_m;
  const double range = std::sqrt(los_x * los_x + los_y * los_y + los_z * los_z);
  if (speed <= 0.0 || range <= 0.0) {
    return 0.0;
  }
  const double along_track_cosine =
      std::abs((pulse.velocity_x_mps * los_x + pulse.velocity_y_mps * los_y +
                pulse.velocity_z_mps * los_z) /
               (speed * range));
  return std::asin(std::min(1.0, along_track_cosine)) * kRadiansToDegrees;
}

geometry::PlatformPulseState BuildCurrentPlatformPulse(
    const config::SarMissionConfig& mission, const session::SarPlatformState& platform) {
  const double degrees_to_radians = 1.0 / kRadiansToDegrees;
  const double reference_latitude_rad = mission.scene_center_latitude_deg * degrees_to_radians;
  geometry::PlatformPulseState pulse;
  pulse.position_m.x_m =
      (platform.longitude_deg - mission.scene_center_longitude_deg) * degrees_to_radians *
      std::cos(reference_latitude_rad) * kEarthRadiusM;
  pulse.position_m.y_m =
      (platform.latitude_deg - mission.scene_center_latitude_deg) * degrees_to_radians *
      kEarthRadiusM;
  pulse.position_m.z_m = platform.altitude_m - mission.scene_center_altitude_m;
  pulse.velocity_x_mps = platform.velocity_east_mps;
  pulse.velocity_y_mps = platform.velocity_north_mps;
  pulse.velocity_z_mps = -platform.velocity_down_mps;
  return pulse;
}

double ResolveMaximumSquintAngleDeg(
    const config::SarMissionConfig& mission, const session::SarCycleInput& input,
    const signal::ComplexMatrix& raw_history,
    const std::deque<geometry::PlatformPulseState>& actual_trajectory) {
  double maximum_angle_deg = 0.0;
  if (input.raw_iq.pulse_states.size() == raw_history.rows && raw_history.rows != 0U) {
    for (const session::SarRawIqFrame::PulseState& state : input.raw_iq.pulse_states) {
      geometry::PlatformPulseState pulse;
      pulse.position_m.x_m = state.position_x_m;
      pulse.position_m.y_m = state.position_y_m;
      pulse.position_m.z_m = state.position_z_m;
      pulse.velocity_x_mps = state.velocity_x_mps;
      pulse.velocity_y_mps = state.velocity_y_mps;
      pulse.velocity_z_mps = state.velocity_z_mps;
      maximum_angle_deg = std::max(maximum_angle_deg, ComputeSquintAngleDeg(pulse));
    }
    return maximum_angle_deg;
  }
  if (!actual_trajectory.empty() && actual_trajectory.size() == raw_history.rows) {
    for (const geometry::PlatformPulseState& pulse : actual_trajectory) {
      maximum_angle_deg = std::max(maximum_angle_deg, ComputeSquintAngleDeg(pulse));
    }
    return maximum_angle_deg;
  }
  return ComputeSquintAngleDeg(BuildCurrentPlatformPulse(mission, input.platform));
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
  session::SarRawPhaseHistorySource raw_history_source =
      session::SarRawPhaseHistorySource::kNone;
  if (config.policy.enable_raw_echo_generation) {
    if (session::HasExternalRawIq(input)) {
      raw_history_source = session::SarRawPhaseHistorySource::kExternalRawIq;
      if (!session::BuildExternalRawIqHistory(config, input, &raw_history,
                                              &impl_->ideal_trajectory_buffer,
                                              &impl_->actual_trajectory_buffer, result)) {
        return false;
      }
      result->diagnostics.push_back(session::MakeInfoDiagnostic(
          "sar.external_raw_iq_snr_unavailable",
          "External raw IQ is already receiver-domain data; hardware link budget and minimum "
          "SNR gating are not reapplied without signal/noise metadata."));
    } else {
      raw_history_source = session::SarRawPhaseHistorySource::kInternallyGenerated;
      double estimated_snr_db = -std::numeric_limits<double>::infinity();
      if (!session::BuildRawPulseHistory(config, input, waveform.samples, &impl_->raw_pulse_buffer,
                                         &impl_->next_pulse_id, &impl_->pulse_fraction_carry,
                                         &raw_history, &impl_->ideal_trajectory_buffer,
                                         &impl_->actual_trajectory_buffer, &estimated_snr_db,
                                         result)) {
        return false;
      }
      session::MarkRawEchoStage(&result->output_frame, estimated_snr_db);
      if (std::isfinite(estimated_snr_db) && estimated_snr_db < config.policy.minimum_snr_db) {
        session::RecordAbort(result, "snr_below_minimum",
                             "SAR estimated SNR is below the configured minimum valid SNR.");
        return false;
      }
    }
    if (raw_history_source == session::SarRawPhaseHistorySource::kExternalRawIq) {
      session::MarkRawEchoStage(&result->output_frame,
                                -std::numeric_limits<double>::infinity());
    }
  }
  if (config.policy.enable_l1_rda_imaging || config.policy.enable_l3_bp_imaging) {
    const double maximum_squint_angle_deg = ResolveMaximumSquintAngleDeg(
        config.mission, input, raw_history, impl_->actual_trajectory_buffer);
    if (maximum_squint_angle_deg > config.policy.max_allowed_squint_angle_deg) {
      session::RecordAbort(result, "squint_angle_exceeds_limit",
                           "SAR aperture squint angle exceeds the configured imaging limit.");
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

  if (config.policy.retain_raw_phase_history) {
    ExportRawPhaseHistory(raw_history, raw_history_source, &result->raw_phase_history);
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
