#include "1q/sar/session/SarSession.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "1q/sar/session/SarSessionFactory.h"
#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarRda.h"
#include "sar/runtime/PulseRingBuffer.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr std::uint32_t kMaxSessionRdaRangeSamples = 1024U;
constexpr std::uint32_t kMaxSessionRdaPulses = 1024U;

bool HasRequestedUpdate(const config::SarRuntimeConfigPatch& patch) {
  return patch.has_enable_raw_echo_generation || patch.has_enable_range_compression ||
         patch.has_enable_l1_rda_imaging || patch.has_retain_raw_phase_history ||
         patch.has_min_valid_snr_db;
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
  if (patch.has_min_valid_snr_db) {
    config->policy.min_valid_snr_db = patch.min_valid_snr_db;
  }
}

SarDiagnosticIssue MakeError(const char* code, const char* message) {
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kError;
  issue.code = code;
  issue.message = message;
  return issue;
}

SarDiagnosticIssue MakeInfo(const char* code, const std::string& message) {
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  return issue;
}

geometry::LocalPoint ToLocalPoint(double latitude_deg, double longitude_deg, double altitude_m,
                                  const config::SarMissionConfig& mission) {
  const double deg_to_rad = 3.141592653589793238462643383279502884 / 180.0;
  const double lat0_rad = mission.scene_center_latitude_deg * deg_to_rad;
  const double dlat_rad = (latitude_deg - mission.scene_center_latitude_deg) * deg_to_rad;
  const double dlon_rad = (longitude_deg - mission.scene_center_longitude_deg) * deg_to_rad;

  geometry::LocalPoint point;
  point.x_m = dlon_rad * std::cos(lat0_rad) * kEarthRadiusM;
  point.y_m = dlat_rad * kEarthRadiusM;
  point.z_m = altitude_m - mission.scene_center_altitude_m;
  return point;
}

double DbsmToSquareMeters(double dbsm) { return std::pow(10.0, dbsm / 10.0); }

bool ValidateRuntimeConfigForStep(const config::SarSessionConfig& config, SarCycleResult* result) {
  if (config.hardware.bandwidth_hz <= 0.0 || config.hardware.sample_rate_hz <= 0.0 ||
      config.hardware.carrier_frequency_hz <= 0.0 ||
      config.hardware.pulse_repetition_frequency_hz <= 0.0 ||
      config.mission.platform_speed_mps <= 0.0 || config.mission.nominal_slant_range_m <= 0.0 ||
      config.mission.range_sample_count == 0U || config.mission.azimuth_pulse_count == 0U) {
    result->has_error = true;
    result->abort_reason = "invalid_config";
    result->diagnostics.push_back(
        MakeError("sar.invalid_config", "SAR runtime config contains non-positive fields."));
    return false;
  }

  if (config.policy.enable_l1_rda_imaging &&
      (config.mission.range_sample_count > kMaxSessionRdaRangeSamples ||
       config.mission.azimuth_pulse_count > kMaxSessionRdaPulses)) {
    result->has_error = true;
    result->abort_reason = "rda_size_gate";
    result->diagnostics.push_back(MakeError(
        "sar.rda_size_gate",
        "SAR session RDA size exceeds current Phase 1 runtime gate; use smaller validation "
        "scenes until performance approval."));
    return false;
  }
  if (config.policy.enable_l1_rda_imaging && !config.policy.enable_raw_echo_generation) {
    result->has_error = true;
    result->abort_reason = "rda_requires_raw_echo";
    result->diagnostics.push_back(
        MakeError("sar.rda_requires_raw_echo",
                  "SAR session RDA requires raw echo generation in the current Phase 1 pipeline."));
    return false;
  }
  return true;
}

bool BuildWaveformAndFilter(const config::SarSessionConfig& config, signal::LfmWaveform* waveform,
                            signal::ComplexVector* matched_filter) {
  signal::LfmWaveformConfig waveform_config;
  waveform_config.bandwidth_hz = config.hardware.bandwidth_hz;
  waveform_config.sample_rate_hz = config.hardware.sample_rate_hz;
  waveform_config.start_frequency_hz = 0.0;
  waveform_config.time_bandwidth_product =
      std::max(config.hardware.bandwidth_hz * config.hardware.pulse_width_s, 1.0);
  return signal::GenerateLfmWaveform(waveform_config, waveform) &&
         signal::BuildMatchedFilter(waveform->samples, matched_filter);
}

std::vector<echo::PointTarget> BuildLocalTargets(const SarCycleInput& input,
                                                 const config::SarMissionConfig& mission) {
  std::vector<echo::PointTarget> targets;
  targets.reserve(input.point_targets.size());
  for (const SarPointTarget& target : input.point_targets) {
    echo::PointTarget local_target;
    local_target.position_m =
        ToLocalPoint(target.latitude_deg, target.longitude_deg, target.altitude_m, mission);
    local_target.rcs_m2 = DbsmToSquareMeters(target.radar_cross_section_dbsm);
    targets.push_back(local_target);
  }
  return targets;
}

bool BuildRawPulseHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                          const signal::ComplexVector& transmit_waveform,
                          runtime::PulseRingBuffer* pulse_buffer, std::uint64_t* next_pulse_id,
                          double* pulse_fraction_carry, signal::ComplexMatrix* history,
                          SarCycleResult* result) {
  if (pulse_buffer == nullptr || next_pulse_id == nullptr || pulse_fraction_carry == nullptr) {
    result->has_error = true;
    result->abort_reason = "pulse_buffer_unavailable";
    result->diagnostics.push_back(
        MakeError("sar.pulse_buffer_unavailable", "SAR pulse ring buffer is unavailable."));
    return false;
  }

  const double requested_pulses =
      input.dt_sec * config.hardware.pulse_repetition_frequency_hz + *pulse_fraction_carry;
  std::size_t pulse_count_to_generate = static_cast<std::size_t>(std::floor(requested_pulses));
  *pulse_fraction_carry = requested_pulses - static_cast<double>(pulse_count_to_generate);
  if (pulse_buffer->size() < config.mission.azimuth_pulse_count) {
    pulse_count_to_generate = std::max(pulse_count_to_generate,
                                       config.mission.azimuth_pulse_count - pulse_buffer->size());
  }

  if (pulse_count_to_generate == 0U) {
    result->diagnostics.push_back(
        MakeInfo("sar.pulse_ring_buffer", "SAR pulse ring buffer reused latest aperture."));
  }

  geometry::StraightStripmapTrackConfig track_config;
  track_config.start_position_m =
      ToLocalPoint(input.platform.latitude_deg, input.platform.longitude_deg,
                   input.platform.altitude_m, config.mission);
  track_config.start_position_m.x_m += static_cast<double>(*next_pulse_id) *
                                       config.mission.platform_speed_mps /
                                       config.hardware.pulse_repetition_frequency_hz;
  track_config.velocity_x_mps = config.mission.platform_speed_mps;
  track_config.prf_hz = config.hardware.pulse_repetition_frequency_hz;
  track_config.pulse_count = static_cast<std::uint32_t>(pulse_count_to_generate);

  std::vector<geometry::PlatformPulseState> pulses;
  if (pulse_count_to_generate > 0U &&
      !geometry::GenerateStraightStripmapTrack(track_config, &pulses)) {
    result->has_error = true;
    result->abort_reason = "track_generation_failed";
    result->diagnostics.push_back(
        MakeError("sar.track_generation_failed", "SAR failed to generate L1 stripmap track."));
    return false;
  }

  const std::vector<echo::PointTarget> targets = BuildLocalTargets(input, config.mission);
  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = config.hardware.sample_rate_hz;
  echo_config.carrier_frequency_hz = config.hardware.carrier_frequency_hz;
  echo_config.range_sample_count = config.mission.range_sample_count;

  history->rows = pulses.size();
  history->cols = config.mission.range_sample_count;
  history->values.assign(history->rows * history->cols, signal::ComplexSample(0.0, 0.0));

  std::size_t clipping_count = 0U;
  for (std::size_t row = 0U; row < pulses.size(); ++row) {
    echo::RawEchoResult echo;
    if (!echo::GeneratePointTargetRawEcho(echo_config, pulses[row], targets, transmit_waveform,
                                          &echo)) {
      result->has_error = true;
      result->abort_reason = "raw_echo_failed";
      result->diagnostics.push_back(
          MakeError("sar.raw_echo_failed", "SAR failed to generate point-target raw echo."));
      return false;
    }
    if (echo.has_clipping) {
      ++clipping_count;
    }
    runtime::PulseRecord record;
    record.pulse_id = *next_pulse_id;
    record.samples = echo.samples;
    if (!pulse_buffer->Push(record)) {
      result->has_error = true;
      result->abort_reason = "pulse_buffer_push_failed";
      result->diagnostics.push_back(MakeError("sar.pulse_buffer_push_failed",
                                              "SAR failed to append raw pulse to ring buffer."));
      return false;
    }
    ++(*next_pulse_id);
  }

  if (clipping_count > 0U) {
    result->diagnostics.push_back(MakeInfo(
        "sar.raw_echo_clipping",
        "SAR raw echo clipping observed in " + std::to_string(clipping_count) + " pulses."));
  }

  std::vector<runtime::PulseRecord> latest_pulses;
  if (!pulse_buffer->ReadLatest(config.mission.azimuth_pulse_count, &latest_pulses)) {
    result->has_error = true;
    result->abort_reason = "pulse_history_unavailable";
    result->diagnostics.push_back(
        MakeError("sar.pulse_history_unavailable",
                  "SAR pulse ring buffer cannot provide a contiguous latest aperture."));
    return false;
  }

  history->rows = latest_pulses.size();
  history->cols = config.mission.range_sample_count;
  history->values.assign(history->rows * history->cols, signal::ComplexSample(0.0, 0.0));
  for (std::size_t row = 0U; row < latest_pulses.size(); ++row) {
    if (latest_pulses[row].samples.size() != history->cols) {
      result->has_error = true;
      result->abort_reason = "pulse_sample_count_mismatch";
      result->diagnostics.push_back(
          MakeError("sar.pulse_sample_count_mismatch",
                    "SAR pulse ring buffer returned a pulse with unexpected range sample count."));
      return false;
    }
    for (std::size_t col = 0U; col < history->cols; ++col) {
      (*history)(row, col) = latest_pulses[row].samples[col];
    }
  }

  result->diagnostics.push_back(
      MakeInfo("sar.pulse_ring_buffer",
               "SAR pulse ring buffer size=" + std::to_string(pulse_buffer->size()) +
                   ", generated=" + std::to_string(pulses.size()) +
                   ", overflow=" + (pulse_buffer->overflow_sticky() ? "true" : "false")));
  return true;
}

}  // namespace

struct SarSession::Impl {
  explicit Impl(const config::SarSessionConfig& initial_config)
      : runtime_config(initial_config),
        raw_pulse_buffer(std::max<std::size_t>(initial_config.mission.azimuth_pulse_count, 1U)) {}

  config::SarSessionConfig runtime_config;
  runtime::PulseRingBuffer raw_pulse_buffer;
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

SarSession SarSessionFactory::Create(const config::SarSessionConfig& config) {
  return SarSession(std::unique_ptr<SarSession::Impl>(new SarSession::Impl(config)));
}

SarOutputFrame SarSession::Step(const SarCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SarCycleResult SarSession::StepWithResult(const SarCycleInput& input) {
  SarCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.output_frame.cycle_index = input.cycle_index;

  if (input.dt_sec <= 0.0) {
    result.has_error = true;
    result.abort_reason = "invalid_dt_sec";
    result.diagnostics.push_back(
        MakeError("sar.invalid_dt_sec", "SAR cycle dt_sec must be positive."));
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return result;
  }

  result.output_frame.range_sample_count = impl_->runtime_config.mission.range_sample_count;
  result.output_frame.azimuth_pulse_count = impl_->runtime_config.mission.azimuth_pulse_count;
  result.output_frame.center_slant_range_m = impl_->runtime_config.mission.nominal_slant_range_m;
  result.output_frame.estimated_snr_db = 0.0;

  if (!ValidateRuntimeConfigForStep(impl_->runtime_config, &result)) {
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return result;
  }

  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!BuildWaveformAndFilter(impl_->runtime_config, &waveform, &matched_filter)) {
    result.has_error = true;
    result.abort_reason = "waveform_generation_failed";
    result.diagnostics.push_back(
        MakeError("sar.waveform_generation_failed", "SAR failed to generate LFM waveform."));
    return result;
  }

  signal::ComplexMatrix raw_history;
  if (impl_->runtime_config.policy.enable_raw_echo_generation) {
    if (!BuildRawPulseHistory(impl_->runtime_config, input, waveform.samples,
                              &impl_->raw_pulse_buffer, &impl_->next_pulse_id,
                              &impl_->pulse_fraction_carry, &raw_history, &result)) {
      return result;
    }
    result.output_frame.completed_stage = SarProcessingStage::kRawEcho;
    result.output_frame.has_raw_echo = true;
  }
  if (impl_->runtime_config.policy.enable_range_compression) {
    result.output_frame.completed_stage = SarProcessingStage::kRangeCompression;
    result.output_frame.has_range_compressed_echo = true;
  }
  if (impl_->runtime_config.policy.enable_l1_rda_imaging) {
    imaging::RdaConfig rda_config;
    rda_config.sample_rate_hz = impl_->runtime_config.hardware.sample_rate_hz;
    rda_config.carrier_frequency_hz = impl_->runtime_config.hardware.carrier_frequency_hz;
    rda_config.prf_hz = impl_->runtime_config.hardware.pulse_repetition_frequency_hz;
    rda_config.platform_velocity_mps = impl_->runtime_config.mission.platform_speed_mps;
    rda_config.reference_range_m = impl_->runtime_config.mission.nominal_slant_range_m;
    rda_config.rcmc_interpolation = imaging::RcmcInterpolation::kLinear;

    imaging::FocusedSarImage image;
    if (!imaging::FocusStripmapRda(rda_config, raw_history, matched_filter, &image)) {
      result.has_error = true;
      result.abort_reason = "rda_failed";
      result.diagnostics.push_back(MakeError("sar.rda_failed", "SAR RDA focus failed."));
      return result;
    }
    const std::size_t peak_index = imaging::FindPeakIndex(image.image);
    result.diagnostics.push_back(MakeInfo(
        "sar.rda_peak",
        "SAR RDA peak index " + std::to_string(peak_index) +
            ", doppler_rate_hz_per_s=" + std::to_string(image.diagnostics.doppler_rate_hz_per_s) +
            ", azimuth_width_3db_bins=" + std::to_string(image.diagnostics.azimuth_width_3db_bins) +
            ", image_entropy_nats=" + std::to_string(image.diagnostics.image_entropy_nats)));
    result.output_frame.completed_stage = SarProcessingStage::kL1RdaImage;
    result.output_frame.has_l1_image = true;
  }

  result.executed_this_cycle = true;
  impl_->previous_output = result.output_frame;
  impl_->has_previous_output = true;
  return result;
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
