#include "sar/session/SarImagingExecutor.h"

#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarMotionCompensation.h"
#include "sar/imaging/SarRda.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

SarDiagnosticIssue MakeInfo(const char* code, const std::string& message) {
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  return issue;
}

// 记录结构化中止错误：设置 has_error、以 tag 作为 abort_reason、追加 code 为
// "sar."+tag 的 Error 诊断。集中此三件套模式，确保 abort_reason 与 diagnostic code
// 始终一致，避免散落字符串字面量产生拼写漂移。
void RecordAbort(SarCycleResult* result, const std::string& tag, const std::string& message) {
  result->has_error = true;
  result->abort_reason = tag;
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kError;
  issue.code = "sar." + tag;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));
}

bool CopyFocusedImage(const signal::ComplexMatrix& source, SarFocusedImageSource image_source,
                      SarFocusedImage* output) {
  if (output == nullptr || source.rows == 0U || source.cols == 0U ||
      source.values.size() != source.rows * source.cols ||
      source.rows > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) ||
      source.cols > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return false;
  }

  SarFocusedImage image;
  image.source = image_source;
  image.row_count = static_cast<std::uint32_t>(source.rows);
  image.column_count = static_cast<std::uint32_t>(source.cols);
  image.real_values.reserve(source.values.size());
  image.imaginary_values.reserve(source.values.size());
  for (const signal::ComplexSample& sample : source.values) {
    image.real_values.push_back(sample.real());
    image.imaginary_values.push_back(sample.imag());
  }
  *output = std::move(image);
  return true;
}

// 根据 policy 决定是否拷贝完整聚焦图像。retain=false 时仅写入占位元数据，
// 跳过大图拷贝；调用方仍可从 row_count/column_count/source 获知图像形状。
bool ExportFocusedImage(const config::SarPolicyConfig& policy,
                        const signal::ComplexMatrix& source, SarFocusedImageSource image_source,
                        SarFocusedImage* output) {
  if (output == nullptr) {
    return false;
  }
  if (!policy.retain_focused_image) {
    SarFocusedImage placeholder;
    placeholder.source = image_source;
    placeholder.row_count = static_cast<std::uint32_t>(source.rows);
    placeholder.column_count = static_cast<std::uint32_t>(source.cols);
    placeholder.is_placeholder = true;
    *output = std::move(placeholder);
    return true;
  }
  return CopyFocusedImage(source, image_source, output);
}

}  // namespace

bool ExecuteL1RdaImaging(const config::SarSessionConfig& config,
                         const signal::ComplexMatrix& raw_history,
                         const signal::ComplexVector& matched_filter,
                         const std::deque<geometry::PlatformPulseState>& ideal_trajectory_buffer,
                         const std::deque<geometry::PlatformPulseState>& actual_trajectory_buffer,
                         SarCycleResult* result) {
  signal::ComplexMatrix rda_input = raw_history;
  if (config.policy.enable_l2_motion_compensation) {
    if (ideal_trajectory_buffer.size() != raw_history.rows ||
        actual_trajectory_buffer.size() != raw_history.rows) {
      RecordAbort(result, "l2_trajectory_history_mismatch",
                  "SAR L2 trajectory history does not match the latest raw aperture.");
      return false;
    }
    const std::vector<geometry::PlatformPulseState> ideal_trajectory(ideal_trajectory_buffer.begin(),
                                                                     ideal_trajectory_buffer.end());
    const std::vector<geometry::PlatformPulseState> actual_trajectory(
        actual_trajectory_buffer.begin(), actual_trajectory_buffer.end());
    imaging::FirstOrderMotionCompensationConfig compensation_config;
    compensation_config.sample_rate_hz = config.hardware.sample_rate_hz;
    compensation_config.carrier_frequency_hz = config.hardware.carrier_frequency_hz;
    compensation_config.reference_point_m.y_m = config.mission.nominal_slant_range_m;
    imaging::MotionCompensationDiagnostics compensation_diagnostics;
    if (!imaging::ApplyFirstOrderMotionCompensation(compensation_config, ideal_trajectory,
                                                    actual_trajectory, raw_history, &rda_input,
                                                    &compensation_diagnostics)) {
      RecordAbort(result, "motion_compensation_failed", "SAR L2 motion compensation failed.");
      return false;
    }
    result->diagnostics.push_back(MakeInfo(
        "sar.motion_compensation",
        "SAR first-order motion compensation max_abs_range_error_m=" +
            std::to_string(compensation_diagnostics.max_abs_range_error_m) +
            ", rms_range_error_m=" + std::to_string(compensation_diagnostics.rms_range_error_m) +
            ", max_abs_envelope_shift_bins=" +
            std::to_string(compensation_diagnostics.max_abs_envelope_shift_bins)));
  }
  imaging::RdaConfig rda_config;
  rda_config.sample_rate_hz = config.hardware.sample_rate_hz;
  rda_config.carrier_frequency_hz = config.hardware.carrier_frequency_hz;
  rda_config.prf_hz = config.hardware.pulse_repetition_frequency_hz;
  rda_config.platform_velocity_mps = config.mission.platform_speed_mps;
  rda_config.reference_range_m = config.mission.nominal_slant_range_m;
  rda_config.rcmc_interpolation = imaging::RcmcInterpolation::kLinear;

  imaging::FocusedSarImage image;
  if (!imaging::FocusStripmapRda(rda_config, rda_input, matched_filter, &image)) {
    RecordAbort(result, "rda_failed", "SAR RDA focus failed.");
    return false;
  }
  if (!ExportFocusedImage(config.policy, image.image, SarFocusedImageSource::kL1Rda,
                          &result->focused_image)) {
    RecordAbort(result, "rda_public_image_export_failed",
                "SAR RDA image could not be converted to the public focused-image payload.");
    return false;
  }
  const std::size_t peak_index = imaging::FindPeakIndex(image.image);
  result->output_frame.phase_reference_mode = SarPhaseReferenceMode::kCenterBroadside;
  result->output_frame.image_quality_mainlobe_method = SarMainlobeEstimationMethod::k3dB;
  result->output_frame.range_width_3db_bins = image.diagnostics.range_width_3db_bins;
  result->output_frame.azimuth_width_3db_bins = image.diagnostics.azimuth_width_3db_bins;
  result->output_frame.range_resolution_3db_m = image.diagnostics.range_resolution_3db_m;
  result->output_frame.azimuth_resolution_3db_m = image.diagnostics.azimuth_resolution_3db_m;
  result->output_frame.image_entropy_nats = image.diagnostics.image_entropy_nats;
  result->output_frame.image_contrast = image.diagnostics.image_contrast;
  result->output_frame.has_image_quality_metrics = true;
  result->output_frame.image_resolution_m_valid = image.diagnostics.resolution_m_valid;
  result->output_frame.phase_reference_applied = image.diagnostics.phase_reference_applied;
  result->diagnostics.push_back(MakeInfo(
      "sar.rda_peak",
      "SAR RDA peak index " + std::to_string(peak_index) +
          ", doppler_rate_hz_per_s=" + std::to_string(image.diagnostics.doppler_rate_hz_per_s) +
          ", azimuth_sample_spacing_m=" +
          std::to_string(image.diagnostics.azimuth_sample_spacing_m) +
          ", azimuth_phase_curvature_rad_per_pulse2=" +
          std::to_string(image.diagnostics.azimuth_phase_curvature_rad_per_pulse2) +
          ", azimuth_quadratic_phase_span_rad=" +
          std::to_string(image.diagnostics.azimuth_quadratic_phase_span_rad) +
          ", max_geometric_doppler_hz=" +
          std::to_string(image.diagnostics.max_geometric_doppler_hz) +
          ", doppler_nyquist_margin=" + std::to_string(image.diagnostics.doppler_nyquist_margin) +
          ", phase_reference_mode=" + image.diagnostics.phase_reference_mode +
          ", phase_reference_applied=" +
          std::to_string(image.diagnostics.phase_reference_applied ? 1 : 0) +
          ", range_width_3db_bins=" + std::to_string(image.diagnostics.range_width_3db_bins) +
          ", azimuth_width_3db_bins=" + std::to_string(image.diagnostics.azimuth_width_3db_bins) +
          ", range_resolution_3db_m=" +
          std::to_string(image.diagnostics.range_resolution_3db_m) +
          ", azimuth_resolution_3db_m=" +
          std::to_string(image.diagnostics.azimuth_resolution_3db_m) +
          ", image_entropy_nats=" + std::to_string(image.diagnostics.image_entropy_nats) +
          ", image_contrast=" + std::to_string(image.diagnostics.image_contrast)));
  result->output_frame.completed_stage = SarProcessingStage::kL1RdaImage;
  result->output_frame.has_l1_image = true;
  return true;
}

bool ExecuteL3BpImaging(const config::SarSessionConfig& config,
                        const signal::ComplexMatrix& raw_history,
                        const signal::ComplexVector& matched_filter,
                        const std::deque<geometry::PlatformPulseState>& actual_trajectory_buffer,
                        SarCycleResult* result) {
  if (actual_trajectory_buffer.size() != raw_history.rows) {
    RecordAbort(result, "l3_trajectory_history_mismatch",
                "SAR L3 trajectory history does not match the latest raw aperture.");
    return false;
  }
  const std::vector<geometry::PlatformPulseState> actual_trajectory(
      actual_trajectory_buffer.begin(), actual_trajectory_buffer.end());
  imaging::GbpConfig bp_config;
  bp_config.sample_rate_hz = config.hardware.sample_rate_hz;
  bp_config.carrier_frequency_hz = config.hardware.carrier_frequency_hz;
  bp_config.grid.azimuth_pixel_count = config.mission.azimuth_pulse_count;
  bp_config.grid.range_pixel_count = config.mission.range_sample_count;
  bp_config.grid.azimuth_spacing_m = config.mission.platform_speed_mps /
                                     config.hardware.pulse_repetition_frequency_hz;
  bp_config.grid.range_spacing_m =
      kSpeedOfLightMps / (2.0 * config.hardware.sample_rate_hz);
  bp_config.grid.azimuth_start_m =
      -0.5 * static_cast<double>(bp_config.grid.azimuth_pixel_count - 1U) *
      bp_config.grid.azimuth_spacing_m;
  imaging::FocusedGbpImage image;
  if (!imaging::FocusSmallSceneBp(bp_config, actual_trajectory, raw_history, matched_filter,
                                  &image)) {
    RecordAbort(result, "l3_bp_failed", "SAR L3 BP focus failed.");
    return false;
  }
  if (!ExportFocusedImage(config.policy, image.image, SarFocusedImageSource::kL3Bp,
                          &result->focused_image)) {
    RecordAbort(result, "bp_public_image_export_failed",
                "SAR BP image could not be converted to the public focused-image payload.");
    return false;
  }
  const imaging::ImageQualityMetrics quality = imaging::EvaluateImageQuality(image.image);
  result->output_frame.phase_reference_mode = SarPhaseReferenceMode::kNative;
  result->output_frame.image_quality_mainlobe_method = SarMainlobeEstimationMethod::k3dB;
  result->output_frame.range_width_3db_bins = quality.range_width_3db_bins;
  result->output_frame.azimuth_width_3db_bins = quality.azimuth_width_3db_bins;
  result->output_frame.image_entropy_nats = quality.entropy_nats;
  result->output_frame.image_contrast = quality.image_contrast;
  result->output_frame.has_image_quality_metrics = quality.valid;
  result->output_frame.image_resolution_m_valid = false;
  result->output_frame.phase_reference_applied = false;
  result->diagnostics.push_back(MakeInfo(
      "sar.bp_peak", "SAR BP peak_row=" + std::to_string(quality.peak_row) +
                         ", peak_col=" + std::to_string(quality.peak_col) +
                         ", image_entropy_nats=" + std::to_string(quality.entropy_nats)));
  result->diagnostics.push_back(
      MakeInfo("sar.bp_traversal", "SAR BP traversal=" + image.diagnostics.traversal_order));
  result->output_frame.completed_stage = SarProcessingStage::kL3BpImage;
  result->output_frame.has_l3_bp_image = true;
  return true;
}

}  // namespace session
}  // namespace sar
