#include "sar/imaging/SarImageQuality.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>

#include "sar/imaging/SarPhaseReference.h"

namespace sar {
namespace imaging {

namespace {

bool IsValidMatrix(const signal::ComplexMatrix& image) {
  return image.rows > 0U && image.cols > 0U && image.values.size() == image.rows * image.cols;
}

std::size_t FindPeak(const signal::ComplexMatrix& image) {
  std::size_t peak = 0U;
  double peak_power = -1.0;
  for (std::size_t index = 0U; index < image.values.size(); ++index) {
    const double power = std::norm(image.values[index]);
    if (power > peak_power) {
      peak_power = power;
      peak = index;
    }
  }
  return peak;
}

bool MainlobeThreshold(const signal::ComplexMatrix& image, std::size_t peak_row,
                       std::size_t peak_col, MainlobeEstimationMethod method,
                       double* threshold) {
  if (threshold == nullptr) {
    return false;
  }
  const double peak_power = std::norm(image(peak_row, peak_col));
  switch (method) {
    case MainlobeEstimationMethod::k3dB:
      *threshold = peak_power * 0.5;
      return true;
    case MainlobeEstimationMethod::k20dB:
      *threshold = peak_power * 0.01;
      return true;
  }
  return false;
}

void FindMainlobeBounds(const signal::ComplexMatrix& image, std::size_t peak_row,
                        std::size_t peak_col, double threshold, std::size_t* row_begin,
                        std::size_t* row_end, std::size_t* col_begin, std::size_t* col_end) {
  *row_begin = peak_row;
  while (*row_begin > 0U && std::norm(image(*row_begin - 1U, peak_col)) >= threshold) {
    --(*row_begin);
  }
  *row_end = peak_row;
  while (*row_end + 1U < image.rows && std::norm(image(*row_end + 1U, peak_col)) >= threshold) {
    ++(*row_end);
  }
  *col_begin = peak_col;
  while (*col_begin > 0U && std::norm(image(peak_row, *col_begin - 1U)) >= threshold) {
    --(*col_begin);
  }
  *col_end = peak_col;
  while (*col_end + 1U < image.cols && std::norm(image(peak_row, *col_end + 1U)) >= threshold) {
    ++(*col_end);
  }
}

double SafeDb(double numerator, double denominator, double multiplier) {
  if (numerator <= 0.0 || denominator <= 0.0) {
    return -std::numeric_limits<double>::infinity();
  }
  return multiplier * std::log10(numerator / denominator);
}

bool HasValidPixelSpacing(const ImageQualityConfig& config) {
  return config.range_pixel_spacing_m > 0.0 && std::isfinite(config.range_pixel_spacing_m) &&
         config.azimuth_pixel_spacing_m > 0.0 && std::isfinite(config.azimuth_pixel_spacing_m);
}

}  // namespace

ImageQualityMetrics EvaluateImageQuality(const signal::ComplexMatrix& image) {
  return EvaluateImageQuality(image, ImageQualityConfig{});
}

ImageQualityMetrics EvaluateImageQuality(const signal::ComplexMatrix& image,
                                         const ImageQualityConfig& config) {
  ImageQualityMetrics metrics;
  if (!IsValidMatrix(image)) {
    return metrics;
  }

  double total_power = 0.0;
  for (const signal::ComplexSample& sample : image.values) {
    total_power += std::norm(sample);
  }
  if (total_power <= 0.0 || !std::isfinite(total_power)) {
    return metrics;
  }

  const std::size_t peak_index = FindPeak(image);
  metrics.peak_row = peak_index / image.cols;
  metrics.peak_col = peak_index % image.cols;
  metrics.peak_magnitude = std::abs(image.values[peak_index]);
  metrics.mainlobe_method = config.mainlobe_method;

  double mainlobe_threshold = 0.0;
  if (!MainlobeThreshold(image, metrics.peak_row, metrics.peak_col, config.mainlobe_method,
                         &mainlobe_threshold)) {
    return ImageQualityMetrics{};
  }

  std::size_t row_begin = 0U;
  std::size_t row_end = 0U;
  std::size_t col_begin = 0U;
  std::size_t col_end = 0U;
  FindMainlobeBounds(image, metrics.peak_row, metrics.peak_col, mainlobe_threshold, &row_begin,
                     &row_end, &col_begin, &col_end);
  metrics.azimuth_width_3db_bins = static_cast<double>(row_end - row_begin + 1U);
  metrics.range_width_3db_bins = static_cast<double>(col_end - col_begin + 1U);
  if (HasValidPixelSpacing(config)) {
    metrics.range_resolution_3db_m = metrics.range_width_3db_bins * config.range_pixel_spacing_m;
    metrics.azimuth_resolution_3db_m =
        metrics.azimuth_width_3db_bins * config.azimuth_pixel_spacing_m;
    metrics.resolution_m_valid = true;
  }

  double mainlobe_power = 0.0;
  double sidelobe_power = 0.0;
  double sidelobe_peak = 0.0;
  double entropy = 0.0;
  double intensity_sum = 0.0;
  double intensity_square_sum = 0.0;
  for (std::size_t row = 0U; row < image.rows; ++row) {
    for (std::size_t col = 0U; col < image.cols; ++col) {
      const double power = std::norm(image(row, col));
      const double magnitude = std::sqrt(power);
      const bool in_mainlobe =
          row >= row_begin && row <= row_end && col >= col_begin && col <= col_end;
      if (in_mainlobe) {
        mainlobe_power += power;
      } else {
        sidelobe_power += power;
        sidelobe_peak = std::max(sidelobe_peak, magnitude);
      }
      if (power > 0.0) {
        const double probability = power / total_power;
        entropy -= probability * std::log(probability);
      }
      intensity_sum += magnitude;
      intensity_square_sum += magnitude * magnitude;
    }
  }

  metrics.pslr_db = SafeDb(sidelobe_peak, metrics.peak_magnitude, 20.0);
  metrics.islr_db = SafeDb(sidelobe_power, mainlobe_power, 10.0);
  metrics.entropy_nats = entropy;
  if (config.compute_contrast) {
    const double count = static_cast<double>(image.values.size());
    const double mean = intensity_sum / count;
    const double mean_square = intensity_square_sum / count;
    const double variance = std::max(0.0, mean_square - mean * mean);
    metrics.image_contrast = mean > 0.0 ? std::sqrt(variance) / mean : 0.0;
  }
  metrics.valid = true;
  return metrics;
}

ImageComparisonMetrics CompareImagesWithGlobalPhaseReference(
    const signal::ComplexMatrix& reference, const signal::ComplexMatrix& candidate) {
  ImageComparisonMetrics metrics;
  if (!IsValidMatrix(reference) || !IsValidMatrix(candidate) || reference.rows != candidate.rows ||
      reference.cols != candidate.cols) {
    return metrics;
  }

  double phase_offset_rad = 0.0;
  if (!EstimateGlobalPhaseOffset(reference, candidate, &phase_offset_rad)) {
    return metrics;
  }

  signal::ComplexSample cross(0.0, 0.0);
  double reference_power = 0.0;
  double candidate_power = 0.0;
  for (std::size_t index = 0U; index < reference.values.size(); ++index) {
    cross += reference.values[index] * std::conj(candidate.values[index]);
    reference_power += std::norm(reference.values[index]);
    candidate_power += std::norm(candidate.values[index]);
  }
  metrics.phase_offset_rad = phase_offset_rad;
  const signal::ComplexSample phase_rotation(std::cos(phase_offset_rad),
                                             std::sin(phase_offset_rad));
  const double reference_norm = std::sqrt(reference_power);
  const double candidate_norm = std::sqrt(candidate_power);
  double error_power = 0.0;
  for (std::size_t index = 0U; index < reference.values.size(); ++index) {
    error_power += std::norm(reference.values[index] / reference_norm -
                             candidate.values[index] * phase_rotation / candidate_norm);
  }
  metrics.normalized_rms_error = std::sqrt(error_power);
  metrics.coherent_correlation = std::abs(cross) / std::sqrt(reference_power * candidate_power);
  metrics.valid = true;
  return metrics;
}

}  // namespace imaging
}  // namespace sar
