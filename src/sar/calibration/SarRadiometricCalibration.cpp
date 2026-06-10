#include "sar/calibration/SarRadiometricCalibration.h"

#include <cmath>

namespace sar {
namespace calibration {

namespace {

bool IsPositiveFinite(double value) { return value > 0.0 && std::isfinite(value); }

bool SampleFactor(const CalibrationSample& sample, double* factor) {
  if (factor == nullptr || !IsPositiveFinite(sample.known_rcs_m2) ||
      !IsPositiveFinite(sample.image_power) || !IsPositiveFinite(sample.slant_range_m) ||
      !IsPositiveFinite(sample.weight)) {
    return false;
  }
  const double range_squared = sample.slant_range_m * sample.slant_range_m;
  const double range_fourth = range_squared * range_squared;
  *factor = sample.known_rcs_m2 / (sample.image_power * range_fourth);
  return IsPositiveFinite(*factor);
}

}  // namespace

bool CalibrateSingle(const CalibrationSample& sample, RadiometricCalibration* calibration) {
  if (calibration == nullptr) {
    return false;
  }
  return CalibrateMultiple(std::vector<CalibrationSample>(1U, sample), calibration);
}

bool CalibrateMultiple(const std::vector<CalibrationSample>& samples,
                       RadiometricCalibration* calibration) {
  if (calibration == nullptr || samples.empty()) {
    return false;
  }
  *calibration = RadiometricCalibration{};
  double weighted_factor_sum = 0.0;
  for (const CalibrationSample& sample : samples) {
    double factor = 0.0;
    if (!SampleFactor(sample, &factor)) {
      return false;
    }
    weighted_factor_sum += sample.weight * factor;
    calibration->weight_sum += sample.weight;
  }
  if (!IsPositiveFinite(weighted_factor_sum) || !IsPositiveFinite(calibration->weight_sum)) {
    return false;
  }
  calibration->image_calibration_factor = weighted_factor_sum / calibration->weight_sum;
  if (!IsPositiveFinite(calibration->image_calibration_factor)) {
    return false;
  }

  calibration->valid = true;
  calibration->residual_error_db.reserve(samples.size());
  for (const CalibrationSample& sample : samples) {
    double measured_rcs_m2 = 0.0;
    double error_db = 0.0;
    if (!InvertRcs(sample.image_power, sample.slant_range_m, *calibration, &measured_rcs_m2) ||
        !EvaluateRadiometricErrorDb(measured_rcs_m2, sample.known_rcs_m2, &error_db)) {
      return false;
    }
    calibration->residual_error_db.push_back(error_db);
  }
  return true;
}

bool InvertRcs(double image_power, double slant_range_m,
               const RadiometricCalibration& calibration, double* measured_rcs_m2) {
  if (measured_rcs_m2 == nullptr || !calibration.valid ||
      !IsPositiveFinite(calibration.image_calibration_factor) || !IsPositiveFinite(image_power) ||
      !IsPositiveFinite(slant_range_m)) {
    return false;
  }
  const double range_squared = slant_range_m * slant_range_m;
  const double range_fourth = range_squared * range_squared;
  *measured_rcs_m2 = calibration.image_calibration_factor * image_power * range_fourth;
  return IsPositiveFinite(*measured_rcs_m2);
}

bool EvaluateRadiometricErrorDb(double measured_rcs_m2, double theoretical_rcs_m2,
                                double* error_db) {
  if (error_db == nullptr || !IsPositiveFinite(measured_rcs_m2) ||
      !IsPositiveFinite(theoretical_rcs_m2)) {
    return false;
  }
  *error_db = 10.0 * std::log10(measured_rcs_m2 / theoretical_rcs_m2);
  return std::isfinite(*error_db);
}

}  // namespace calibration
}  // namespace sar
