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

bool BuildCalibrationObservation(const CalibrationObservationRequest& request,
                                 const signal::ComplexMatrix& unnormalized_image,
                                 CalibrationObservation* observation) {
  if (observation == nullptr || request.observation_id.empty() ||
      !IsPositiveFinite(request.known_rcs_m2) || !IsPositiveFinite(request.slant_range_m) ||
      !IsPositiveFinite(request.weight) || request.image_is_normalized ||
      request.aperture_start_pulse_id > request.aperture_end_pulse_id ||
      unnormalized_image.rows == 0U || unnormalized_image.cols == 0U ||
      unnormalized_image.values.size() != unnormalized_image.rows * unnormalized_image.cols ||
      request.image_row >= unnormalized_image.rows || request.image_col >= unnormalized_image.cols) {
    return false;
  }
  const double image_power = std::norm(unnormalized_image(request.image_row, request.image_col));
  if (!IsPositiveFinite(image_power)) {
    return false;
  }
  observation->observation_id = request.observation_id;
  observation->known_rcs_m2 = request.known_rcs_m2;
  observation->image_power = image_power;
  observation->slant_range_m = request.slant_range_m;
  observation->image_row = request.image_row;
  observation->image_col = request.image_col;
  observation->aperture_start_pulse_id = request.aperture_start_pulse_id;
  observation->aperture_end_pulse_id = request.aperture_end_pulse_id;
  observation->weight = request.weight;
  return true;
}

bool ConvertObservationsToSamples(const std::vector<CalibrationObservation>& observations,
                                  std::vector<CalibrationSample>* samples) {
  if (samples == nullptr || observations.empty()) {
    return false;
  }
  std::vector<CalibrationSample> converted;
  converted.reserve(observations.size());
  for (const CalibrationObservation& observation : observations) {
    CalibrationSample sample;
    sample.known_rcs_m2 = observation.known_rcs_m2;
    sample.image_power = observation.image_power;
    sample.slant_range_m = observation.slant_range_m;
    sample.weight = observation.weight;
    double factor = 0.0;
    if (observation.observation_id.empty() ||
        observation.aperture_start_pulse_id > observation.aperture_end_pulse_id ||
        !SampleFactor(sample, &factor)) {
      return false;
    }
    converted.push_back(sample);
  }
  *samples = converted;
  return true;
}

bool ExecuteCalibrationRequests(CalibrationImagePath actual_image_path,
                                const signal::ComplexMatrix& unnormalized_image,
                                const std::vector<CalibrationExecutionRequest>& requests,
                                CalibrationExecutionResult* result) {
  if (result == nullptr) {
    return false;
  }
  CalibrationExecutionResult candidate;
  candidate.request_count = requests.size();
  if (requests.empty()) {
    candidate.failure = CalibrationExecutionFailure::kEmptyRequestList;
    *result = candidate;
    return false;
  }
  std::vector<CalibrationObservation> observations;
  observations.reserve(requests.size());
  for (const CalibrationExecutionRequest& request : requests) {
    if (request.request_id.empty()) {
      candidate.failure = CalibrationExecutionFailure::kInvalidRequest;
      *result = candidate;
      return false;
    }
    if (request.image_path != actual_image_path) {
      candidate.failure = CalibrationExecutionFailure::kImagePathMismatch;
      *result = candidate;
      return false;
    }
    CalibrationObservation observation;
    if (!BuildCalibrationObservation(request.observation, unnormalized_image, &observation)) {
      candidate.failure = CalibrationExecutionFailure::kObservationBuildFailed;
      *result = candidate;
      return false;
    }
    observations.push_back(observation);
  }

  std::vector<CalibrationSample> samples;
  if (!ConvertObservationsToSamples(observations, &samples)) {
    candidate.failure = CalibrationExecutionFailure::kObservationConversionFailed;
    *result = candidate;
    return false;
  }
  if (!CalibrateMultiple(samples, &candidate.calibration)) {
    candidate.failure = CalibrationExecutionFailure::kCalibrationFailed;
    *result = candidate;
    return false;
  }
  candidate.residuals.reserve(requests.size());
  for (std::size_t index = 0U; index < requests.size(); ++index) {
    CalibrationExecutionResidual residual;
    residual.request_id = requests[index].request_id;
    residual.observation_id = observations[index].observation_id;
    residual.residual_error_db = candidate.calibration.residual_error_db[index];
    candidate.residuals.push_back(residual);
  }
  candidate.valid = true;
  candidate.failure = CalibrationExecutionFailure::kNone;
  *result = candidate;
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
