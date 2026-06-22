#include <cmath>
#include <cstddef>

#include "sar/imaging/SarPgaGradientTruthComparison.h"
#include "sar/imaging/SarPgaPhaseGradientEstimator.h"
#include "sar/imaging/SarPgaSupportGradientTruth.h"

int main() {
  sar::imaging::PgaSupportGradientTruthRequest truth_request;
  truth_request.request_id = 1U;
  truth_request.aperture_profile.push_back(std::polar(1.0, 0.0));
  truth_request.aperture_profile.push_back(std::polar(2.0, 0.5));
  truth_request.aperture_profile.push_back(std::polar(3.0, 1.0));
  truth_request.aperture_profile.push_back(std::polar(4.0, 2.0));
  truth_request.injected_phase_rad.push_back(0.0);
  truth_request.injected_phase_rad.push_back(0.5);
  truth_request.injected_phase_rad.push_back(1.0);
  truth_request.injected_phase_rad.push_back(2.0);
  truth_request.peak_relative_threshold = 0.25;
  truth_request.minimum_supported_samples = 4U;
  const sar::imaging::PgaSupportGradientTruthResult truth =
      sar::imaging::ExecutePgaSupportGradientTruth(truth_request);
  if (truth.status != sar::imaging::PgaSupportGradientTruthStatus::kSucceeded) {
    return 1;
  }

  sar::imaging::PgaPhaseGradientEstimatorRequest estimate_request;
  estimate_request.request_id = 2U;
  estimate_request.aperture_profile = truth_request.aperture_profile;
  estimate_request.support_mask = truth.support_mask;
  estimate_request.minimum_valid_pair_count = 3U;
  const sar::imaging::PgaPhaseGradientEstimatorResult estimate =
      sar::imaging::EstimatePgaPhaseGradient(estimate_request);
  if (estimate.status != sar::imaging::PgaPhaseGradientEstimatorStatus::kSucceeded) {
    return 2;
  }

  sar::imaging::PgaGradientComparisonRequest comparison_request;
  comparison_request.request_id = 3U;
  comparison_request.estimated_wrapped_gradient_rad = estimate.wrapped_gradient_rad;
  comparison_request.estimator_valid_pair_mask = estimate.valid_pair_mask;
  comparison_request.truth_wrapped_gradient_rad = truth.wrapped_forward_gradient_rad;
  comparison_request.truth_valid_pair_mask.assign(
      truth.wrapped_forward_gradient_rad.size(), 1U);
  comparison_request.minimum_jointly_valid_pair_count = 3U;
  comparison_request.maximum_abs_wrapped_error_tolerance_rad = 1.0e-12;
  comparison_request.rms_wrapped_error_tolerance_rad = 1.0e-12;
  const sar::imaging::PgaGradientComparisonResult comparison =
      sar::imaging::ComparePgaGradientTruth(comparison_request);
  return comparison.status == sar::imaging::PgaGradientComparisonStatus::kPassed ? 0 : 3;
}
