/**
 * @file SarOmegaKPointTargetAcceptance.h
 * @brief Independent point-target acceptance evaluator for Omega-K candidates.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_POINT_TARGET_ACCEPTANCE_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_POINT_TARGET_ACCEPTANCE_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class OmegaKPointTargetAcceptanceStatus { kPassed = 0, kFailed = 1, kRejected = 2 };
enum class OmegaKPointTargetAcceptanceReason {
  kNone = 0,
  kInvalidRequest = 1,
  kTruthNotIndependent = 2,
  kOutsideCommonSupport = 3,
  kInvalidCandidate = 4,
};

struct OmegaKPointTargetTruth {
  bool independently_generated{false};
  bool inside_common_support{false};
  double absolute_slant_range_m{0.0};
  double azimuth_coordinate{0.0};
  double peak_phase_rad{0.0};
  double peak_magnitude{0.0};
  std::size_t range_mainlobe_half_width{0U};
  std::size_t azimuth_mainlobe_half_width{0U};
};

struct OmegaKPointTargetTolerances {
  double maximum_range_error_m{0.0};
  double maximum_azimuth_error{0.0};
  double maximum_abs_phase_error_rad{0.0};
  double maximum_relative_magnitude_error{0.0};
  double maximum_range_pslr_db{0.0};
  double maximum_azimuth_pslr_db{0.0};
  double maximum_range_islr_db{0.0};
  double maximum_azimuth_islr_db{0.0};
};

struct OmegaKPointTargetAcceptanceRequest {
  std::uint64_t request_id{0U};
  std::vector<double> absolute_slant_ranges_m;
  std::vector<double> azimuth_coordinates;
  signal::ComplexMatrix numerical_image_candidate;
  OmegaKPointTargetTruth truth;
  OmegaKPointTargetTolerances tolerances;
};

struct OmegaKPointTargetAcceptanceResult {
  std::uint64_t request_id{0U};
  OmegaKPointTargetAcceptanceStatus status{OmegaKPointTargetAcceptanceStatus::kRejected};
  OmegaKPointTargetAcceptanceReason reason{OmegaKPointTargetAcceptanceReason::kNone};
  std::size_t peak_row{0U};
  std::size_t peak_col{0U};
  double range_error_m{0.0};
  double azimuth_error{0.0};
  double wrapped_phase_error_rad{0.0};
  double relative_magnitude_error{0.0};
  double range_pslr_db{0.0};
  double azimuth_pslr_db{0.0};
  double range_islr_db{0.0};
  double azimuth_islr_db{0.0};
};

OmegaKPointTargetAcceptanceResult EvaluateOmegaKPointTargetCandidate(
    const OmegaKPointTargetAcceptanceRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_POINT_TARGET_ACCEPTANCE_H_
