/**
 * @file SarPgaPhaseGradientEstimator.h
 * @brief Bounded adjacent-sample PGA phase-gradient estimator.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PGA_PHASE_GRADIENT_ESTIMATOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PGA_PHASE_GRADIENT_ESTIMATOR_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class PgaPhaseGradientEstimatorStatus { kSucceeded = 0, kRejected = 1 };
enum class PgaPhaseGradientEstimatorReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidProfile = 2,
  kInvalidSupportMask = 3,
  kInsufficientValidPairs = 4,
};

struct PgaPhaseGradientEstimatorRequest {
  std::uint64_t request_id{0U};
  signal::ComplexVector aperture_profile;
  std::vector<std::uint8_t> support_mask;
  std::size_t minimum_valid_pair_count{0U};
};

struct PgaPhaseGradientEstimatorResult {
  std::uint64_t request_id{0U};
  PgaPhaseGradientEstimatorStatus status{PgaPhaseGradientEstimatorStatus::kRejected};
  PgaPhaseGradientEstimatorReason reason{PgaPhaseGradientEstimatorReason::kNone};
  std::size_t valid_pair_count{0U};
  std::vector<std::uint8_t> valid_pair_mask;
  std::vector<double> wrapped_gradient_rad;
};

PgaPhaseGradientEstimatorResult EstimatePgaPhaseGradient(
    const PgaPhaseGradientEstimatorRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PGA_PHASE_GRADIENT_ESTIMATOR_H_
