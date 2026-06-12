/**
 * @file SarPgaSupportGradientTruth.h
 * @brief Deterministic PGA support selection and phase-gradient truth.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PGA_SUPPORT_GRADIENT_TRUTH_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PGA_SUPPORT_GRADIENT_TRUTH_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class PgaSupportGradientTruthStatus { kSucceeded = 0, kRejected = 1 };
enum class PgaSupportGradientTruthReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidThreshold = 2,
  kInvalidProfile = 3,
  kZeroEnergy = 4,
  kInsufficientSupport = 5,
  kInvalidPhaseTruth = 6,
};

struct PgaSupportGradientTruthRequest {
  std::uint64_t request_id{0U};
  signal::ComplexVector aperture_profile;
  std::vector<double> injected_phase_rad;
  double peak_relative_threshold{0.0};
  std::size_t minimum_supported_samples{0U};
};

struct PgaSupportGradientTruthResult {
  std::uint64_t request_id{0U};
  PgaSupportGradientTruthStatus status{PgaSupportGradientTruthStatus::kRejected};
  PgaSupportGradientTruthReason reason{PgaSupportGradientTruthReason::kNone};
  std::size_t peak_index{0U};
  std::size_t supported_sample_count{0U};
  std::vector<std::uint8_t> support_mask;
  std::vector<double> wrapped_forward_gradient_rad;
};

PgaSupportGradientTruthResult ExecutePgaSupportGradientTruth(
    const PgaSupportGradientTruthRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PGA_SUPPORT_GRADIENT_TRUTH_H_
