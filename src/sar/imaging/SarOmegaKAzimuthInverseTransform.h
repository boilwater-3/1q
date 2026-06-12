/**
 * @file SarOmegaKAzimuthInverseTransform.h
 * @brief Numerical Omega-K azimuth inverse-transform executor.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_AZIMUTH_INVERSE_TRANSFORM_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_AZIMUTH_INVERSE_TRANSFORM_H_

#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class OmegaKAzimuthInverseStatus { kSucceeded = 0, kRejected = 1 };
enum class OmegaKAzimuthInverseReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidAxis = 2,
  kInvalidNormalization = 3,
  kInvalidMatrix = 4,
  kTransformFailure = 5,
};

struct OmegaKAzimuthInverseRequest {
  std::uint64_t request_id{0U};
  std::vector<double> absolute_slant_ranges_m;
  std::vector<double> output_azimuth_coordinates;
  double additional_normalization{0.0};
  signal::ComplexMatrix compensated_intermediate;
};

struct OmegaKAzimuthInverseResult {
  std::uint64_t request_id{0U};
  OmegaKAzimuthInverseStatus status{OmegaKAzimuthInverseStatus::kRejected};
  OmegaKAzimuthInverseReason reason{OmegaKAzimuthInverseReason::kNone};
  std::vector<double> absolute_slant_ranges_m;
  std::vector<double> output_azimuth_coordinates;
  double additional_normalization{0.0};
  signal::ComplexMatrix numerical_image_candidate;
};

OmegaKAzimuthInverseResult ExecuteOmegaKAzimuthInverseTransform(
    const OmegaKAzimuthInverseRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_AZIMUTH_INVERSE_TRANSFORM_H_
