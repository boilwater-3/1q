/**
 * @file SarOmegaKReferenceMapping.h
 * @brief Omega-K reference metadata and absolute-range mapping executor.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_MAPPING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_MAPPING_H_

#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class OmegaKDelaySign {
  kUnspecified = 0,
  kPositiveIncreasesRange = 1,
  kPositiveDecreasesRange = 2,
};

enum class OmegaKReferencePhaseSign {
  kUnspecified = 0,
  kPositive = 1,
  kNegative = 2,
  kIdentity = 3,
};

enum class OmegaKReferenceMappingStatus { kSucceeded = 0, kRejected = 1 };
enum class OmegaKReferenceMappingReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidPhysicalMetadata = 2,
  kInvalidConvention = 3,
  kInvalidAxis = 4,
  kInvalidMatrix = 5,
  kInvalidAbsoluteRange = 6,
};

struct OmegaKReferenceMappingRequest {
  std::uint64_t request_id{0U};
  double propagation_speed_mps{0.0};
  double reference_slant_range_m{0.0};
  OmegaKDelaySign delay_sign{OmegaKDelaySign::kUnspecified};
  OmegaKReferencePhaseSign reference_phase_sign{OmegaKReferencePhaseSign::kUnspecified};
  double transform_normalization{0.0};
  std::vector<double> relative_delays_s;
  std::vector<double> azimuth_coordinates;
  signal::ComplexMatrix relative_delay_domain;
};

struct OmegaKReferenceMappingResult {
  std::uint64_t request_id{0U};
  OmegaKReferenceMappingStatus status{OmegaKReferenceMappingStatus::kRejected};
  OmegaKReferenceMappingReason reason{OmegaKReferenceMappingReason::kNone};
  std::vector<double> absolute_slant_ranges_m;
  std::vector<double> azimuth_coordinates;
  signal::ComplexMatrix referenced_intermediate;
};

OmegaKReferenceMappingResult ExecuteOmegaKReferenceMapping(
    const OmegaKReferenceMappingRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_MAPPING_H_
