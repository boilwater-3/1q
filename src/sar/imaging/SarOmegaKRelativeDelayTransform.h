/**
 * @file SarOmegaKRelativeDelayTransform.h
 * @brief Omega-K 收缩频谱相对延迟变换执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_H_

#include <cstdint>
#include <vector>

#include "sar/imaging/SarOmegaKReducedRangeAxis.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class OmegaKRelativeDelayStatus { kSucceeded = 0, kRejected = 1 };
enum class OmegaKRelativeDelayReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidFrequencyAxis = 2,
  kInvalidSpectrum = 3,
  kTransformFailure = 4,
};

struct OmegaKRelativeDelayRequest {
  std::uint64_t request_id{0U};
  std::vector<double> reduced_range_frequencies_hz;
  signal::ComplexMatrix reduced_spectrum;
};

struct OmegaKRelativeDelayResult {
  std::uint64_t request_id{0U};
  OmegaKRelativeDelayStatus status{OmegaKRelativeDelayStatus::kRejected};
  OmegaKRelativeDelayReason reason{OmegaKRelativeDelayReason::kNone};
  OmegaKReducedRangeAxisDiagnostics axis_diagnostics;
  signal::ComplexMatrix relative_delay_domain;
};

OmegaKRelativeDelayResult ExecuteOmegaKRelativeDelayTransform(
    const OmegaKRelativeDelayRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_H_
