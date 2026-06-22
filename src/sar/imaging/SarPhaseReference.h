/**
 * @file SarPhaseReference.h
 * @brief SAR 内部相位重参考与全局常数相位估计工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PHASE_REFERENCE_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PHASE_REFERENCE_H_

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class PhaseReferenceMode {
  kNative = 0,
  kCenterBroadside = 1,
};

struct PhaseReferenceConfig {
  PhaseReferenceMode mode{PhaseReferenceMode::kCenterBroadside};
  double carrier_frequency_hz{0.0};
  double prf_hz{0.0};
  double platform_velocity_mps{0.0};
  double range_bin_spacing_m{0.0};
};

struct PhaseReferenceDiagnostics {
  bool applied{false};
  PhaseReferenceMode mode{PhaseReferenceMode::kNative};
  double min_phase_rad{0.0};
  double max_phase_rad{0.0};
};

bool NeedsPhaseReference(PhaseReferenceMode mode, bool* needs_reference);

bool ApplyBroadsideCenterPhaseReference(const PhaseReferenceConfig& config,
                                        signal::ComplexMatrix* image,
                                        PhaseReferenceDiagnostics* diagnostics);

bool EstimateGlobalPhaseOffset(const signal::ComplexMatrix& reference,
                               const signal::ComplexMatrix& candidate,
                               double* phase_offset_rad);

bool ApplyGlobalPhaseOffset(double phase_offset_rad, signal::ComplexMatrix* image);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PHASE_REFERENCE_H_
