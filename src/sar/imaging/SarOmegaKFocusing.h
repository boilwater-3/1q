/**
 * @file SarOmegaKFocusing.h
 * @brief Omega-K stripmap 聚焦编排器:把 front-end + 全部 Omega-K 部件串联成完整聚焦入口。
 *
 * 数据流(契约 §4.2): raw history → front-end(2D FFT + H_bulk) → Stolt 几何 →
 * 共同支持 → 网格收缩 → 相对延迟变换 → 参考映射 → 参考相位补偿 → 方位逆变换 → 图像。
 * 仅 L1 匀速直线 broadside 条带(契约 §7)。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_FOCUSING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_FOCUSING_H_

#include <string>

#include "sar/imaging/SarOmegaKAzimuthInverseTransform.h"
#include "sar/imaging/SarOmegaKCommonSupport.h"
#include "sar/imaging/SarOmegaKGeometry.h"
#include "sar/imaging/SarOmegaKGridReduction.h"
#include "sar/imaging/SarOmegaKReferenceMapping.h"
#include "sar/imaging/SarOmegaKReferencePhaseCompensation.h"
#include "sar/imaging/SarOmegaKRelativeDelayTransform.h"
#include "sar/imaging/SarOmegaKSpectrumFrontEnd.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

struct OmegaKConfig {
  std::size_t range_sample_count{0U};
  std::size_t azimuth_pulse_count{0U};
  double sample_rate_hz{0.0};
  double prf_hz{0.0};
  double carrier_frequency_hz{0.0};
  double platform_velocity_mps{0.0};
  double reference_range_m{0.0};
};

struct OmegaKDiagnostics {
  OmegaKSpectrumFrontEndResult front_end{};
  OmegaKCommonSupportDiagnostics common_support{};
  OmegaKGridReductionResult grid_reduction{};
  OmegaKRelativeDelayResult relative_delay{};
  OmegaKReferenceMappingResult reference_mapping{};
  OmegaKAzimuthInverseResult azimuth_inverse{};
  std::string failure_stage{"none"};
};

struct FocusedOmegaKImage {
  signal::ComplexMatrix image{};
  OmegaKDiagnostics diagnostics{};
};

bool FocusStripmapOmegaK(const OmegaKConfig& config,
                         const signal::ComplexMatrix& raw_pulse_history,
                         FocusedOmegaKImage* output);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_FOCUSING_H_
