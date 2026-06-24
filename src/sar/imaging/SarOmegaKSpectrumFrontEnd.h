/**
 * @file SarOmegaKSpectrumFrontEnd.h
 * @brief Omega-K 前端执行器:从 raw baseband history 产生 2D 波数谱(source_spectrum)。
 *
 * front-end = raw history 的 2D FFT(距离正向 + 方位正向) + bulk 参考函数 H_bulk。
 * 距离压缩被吸收进 H_bulk 的 K_r 依赖,不单独调用 matched filter(契约 §3.2)。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_SPECTRUM_FRONT_END_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_SPECTRUM_FRONT_END_H_

#include <cstdint>

#include "sar/imaging/SarOmegaKGeometry.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class OmegaKSpectrumFrontEndStatus { kSucceeded = 0, kRejected = 1 };
enum class OmegaKSpectrumFrontEndReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidConfig = 2,
  kInvalidRawHistory = 3,
  kGeometryFailure = 4,
  kTransformFailure = 5,
};

struct OmegaKSpectrumFrontEndRequest {
  std::uint64_t request_id{0U};
  OmegaKGeometryConfig config;
  signal::ComplexMatrix raw_pulse_history;
};

struct OmegaKSpectrumFrontEndResult {
  std::uint64_t request_id{0U};
  OmegaKSpectrumFrontEndStatus status{OmegaKSpectrumFrontEndStatus::kRejected};
  OmegaKSpectrumFrontEndReason reason{OmegaKSpectrumFrontEndReason::kNone};
  OmegaKGeometryDiagnostics geometry;
  signal::ComplexMatrix source_spectrum;
};

OmegaKSpectrumFrontEndResult ExecuteOmegaKSpectrumFrontEnd(
    const OmegaKSpectrumFrontEndRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_SPECTRUM_FRONT_END_H_
