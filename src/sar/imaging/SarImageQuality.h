/**
 * @file SarImageQuality.h
 * @brief SAR 内部复图像质量评估与跨算法比较工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_IMAGE_QUALITY_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_IMAGE_QUALITY_H_

#include <cstddef>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

struct ImageQualityMetrics {
  bool valid{false};
  std::size_t peak_row{0U};
  std::size_t peak_col{0U};
  double peak_magnitude{0.0};
  double range_width_3db_bins{0.0};
  double azimuth_width_3db_bins{0.0};
  double pslr_db{0.0};
  double islr_db{0.0};
  double entropy_nats{0.0};
};

struct ImageComparisonMetrics {
  bool valid{false};
  double phase_offset_rad{0.0};
  double normalized_rms_error{0.0};
  double coherent_correlation{0.0};
};

ImageQualityMetrics EvaluateImageQuality(const signal::ComplexMatrix& image);

ImageComparisonMetrics CompareImagesWithGlobalPhaseReference(
    const signal::ComplexMatrix& reference, const signal::ComplexMatrix& candidate);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_IMAGE_QUALITY_H_
