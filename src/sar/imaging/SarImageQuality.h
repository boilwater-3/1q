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

enum class MainlobeEstimationMethod {
  k3dB = 0,
  k20dB = 1,
};

struct ImageQualityConfig {
  MainlobeEstimationMethod mainlobe_method{MainlobeEstimationMethod::k3dB};
  double range_pixel_spacing_m{0.0};
  double azimuth_pixel_spacing_m{0.0};
  bool compute_contrast{true};
};

struct ImageQualityMetrics {
  bool valid{false};
  std::size_t peak_row{0U};
  std::size_t peak_col{0U};
  double peak_magnitude{0.0};
  MainlobeEstimationMethod mainlobe_method{MainlobeEstimationMethod::k3dB};
  double range_width_3db_bins{0.0};
  double azimuth_width_3db_bins{0.0};
  bool resolution_m_valid{false};
  double range_resolution_3db_m{0.0};
  double azimuth_resolution_3db_m{0.0};
  double pslr_db{0.0};
  double islr_db{0.0};
  double entropy_nats{0.0};
  double image_contrast{0.0};
};

struct ImageComparisonMetrics {
  bool valid{false};
  double phase_offset_rad{0.0};
  // 两幅图分别单位能量归一化并消除全局常数相位后的 L2 误差。
  double normalized_rms_error{0.0};
  double coherent_correlation{0.0};
};

ImageQualityMetrics EvaluateImageQuality(const signal::ComplexMatrix& image);
ImageQualityMetrics EvaluateImageQuality(const signal::ComplexMatrix& image,
                                         const ImageQualityConfig& config);

ImageComparisonMetrics CompareImagesWithGlobalPhaseReference(
    const signal::ComplexMatrix& reference, const signal::ComplexMatrix& candidate);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_IMAGE_QUALITY_H_
