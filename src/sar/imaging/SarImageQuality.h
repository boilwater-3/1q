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

/**
 * @brief 主瓣宽度估计方法（3dB 或 20dB 下降点）。
 */
enum class MainlobeEstimationMethod {
  k3dB = 0,  /**< 3dB 下降点估计 */
  k20dB = 1, /**< 20dB 下降点估计 */
};

/**
 * @brief 图像质量评估配置。
 */
struct ImageQualityConfig {
  MainlobeEstimationMethod mainlobe_method{MainlobeEstimationMethod::k3dB}; /**< 主瓣估计方法 */
  double range_pixel_spacing_m{0.0};   /**< 距离像素间距（m） */
  double azimuth_pixel_spacing_m{0.0}; /**< 方位像素间距（m） */
  bool compute_contrast{true};         /**< 是否计算图像对比度 */
};

/**
 * @brief 复图像质量评估指标。
 */
struct ImageQualityMetrics {
  bool valid{false};                                  /**< 评估是否有效 */
  std::size_t peak_row{0U};                           /**< 峰值行索引 */
  std::size_t peak_col{0U};                           /**< 峰值列索引 */
  double peak_magnitude{0.0};                         /**< 峰值幅度 */
  MainlobeEstimationMethod mainlobe_method{MainlobeEstimationMethod::k3dB}; /**< 采用的主瓣估计方法 */
  double range_width_3db_bins{0.0};                   /**< 距离向 3dB 主瓣宽度（bin） */
  double azimuth_width_3db_bins{0.0};                 /**< 方位向 3dB 主瓣宽度（bin） */
  bool resolution_m_valid{false};                     /**< 物理分辨率是否有效 */
  double range_resolution_3db_m{0.0};                 /**< 距离向 3dB 分辨率（m） */
  double azimuth_resolution_3db_m{0.0};               /**< 方位向 3dB 分辨率（m） */
  double pslr_db{0.0};                                /**< 峰值旁瓣比 PSLR（dB） */
  double islr_db{0.0};                                /**< 积分旁瓣比 ISLR（dB） */
  double entropy_nats{0.0};                           /**< 图像熵（nats） */
  double image_contrast{0.0};                         /**< 图像对比度 */
};

/**
 * @brief 两幅复图像的比较指标。
 */
struct ImageComparisonMetrics {
  bool valid{false};                /**< 比较是否有效 */
  double phase_offset_rad{0.0};     /**< 全局相位偏移（rad） */
  double normalized_rms_error{0.0}; /**< 两幅图分别单位能量归一化并消除全局常数相位后的 L2 误差 */
  double coherent_correlation{0.0}; /**< 相干相关系数 */
};

/**
 * @brief 用默认配置评估复图像质量。
 * @param[in] image 待评估复图像。
 * @return 质量评估指标。
 */
ImageQualityMetrics EvaluateImageQuality(const signal::ComplexMatrix& image);
/**
 * @brief 用指定配置评估复图像质量。
 * @param[in] image 待评估复图像。
 * @param[in] config 评估配置。
 * @return 质量评估指标。
 */
ImageQualityMetrics EvaluateImageQuality(const signal::ComplexMatrix& image,
                                         const ImageQualityConfig& config);

/**
 * @brief 以全局相位参考比较两幅复图像。
 * @param[in] reference 参考复图像。
 * @param[in] candidate 待比较复图像。
 * @return 比较指标（含归一化 RMS 误差与相干相关）。
 */
ImageComparisonMetrics CompareImagesWithGlobalPhaseReference(
    const signal::ComplexMatrix& reference, const signal::ComplexMatrix& candidate);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_IMAGE_QUALITY_H_
