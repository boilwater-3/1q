/**
 * @file SarAutofocusPhaseTruth.h
 * @brief 自聚焦残余相位误差注入与可观测真值诊断。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_AUTOFOCUS_PHASE_TRUTH_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_AUTOFOCUS_PHASE_TRUTH_H_

#include <cstddef>
#include <vector>

namespace sar {
namespace imaging {

/**
 * @brief 自聚焦相位真值注入配置（多项式相位误差）。
 */
struct AutofocusPhaseTruthConfig {
  std::size_t sample_count{0U}; /**< 样本数 */
  double constant_rad{0.0};     /**< 常数项相位（rad） */
  double linear_rad{0.0};       /**< 线性项相位（rad） */
  double quadratic_rad{0.0};    /**< 二次项相位（rad） */
  double cubic_rad{0.0};        /**< 三次项相位（rad） */
};

/**
 * @brief 自聚焦相位真值可观测性诊断。
 */
struct AutofocusPhaseTruthDiagnostics {
  bool valid{false};                             /**< 诊断是否有效 */
  std::size_t sample_count{0U};                  /**< 样本数 */
  double fitted_unobservable_constant_rad{0.0};  /**< 拟合的不可观测常数项（rad） */
  double fitted_unobservable_linear_rad{0.0};    /**< 拟合的不可观测线性项（rad） */
  double observable_rms_rad{0.0};                /**< 可观测相位 RMS（rad） */
  double observable_max_abs_rad{0.0};            /**< 可观测相位最大绝对值（rad） */
  double correction_rms_rad{0.0};                /**< 校正相位 RMS（rad） */
  double correction_max_abs_rad{0.0};            /**< 校正相位最大绝对值（rad） */
  double removal_residual_mean_rad{0.0};         /**< 去除后残差均值（rad） */
  double removal_residual_linear_projection_rad{0.0}; /**< 残差线性投影（rad） */
  std::vector<double> normalized_aperture_coordinates; /**< 归一化孔径坐标 */
  std::vector<double> raw_phase_error_rad;       /**< 原始相位误差（rad） */
  std::vector<double> unobservable_phase_rad;    /**< 不可观测相位（rad） */
  std::vector<double> observable_phase_error_rad;/**< 可观测相位误差（rad） */
  std::vector<double> correction_phase_rad;      /**< 校正相位（rad） */
};

/**
 * @brief 注入多项式相位误差并评估自聚焦可观测真值。
 * @param[in] config 相位真值配置。
 * @param[out] diagnostics 可观测性诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool EvaluateAutofocusPhaseTruth(const AutofocusPhaseTruthConfig& config,
                                 AutofocusPhaseTruthDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_AUTOFOCUS_PHASE_TRUTH_H_
