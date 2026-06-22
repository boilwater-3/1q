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

struct AutofocusPhaseTruthConfig {
  std::size_t sample_count{0U};
  double constant_rad{0.0};
  double linear_rad{0.0};
  double quadratic_rad{0.0};
  double cubic_rad{0.0};
};

struct AutofocusPhaseTruthDiagnostics {
  bool valid{false};
  std::size_t sample_count{0U};
  double fitted_unobservable_constant_rad{0.0};
  double fitted_unobservable_linear_rad{0.0};
  double observable_rms_rad{0.0};
  double observable_max_abs_rad{0.0};
  double correction_rms_rad{0.0};
  double correction_max_abs_rad{0.0};
  double removal_residual_mean_rad{0.0};
  double removal_residual_linear_projection_rad{0.0};
  std::vector<double> normalized_aperture_coordinates;
  std::vector<double> raw_phase_error_rad;
  std::vector<double> unobservable_phase_rad;
  std::vector<double> observable_phase_error_rad;
  std::vector<double> correction_phase_rad;
};

bool EvaluateAutofocusPhaseTruth(const AutofocusPhaseTruthConfig& config,
                                 AutofocusPhaseTruthDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_AUTOFOCUS_PHASE_TRUTH_H_
