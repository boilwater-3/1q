/**
 * @file SarRadiometricCalibration.h
 * @brief SAR 内部图像响应辐射定标工具。
 */

#ifndef ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_
#define ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_

#include <vector>

namespace sar {
namespace calibration {

struct CalibrationSample {
  double known_rcs_m2{0.0};
  double image_power{0.0};
  double slant_range_m{0.0};
  double weight{1.0};
};

struct RadiometricCalibration {
  bool valid{false};
  double image_calibration_factor{0.0};
  double weight_sum{0.0};
  std::vector<double> residual_error_db{};
};

bool CalibrateSingle(const CalibrationSample& sample, RadiometricCalibration* calibration);

bool CalibrateMultiple(const std::vector<CalibrationSample>& samples,
                       RadiometricCalibration* calibration);

bool InvertRcs(double image_power, double slant_range_m,
               const RadiometricCalibration& calibration, double* measured_rcs_m2);

bool EvaluateRadiometricErrorDb(double measured_rcs_m2, double theoretical_rcs_m2,
                                double* error_db);

}  // namespace calibration
}  // namespace sar

#endif  // ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_
