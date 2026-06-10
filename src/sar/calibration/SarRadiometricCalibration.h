/**
 * @file SarRadiometricCalibration.h
 * @brief SAR 内部图像响应辐射定标工具。
 */

#ifndef ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_
#define ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace calibration {

struct CalibrationObservationRequest {
  std::string observation_id{};
  double known_rcs_m2{0.0};
  double slant_range_m{0.0};
  std::size_t image_row{0U};
  std::size_t image_col{0U};
  std::uint64_t aperture_start_pulse_id{0U};
  std::uint64_t aperture_end_pulse_id{0U};
  double weight{1.0};
  bool image_is_normalized{false};
};

struct CalibrationObservation {
  std::string observation_id{};
  double known_rcs_m2{0.0};
  double image_power{0.0};
  double slant_range_m{0.0};
  std::size_t image_row{0U};
  std::size_t image_col{0U};
  std::uint64_t aperture_start_pulse_id{0U};
  std::uint64_t aperture_end_pulse_id{0U};
  double weight{1.0};
};

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

bool BuildCalibrationObservation(const CalibrationObservationRequest& request,
                                 const signal::ComplexMatrix& unnormalized_image,
                                 CalibrationObservation* observation);

bool ConvertObservationsToSamples(const std::vector<CalibrationObservation>& observations,
                                  std::vector<CalibrationSample>* samples);

bool InvertRcs(double image_power, double slant_range_m,
               const RadiometricCalibration& calibration, double* measured_rcs_m2);

bool EvaluateRadiometricErrorDb(double measured_rcs_m2, double theoretical_rcs_m2,
                                double* error_db);

}  // namespace calibration
}  // namespace sar

#endif  // ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_
