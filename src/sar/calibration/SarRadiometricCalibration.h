// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

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

enum class CalibrationImagePath {
  kRda = 0,
  kGbp = 1,
  kBp = 2,
};

enum class CalibrationExecutionFailure {
  kNone = 0,
  kEmptyRequestList = 1,
  kInvalidRequest = 2,
  kImagePathMismatch = 3,
  kObservationBuildFailed = 4,
  kObservationConversionFailed = 5,
  kCalibrationFailed = 6,
};

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

struct CalibrationExecutionRequest {
  std::string request_id{};
  CalibrationImagePath image_path{CalibrationImagePath::kRda};
  CalibrationObservationRequest observation{};
};

struct CalibrationExecutionResidual {
  std::string request_id{};
  std::string observation_id{};
  double residual_error_db{0.0};
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

struct CalibrationExecutionResult {
  bool valid{false};
  CalibrationExecutionFailure failure{CalibrationExecutionFailure::kNone};
  std::size_t request_count{0U};
  RadiometricCalibration calibration{};
  std::vector<CalibrationExecutionResidual> residuals{};
};

bool CalibrateSingle(const CalibrationSample& sample, RadiometricCalibration* calibration);

bool CalibrateMultiple(const std::vector<CalibrationSample>& samples,
                       RadiometricCalibration* calibration);

bool BuildCalibrationObservation(const CalibrationObservationRequest& request,
                                 const signal::ComplexMatrix& unnormalized_image,
                                 CalibrationObservation* observation);

bool ConvertObservationsToSamples(const std::vector<CalibrationObservation>& observations,
                                  std::vector<CalibrationSample>* samples);

bool ExecuteCalibrationRequests(CalibrationImagePath actual_image_path,
                                const signal::ComplexMatrix& unnormalized_image,
                                const std::vector<CalibrationExecutionRequest>& requests,
                                CalibrationExecutionResult* result);

bool InvertRcs(double image_power, double slant_range_m,
               const RadiometricCalibration& calibration, double* measured_rcs_m2);

bool EvaluateRadiometricErrorDb(double measured_rcs_m2, double theoretical_rcs_m2,
                                double* error_db);

}  // namespace calibration
}  // namespace sar

#endif  // ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_
