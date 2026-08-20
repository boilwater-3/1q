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

/**
 * @brief 定标所对应的成像路径。
 */
enum class CalibrationImagePath {
  kRda = 0, /**< 距离-多普勒（RDA）成像路径 */
  kGbp = 1, /**< 小场景全局后向投影（GBP）参考路径 */
  kBp = 2,   /**< 逐脉冲后向投影（BP）路径 */
};

/**
 * @brief 定标执行失败原因编码。
 */
enum class CalibrationExecutionFailure {
  kNone = 0,                      /**< 无错误 */
  kEmptyRequestList = 1,          /**< 请求列表为空 */
  kInvalidRequest = 2,            /**< 请求字段非法 */
  kImagePathMismatch = 3,         /**< 请求的成像路径与实际图像不符 */
  kObservationBuildFailed = 4,    /**< 观测构造失败 */
  kObservationConversionFailed = 5, /**< 观测转样本失败 */
  kCalibrationFailed = 6,          /**< 定标计算失败 */
};

/**
 * @brief 单个定标观测请求（已知 RCS 角反射器在图像中的位置）。
 */
struct CalibrationObservationRequest {
  std::string observation_id{};                  /**< 观测标识 */
  double known_rcs_m2{0.0};                       /**< 已知目标 RCS（m²） */
  double slant_range_m{0.0};                      /**< 目标斜距（m） */
  std::size_t image_row{0U};                      /**< 图像行索引 */
  std::size_t image_col{0U};                      /**< 图像列索引 */
  std::uint64_t aperture_start_pulse_id{0U};      /**< 孔径起始脉冲 ID */
  std::uint64_t aperture_end_pulse_id{0U};        /**< 孔径结束脉冲 ID */
  double weight{1.0};                             /**< 该观测的加权权重 */
  bool image_is_normalized{false};                /**< 图像是否已单位能量归一化 */
};

/**
 * @brief 单条定标执行请求（请求 ID + 成像路径 + 观测）。
 */
struct CalibrationExecutionRequest {
  std::string request_id{};                                  /**< 请求标识 */
  CalibrationImagePath image_path{CalibrationImagePath::kRda}; /**< 期望的成像路径 */
  CalibrationObservationRequest observation{};               /**< 观测请求 */
};

/**
 * @brief 单条定标残差结果。
 */
struct CalibrationExecutionResidual {
  std::string request_id{};       /**< 关联的请求 ID */
  std::string observation_id{};   /**< 关联的观测 ID */
  double residual_error_db{0.0};  /**< 残差误差（dB） */
};

/**
 * @brief 由图像实际像素构造的定标观测（含估算图像功率）。
 */
struct CalibrationObservation {
  std::string observation_id{};                  /**< 观测标识 */
  double known_rcs_m2{0.0};                       /**< 已知目标 RCS（m²） */
  double image_power{0.0};                        /**< 从图像抽取的功率 */
  double slant_range_m{0.0};                      /**< 目标斜距（m） */
  std::size_t image_row{0U};                      /**< 图像行索引 */
  std::size_t image_col{0U};                      /**< 图像列索引 */
  std::uint64_t aperture_start_pulse_id{0U};      /**< 孔径起始脉冲 ID */
  std::uint64_t aperture_end_pulse_id{0U};        /**< 孔径结束脉冲 ID */
  double weight{1.0};                             /**< 加权权重 */
};

/**
 * @brief 归一化后的定标样本（RCS / 功率 / 斜距 / 权重）。
 */
struct CalibrationSample {
  double known_rcs_m2{0.0};  /**< 已知目标 RCS（m²） */
  double image_power{0.0};   /**< 图像功率 */
  double slant_range_m{0.0}; /**< 目标斜距（m） */
  double weight{1.0};        /**< 加权权重 */
};

/**
 * @brief 辐射定标结果（标定因子与残差）。
 */
struct RadiometricCalibration {
  bool valid{false};                         /**< 定标是否成功 */
  double image_calibration_factor{0.0};      /**< 图像定标因子 */
  double weight_sum{0.0};                    /**< 参与定标的权重之和 */
  std::vector<double> residual_error_db{};   /**< 各样本残差（dB） */
};

/**
 * @brief 定标执行聚合结果。
 */
struct CalibrationExecutionResult {
  bool valid{false};                                           /**< 是否成功 */
  CalibrationExecutionFailure failure{CalibrationExecutionFailure::kNone}; /**< 失败原因 */
  std::size_t request_count{0U};                               /**< 处理的请求数 */
  RadiometricCalibration calibration{};                        /**< 定标结果 */
  std::vector<CalibrationExecutionResidual> residuals{};       /**< 各请求残差 */
};

/**
 * @brief 用单个样本计算辐射定标因子。
 * @param[in] sample 定标样本。
 * @param[out] calibration 定标结果。
 * @return 成功返回 true，失败返回 false。
 */
bool CalibrateSingle(const CalibrationSample& sample, RadiometricCalibration* calibration);

/**
 * @brief 用多个样本加权最小二乘计算辐射定标因子。
 * @param[in] samples 定标样本列表。
 * @param[out] calibration 定标结果（含各样本残差）。
 * @return 成功返回 true，失败返回 false。
 */
bool CalibrateMultiple(const std::vector<CalibrationSample>& samples,
                       RadiometricCalibration* calibration);

/**
 * @brief 由未归一化图像与请求构造定标观测（抽取图像功率）。
 * @param[in] request 观测请求（含目标位置与孔径区间）。
 * @param[in] unnormalized_image 未做单位能量归一化的复图像。
 * @param[out] observation 构造出的观测。
 * @return 成功返回 true，失败返回 false。
 */
bool BuildCalibrationObservation(const CalibrationObservationRequest& request,
                                 const signal::ComplexMatrix& unnormalized_image,
                                 CalibrationObservation* observation);

/**
 * @brief 将观测列表转换为归一化定标样本列表。
 * @param[in] observations 观测列表。
 * @param[out] samples 输出样本列表。
 * @return 成功返回 true，失败返回 false。
 */
bool ConvertObservationsToSamples(const std::vector<CalibrationObservation>& observations,
                                  std::vector<CalibrationSample>* samples);

/**
 * @brief 批量执行定标请求并返回聚合结果。
 * @param[in] actual_image_path 实际图像对应的成像路径。
 * @param[in] unnormalized_image 未归一化复图像。
 * @param[in] requests 定标请求列表。
 * @param[out] result 聚合定标结果（含残差）。
 * @return 成功返回 true，失败返回 false，失败原因写入 result.failure。
 */
bool ExecuteCalibrationRequests(CalibrationImagePath actual_image_path,
                                const signal::ComplexMatrix& unnormalized_image,
                                const std::vector<CalibrationExecutionRequest>& requests,
                                CalibrationExecutionResult* result);

/**
 * @brief 由图像功率反算测量 RCS。
 * @param[in] image_power 图像功率。
 * @param[in] slant_range_m 目标斜距（m）。
 * @param[in] calibration 定标结果。
 * @param[out] measured_rcs_m2 反算得到的测量 RCS（m²）。
 * @return 成功返回 true，失败返回 false。
 */
bool InvertRcs(double image_power, double slant_range_m,
               const RadiometricCalibration& calibration, double* measured_rcs_m2);

/**
 * @brief 计算测量 RCS 与理论 RCS 的辐射误差（dB）。
 * @param[in] measured_rcs_m2 测量 RCS（m²）。
 * @param[in] theoretical_rcs_m2 理论 RCS（m²）。
 * @param[out] error_db 辐射误差（dB）。
 * @return 成功返回 true，失败返回 false。
 */
bool EvaluateRadiometricErrorDb(double measured_rcs_m2, double theoretical_rcs_m2,
                                double* error_db);

}  // namespace calibration
}  // namespace sar

#endif  // ONEQ_SRC_SAR_CALIBRATION_SAR_RADIOMETRIC_CALIBRATION_H_
