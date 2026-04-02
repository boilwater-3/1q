/**
 * @file EosRadiativeTransfer.h
 * @brief 定义 EOS 辐射传输可插拔模型接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_RADIATIVE_TRANSFER_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_RADIATIVE_TRANSFER_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace foundation {
namespace radiative_transfer {

/**
 * @brief RadiativeTransferModel 描述路径辐射传输模型类型。
 */
enum class ONEQ_API RadiativeTransferModel {
  kDerivedBeerLambert = 0,  /**< 派生 Beer-Lambert（默认，兼容现有逻辑） */
  kHumidityWeighted,        /**< 云湿主导模型 */
  kAdaptivePathRadiance     /**< 含路径辐射惩罚的自适应模型 */
};

/**
 * @brief RadiativeTransferInputs 描述辐射传输评估输入。
 */
struct ONEQ_API RadiativeTransferInputs {
  RadiativeTransferModel model{RadiativeTransferModel::kDerivedBeerLambert};
  float base_transmittance{0.85f};        /**< 基础大气透明度，范围 [0, 1] */
  float cloud_coverage_ratio{0.2f};       /**< 云量，范围 [0, 1] */
  float path_length_m{1000.0f};           /**< 路径长度（单位：m） */
  float aerosol_density_factor{1.0f};     /**< 气溶胶密度系数（>= 1） */
  float turbulence_factor{1.0f};          /**< 湍流强度系数（>= 1） */
};

/**
 * @brief RadiativeTransferResult 描述辐射传输评估结果。
 */
struct ONEQ_API RadiativeTransferResult {
  float transmittance{1.0f};                  /**< 路径透过率，范围 [0, 1] */
  float total_extinction_coeff_per_m{0.0f};   /**< 总消光系数（单位：1/m） */
  float path_radiance_penalty_scale{1.0f};    /**< 路径辐射惩罚系数（>= 1） */
};

/**
 * @brief 评估路径辐射传输结果。
 * @param[in] inputs 辐射传输输入。
 * @return 辐射传输评估结果。
 */
ONEQ_API RadiativeTransferResult EvaluateRadiativeTransfer(
    const RadiativeTransferInputs& inputs);

}  // namespace radiative_transfer
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_RADIATIVE_TRANSFER_H_
