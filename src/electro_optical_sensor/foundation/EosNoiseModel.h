/**
 * @file EosNoiseModel.h
 * @brief 定义背景噪声统计模型接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_NOISE_MODEL_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_NOISE_MODEL_H_

namespace electro_optical_sensor {
namespace foundation {
namespace noise {

/**
 * @brief BackgroundNoiseModelInputs 描述背景噪声统计输入。
 */
struct BackgroundNoiseModelInputs {
  float background_flux_w{0.0f};               /**< 背景通量（单位：W） */
  float electrical_bandwidth_hz{1000.0f};      /**< 等效电带宽（单位：Hz） */
  float integration_time_sec{1.0f / 30.0f};    /**< 积分时间（单位：s） */
  float detector_area_cm2{0.25f};              /**< 探测器面积（单位：cm^2） */
  float cloud_coverage_ratio{0.2f};            /**< 云量，范围 [0, 1] */
  float scene_complexity_factor{1.0f};         /**< 场景复杂度系数（>= 1） */
  float photon_noise_enhancement_factor{1.0f}; /**< 光子噪声放大系数（>= 1） */
};

/**
 * @brief BackgroundNoiseStatistics 描述背景噪声统计结果。
 */
struct BackgroundNoiseStatistics {
  float mean_noise_power_w{0.0f};        /**< 背景均值噪声功率（单位：W） */
  float sigma_noise_power_w{0.0f};       /**< 背景噪声标准差（单位：W） */
  float equivalent_noise_power_w{0.0f};  /**< 等效背景噪声功率（单位：W） */
  float suppression_weight{0.05f};       /**< 背景抑制权重，范围 [0.05, 0.60] */
};

/**
 * @brief 计算背景噪声统计结果。
 * @param[in] inputs 背景噪声统计输入。
 * @return 背景噪声统计结果。
 */
BackgroundNoiseStatistics ComputeBackgroundNoiseStatistics(
    const BackgroundNoiseModelInputs& inputs);

/**
 * @brief 基于背景噪声统计对接收信号做保守扣减。
 * @param[in] received_power_w 接收功率（单位：W）。
 * @param[in] background_flux_w 背景通量（单位：W）。
 * @param[in] stats 背景噪声统计结果。
 * @return 扣减后的有效信号功率（单位：W）。
 */
float ComputeEffectiveSignalPowerW(float received_power_w, float background_flux_w,
                                            const BackgroundNoiseStatistics& stats);

}  // namespace noise
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_NOISE_MODEL_H_
