/**
 * @file EosSpatialSpectrum.h
 * @brief 定义空间频率谱可分辨性评估接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_SPATIAL_SPECTRUM_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_SPATIAL_SPECTRUM_H_

namespace electro_optical_sensor {
namespace foundation {
namespace spatial_spectrum {

/**
 * @brief SpatialSpectrumInputs 描述空间频率可分辨性评估输入。
 */
struct SpatialSpectrumInputs {
  float target_characteristic_size_m{1.0f};  /**< 目标等效尺寸（单位：m） */
  float ground_sample_distance_m{0.2f};      /**< 地面采样间隔/GSD（单位：m） */
  float optical_mtf_reference{0.65f};        /**< 光学 MTF 基准，范围 [0, 1] */
  float sampling_efficiency{0.90f};          /**< 采样效率，范围 [0, 1] */
  float scene_contrast_ratio{0.30f};         /**< 场景等效对比度，范围 [0, 1] */
};

/**
 * @brief SpatialSpectrumResult 描述空间频率评估结果。
 */
struct SpatialSpectrumResult {
  float target_frequency_cpm{0.0f};      /**< 目标特征频率（cycles/m） */
  float nyquist_frequency_cpm{0.0f};     /**< 采样奈奎斯特频率（cycles/m） */
  float optical_pass_ratio{0.0f};        /**< 光学通带比例，范围 [0, 1] */
  float resolvability_score{0.0f};       /**< 可分辨性评分，范围 [0, 1] */
  float spectrum_quality_gain{0.0f};     /**< 频谱质量增益，范围 [0.05, 1.0] */
};

/**
 * @brief 评估目标空间频率可分辨性。
 * @param[in] inputs 空间频率评估输入。
 * @return 空间频率评估结果。
 */
SpatialSpectrumResult EvaluateSpatialResolvability(
    const SpatialSpectrumInputs& inputs);

}  // namespace spatial_spectrum
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_SPATIAL_SPECTRUM_H_
