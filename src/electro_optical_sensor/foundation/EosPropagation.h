/**
 * @file EosPropagation.h
 * @brief 定义大气传输、接收功率与 SNR 计算函数。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PROPAGATION_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PROPAGATION_H_

namespace electro_optical_sensor {
namespace foundation {
namespace propagation {

/**
 * @brief NepNoiseModelInputs 描述 NEP 细化噪声模型输入。
 */
struct NepNoiseModelInputs {
  float detector_detectivity_cm_sqrt_hz_per_w{1.0e10f}; /**< 探测率 D*（单位：cm*sqrt(Hz)/W） */
  float detector_area_cm2{0.25f};                       /**< 探测器面积（单位：cm^2） */
  float electrical_bandwidth_hz{1000.0f};              /**< 电噪声带宽（单位：Hz） */
  float optical_transmittance{0.85f};                  /**< 光学透过率，范围 [0, 1] */
  float integration_time_sec{1.0f / 30.0f};            /**< 积分时间（单位：s） */
  float system_noise_factor{1.0f};                     /**< 系统噪声放大系数 */
};

/**
 * @brief SnrEvaluationResult 描述 SNR 细化评估输出。
 */
struct SnrEvaluationResult {
  float nep_w{0.0f};         /**< 估算 NEP（单位：W） */
  float snr_linear{0.0f};    /**< 线性 SNR */
  float snr_db{-120.0f};     /**< dB SNR */
};

/**
 * @brief 计算 Beer-Lambert 透过率 `exp(-alpha * L)`。
 * @param[in] attenuation_coeff_per_m 衰减系数（单位：1/m）。
 * @param[in] path_length_m 传播距离（单位：m）。
 * @return 透过率，范围 [0, 1]。
 */
float ComputeBeerLambertTransmittance(float attenuation_coeff_per_m, float path_length_m);

/**
 * @brief 计算合成大气透过率。
 * @param[in] base_extinction_coeff_per_m 基础消光系数（单位：1/m）。
 * @param[in] humidity_absorption_coeff_per_m 湿度吸收系数（单位：1/m）。
 * @param[in] path_length_m 传播距离（单位：m）。
 * @return 透过率，范围 [0, 1]。
 */
float ComputeAtmosphericTransmittance(float base_extinction_coeff_per_m,
                                               float humidity_absorption_coeff_per_m,
                                               float path_length_m);

/**
 * @brief 计算背景通量。
 * @param[in] background_radiance_w_sr_m2 背景辐亮度（单位：W/(sr*m^2)）。
 * @param[in] aperture_area_m2 入瞳面积（单位：m^2）。
 * @param[in] fov_solid_angle_sr 视场立体角（单位：sr）。
 * @param[in] optical_transmittance 光学系统透过率，范围 [0, 1]。
 * @return 背景通量（单位：W）。
 */
float ComputeBackgroundFluxW(float background_radiance_w_sr_m2, float aperture_area_m2,
                                      float fov_solid_angle_sr, float optical_transmittance);

/**
 * @brief 计算焦平面接收功率。
 * @param[in] source_radiance_w_sr_m2 源辐亮度（单位：W/(sr*m^2)）。
 * @param[in] projected_area_m2 目标投影面积（单位：m^2）。
 * @param[in] range_m 目标距离（单位：m）。
 * @param[in] aperture_area_m2 入瞳面积（单位：m^2）。
 * @param[in] atmospheric_transmittance 大气透过率，范围 [0, 1]。
 * @param[in] optical_transmittance 光学系统透过率，范围 [0, 1]。
 * @return 接收功率（单位：W）。
 */
float ComputeReceivedPowerW(float source_radiance_w_sr_m2, float projected_area_m2,
                                     float range_m, float aperture_area_m2,
                                     float atmospheric_transmittance,
                                     float optical_transmittance);

/**
 * @brief 计算线性信噪比。
 * @param[in] received_power_w 接收功率（单位：W）。
 * @param[in] nep_w 噪声等效功率（单位：W）。
 * @return 线性 SNR。
 */
float ComputeSnrLinear(float received_power_w, float nep_w);

/**
 * @brief 计算 dB 信噪比。
 * @param[in] snr_linear 线性 SNR。
 * @return dB SNR。
 */
float ComputeSnrDb(float snr_linear);

/**
 * @brief 基于探测率与带宽估算 NEP。
 * @param[in] inputs NEP 噪声模型输入。
 * @return 估算 NEP（单位：W）。
 */
float ComputeNepW(const NepNoiseModelInputs& inputs);

/**
 * @brief 基于 NEP 细化模型评估 SNR。
 * @param[in] received_power_w 接收功率（单位：W）。
 * @param[in] inputs NEP 噪声模型输入。
 * @return SNR 评估结果。
 */
SnrEvaluationResult EvaluateSnrWithNep(float received_power_w,
                                                const NepNoiseModelInputs& inputs);

}  // namespace propagation
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PROPAGATION_H_
