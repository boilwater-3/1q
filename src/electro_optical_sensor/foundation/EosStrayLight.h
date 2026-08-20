/**
 * @file EosStrayLight.h
 * @brief 定义遮光罩杂散光抑制模型接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_STRAY_LIGHT_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_STRAY_LIGHT_H_

namespace electro_optical_sensor {
namespace foundation {
namespace stray_light {

/**
 * @brief StrayLightFilterInputs 描述杂散光抑制评估输入。
 */
struct StrayLightFilterInputs {
  bool enabled{false};                        /**< 是否启用遮光罩抑制 */
  float target_azimuth_deg{0.0f};            /**< 目标方位角（单位：deg） */
  float target_elevation_deg{0.0f};          /**< 目标仰角（单位：deg） */
  float sun_azimuth_deg{180.0f};             /**< 太阳方位角（单位：deg） */
  float sun_altitude_deg{45.0f};             /**< 太阳高度角（单位：deg） */
  float cloud_coverage_ratio{0.2f};          /**< 云量，范围 [0, 1] */
  float hood_inner_half_angle_deg{12.0f};    /**< 遮光罩内半角（单位：deg） */
  float hood_outer_half_angle_deg{75.0f};    /**< 遮光罩外半角（单位：deg） */
  float min_suppression_ratio{0.20f};        /**< 最低抑制比例，范围 [0, 1] */
  float max_suppression_ratio{0.85f};        /**< 最高抑制比例，范围 [0, 1] */
};

/**
 * @brief StrayLightFilterResult 描述杂散光抑制结果。
 */
struct StrayLightFilterResult {
  float sun_separation_deg{180.0f};          /**< 目标视线与太阳方向夹角（单位：deg） */
  float contamination_ratio{0.0f};           /**< 杂散光污染比例，范围 [0, 1] */
  float suppression_ratio{0.0f};             /**< 抑制比例，范围 [0, 1] */
  float background_penalty_scale{1.0f};      /**< 背景惩罚缩放系数（>= 1） */
  float signal_transmission_scale{1.0f};     /**< 目标信号透过缩放系数，范围 (0, 1] */
};

/**
 * @brief 评估遮光罩杂散光抑制结果。
 * @param[in] inputs 杂散光抑制输入。
 * @return 杂散光抑制结果。
 */
StrayLightFilterResult EvaluateStrayLightFilter(
    const StrayLightFilterInputs& inputs);

}  // namespace stray_light
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_STRAY_LIGHT_H_
