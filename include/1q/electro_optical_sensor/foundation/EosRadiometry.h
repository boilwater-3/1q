/**
 * @file EosRadiometry.h
 * @brief 定义光学传感器基础辐射度量计算函数。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_RADIOMETRY_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_RADIOMETRY_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace foundation {
namespace radiometry {

/**
 * @brief IlluminationCondition 表示可见光照条件。
 */
enum class IlluminationCondition {
  kDay = 0,  /**< 白天 */
  kNight,    /**< 夜间 */
  kTwilight  /**< 晨昏 */
};

/**
 * @brief InfraredRadianceInputs 描述红外辐射亮度计算输入。
 */
struct ONEQ_API InfraredRadianceInputs {
  float wavelength_um{4.0f};          /**< 工作波长（单位：um） */
  float target_temperature_k{300.0f}; /**< 目标等效温度（单位：K） */
  float emissivity{0.9f};             /**< 发射率，范围 [0, 1] */
  float background_temperature_k{290.0f}; /**< 背景温度（单位：K） */
};

/**
 * @brief VisibleRadianceInputs 描述可见光朗伯反射亮度计算输入。
 */
struct ONEQ_API VisibleRadianceInputs {
  float solar_irradiance_w_m2{800.0f}; /**< 地表太阳辐照度（单位：W/m^2） */
  float solar_altitude_deg{45.0f};     /**< 太阳高度角（单位：deg） */
  float atmospheric_transmittance{0.85f}; /**< 大气透明度，范围 [0, 1] */
  float cloud_coverage_ratio{0.2f};    /**< 云量，范围 [0, 1] */
  float reflectance{0.2f};             /**< 漫反射率，范围 [0, 1] */
  float projected_area_m2{1.0f};       /**< 投影面积（单位：m^2） */
  float range_m{1000.0f};              /**< 斜距（单位：m） */
  IlluminationCondition illumination{IlluminationCondition::kDay}; /**< 光照条件 */
};

/**
 * @brief 计算指定波长和温度的黑体光谱辐射亮度。
 * @param[in] wavelength_um 波长（单位：um）。
 * @param[in] temperature_k 温度（单位：K）。
 * @return 光谱辐射亮度，单位 `W/(sr*m^3)`。
 */
ONEQ_API float ComputePlanckRadiance(float wavelength_um, float temperature_k);

/**
 * @brief 计算红外目标辐射亮度（目标发射亮度与背景亮度差）。
 * @param[in] inputs 红外辐射亮度输入。
 * @return 有符号辐射亮度差，单位 `W/(sr*m^3)`。
 */
ONEQ_API float ComputeInfraredRadianceDelta(const InfraredRadianceInputs& inputs);

/**
 * @brief 计算朗伯反射可见光辐射亮度。
 * @param[in] inputs 可见光辐射亮度输入。
 * @return 目标可见光辐射亮度，单位 `W/(sr*m^2)`。
 */
ONEQ_API float ComputeVisibleLambertianRadiance(const VisibleRadianceInputs& inputs);

/**
 * @brief 计算相对对比度 `(target - background) / background`。
 * @param[in] target_radiance 目标辐射亮度。
 * @param[in] background_radiance 背景辐射亮度。
 * @return 相对对比度。
 */
ONEQ_API float ComputeRelativeContrast(float target_radiance, float background_radiance);

}  // namespace radiometry
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_RADIOMETRY_H_
