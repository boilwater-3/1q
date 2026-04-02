/**
 * @file EosOpticalCharacteristics.h
 * @brief 定义光学系统几何、成像质量与空间分辨率计算函数。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_OPTICAL_CHARACTERISTICS_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_OPTICAL_CHARACTERISTICS_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace foundation {
namespace optics {

/**
 * @brief 计算焦高比（焦距/高度）。
 * @param[in] focal_length_m 焦距（单位：m）。
 * @param[in] platform_altitude_m 平台高度（单位：m）。
 * @return 焦高比。
 */
ONEQ_API float ComputeFocalHeightRatio(float focal_length_m, float platform_altitude_m);

/**
 * @brief 计算瞬时视场角。
 * @param[in] detector_size_m 像面有效尺寸（单位：m）。
 * @param[in] focal_length_m 焦距（单位：m）。
 * @return 视场角（单位：deg）。
 */
ONEQ_API float ComputeInstantaneousFovDeg(float detector_size_m, float focal_length_m);

/**
 * @brief 计算地面扫描幅宽。
 * @param[in] platform_altitude_m 平台高度（单位：m）。
 * @param[in] fov_deg 视场角（单位：deg）。
 * @return 扫描幅宽（单位：m）。
 */
ONEQ_API float ComputeGroundScanWidthM(float platform_altitude_m, float fov_deg);

/**
 * @brief 计算地面投影距离。
 * @param[in] platform_altitude_m 平台高度（单位：m）。
 * @param[in] look_angle_deg 俯视偏角（单位：deg）。
 * @return 地面投影距离（单位：m）。
 */
ONEQ_API float ComputeGroundProjectionDistanceM(float platform_altitude_m, float look_angle_deg);

/**
 * @brief 计算衍射极限角分辨率（Airy 近似）。
 * @param[in] wavelength_um 波长（单位：um）。
 * @param[in] aperture_diameter_m 口径（单位：m）。
 * @return 角分辨率（单位：rad）。
 */
ONEQ_API float ComputeDiffractionLimitedAngularResolutionRad(float wavelength_um,
                                                             float aperture_diameter_m);

/**
 * @brief 计算地面空间分辨率（GSD 近似）。
 * @param[in] range_m 斜距（单位：m）。
 * @param[in] angular_resolution_rad 角分辨率（单位：rad）。
 * @return 地面空间分辨率（单位：m）。
 */
ONEQ_API float ComputeGroundSampleDistanceM(float range_m, float angular_resolution_rad);

/**
 * @brief 计算离焦系数。
 * @param[in] focus_distance_m 对焦距离（单位：m）。
 * @param[in] target_distance_m 目标距离（单位：m）。
 * @return 离焦系数（无量纲）。
 */
ONEQ_API float ComputeDefocusCoefficient(float focus_distance_m, float target_distance_m);

/**
 * @brief 计算弥散圆直径近似值。
 * @param[in] aperture_diameter_m 口径（单位：m）。
 * @param[in] focal_length_m 焦距（单位：m）。
 * @param[in] focus_distance_m 对焦距离（单位：m）。
 * @param[in] target_distance_m 目标距离（单位：m）。
 * @return 弥散圆直径（单位：m）。
 */
ONEQ_API float ComputeCircleOfConfusionDiameterM(float aperture_diameter_m, float focal_length_m,
                                                 float focus_distance_m, float target_distance_m);

/**
 * @brief 计算折射位移近似值。
 * @param[in] range_m 传播距离（单位：m）。
 * @param[in] refractive_index 梯度层等效折射率。
 * @return 折射引起的横向位移（单位：m）。
 */
ONEQ_API float ComputeRefractiveShiftM(float range_m, float refractive_index);

}  // namespace optics
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_OPTICAL_CHARACTERISTICS_H_
