/**
 * @file SarAntenna.h
 * @brief SAR 天线方向图、增益与合成孔径时间计算。
 */

#ifndef ONEQ_SRC_SAR_GEOMETRY_SAR_ANTENNA_H_
#define ONEQ_SRC_SAR_GEOMETRY_SAR_ANTENNA_H_

namespace sar {
namespace geometry {

/**
 * @brief 天线物理参数。
 */
struct AntennaParams {
  double length_m{0.0};              ///< 方位向天线长度
  double width_m{0.0};               ///< 距离向天线宽度
  double peak_gain_linear{1.0};      ///< 峰值增益(线性)
  double beam_width_azimuth_rad{0.0}; ///< 方位波束宽度(rad)
  double beam_width_range_rad{0.0};   ///< 距离波束宽度(rad)
  double boresight_azimuth_rad{0.0};  ///< 方位角指向(rad)
};

/**
 * @brief 天线增益 G = 4π·A_eff/λ², 有效面积 A_eff = peak_gain·λ²/(4π)·(L·W)/(peak_aperture)。
 *        简化: G(λ) = 4π·length·width / λ²。
 */
double AntennaGain(const AntennaParams& antenna, double wavelength_m);

/**
 * @brief 合成孔径雷达方位方向图 sinc² 近似。
 *        pattern(θ) = sinc²(π·L·sin(θ)/λ), 其中 θ 为离 boresight 角。
 */
double AzimuthPattern(const AntennaParams& antenna, double wavelength_m,
                      double off_boresight_rad);

/**
 * @brief 通用 sinc² 天线方向图: pattern(θ) = sinc²(π·length·sin(θ)/λ)。
 */
double SincPattern(double length_m, double wavelength_m, double off_boresight_rad);

/**
 * @brief 合成孔径时间: T_synth ≈ R0·θ_bw / v。
 *        θ_bw = λ / length_m (方位波束宽度)。
 */
double SyntheticApertureTime(const AntennaParams& antenna, double slant_range_m,
                             double platform_velocity_mps);

/**
 * @brief 方位分辨率。
 *        合成孔径模式: ρ_az = length_m / 2。
 *        实孔径模式: ρ_az = slant_range_m · λ / length_m。
 */
double AntennaResolution(const AntennaParams& antenna, double slant_range_m,
                         double wavelength_m, bool synthetic_aperture);

}  // namespace geometry
}  // namespace sar

#endif  // ONEQ_SRC_SAR_GEOMETRY_SAR_ANTENNA_H_
