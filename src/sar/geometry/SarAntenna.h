/**
 * @file SarAntenna.h
 * @brief SAR 天线方向图、增益与合成孔径时间计算。
 */

#ifndef ONEQ_SRC_SAR_GEOMETRY_SAR_ANTENNA_H_
#define ONEQ_SRC_SAR_GEOMETRY_SAR_ANTENNA_H_

#include <cmath>

#include "1q/sar/config/SarHardwareConfig.h"

namespace sar {
namespace geometry {

/**
 * @brief 天线方向图模型类型。
 * @note 借鉴 AR 的可配置方向图模型，SAR 默认使用 sinc²（均匀孔径理论解）。
 */
enum class AntennaPatternModel {
  kSincPattern = 0,      ///< sinc² 方向图（均匀孔径理论解，默认）
  kGaussianMainLobe = 1, ///< 高斯主瓣近似
  kParabolicMainLobe = 2,///< 抛物线主瓣近似（dB 域）
  kCosinePower = 3       ///< 余弦幂方向图近似
};

/**
 * @brief 天线物理参数。
 */
struct AntennaParams {
  double length_m{0.0};               ///< 方位向天线长度
  double width_m{0.0};                ///< 距离向天线宽度
  double peak_gain_linear{1.0};       ///< 峰值增益(线性)
  double beam_width_azimuth_rad{0.0}; ///< 方位波束宽度(rad)
  double beam_width_range_rad{0.0};   ///< 距离波束宽度(rad)
  double boresight_azimuth_rad{0.0};  ///< 方位角指向(rad)
  double boresight_elevation_rad{0.0};///< 俯仰角指向(rad)
  AntennaPatternModel pattern_model{AntennaPatternModel::kSincPattern}; ///< 方向图模型
  double max_sidelobe_level_db{-20.0};///< 最大旁瓣电平(dB)，仅非 sinc² 模式生效
  double backlobe_level_db{-35.0};    ///< 后瓣电平(dB)
};

/**
 * @brief 从硬件配置构造天线物理参数。
 *
 * 从 SarHardwareConfig 的物理尺寸和增益推导派生量：
 * - peak_gain_linear = 10^(antenna_gain_db/10)
 * - beam_width_azimuth_rad = λ / length_m
 * - beam_width_range_rad  = λ / width_m
 *
 * @param[in] config SAR 硬件配置。
 * @param[in] wavelength_m 载波波长（单位：m）。
 * @return 填充派生量的 AntennaParams。
 */
inline AntennaParams MakeAntennaParams(const config::SarHardwareConfig& config,
                                       double wavelength_m) {
  AntennaParams params;
  params.length_m = config.antenna_length_m;
  params.width_m = config.antenna_width_m;
  params.peak_gain_linear = std::pow(10.0, config.antenna_gain_db / 10.0);
  params.beam_width_azimuth_rad =
      (params.length_m > 0.0) ? (wavelength_m / params.length_m) : 0.0;
  params.beam_width_range_rad =
      (params.width_m > 0.0) ? (wavelength_m / params.width_m) : 0.0;
  return params;
}

/**
 * @brief 天线增益 G = 4π·A_eff/λ², 有效面积 A_eff = peak_gain·λ²/(4π)·(L·W)/(peak_aperture)。
 *        简化: G(λ) = 4π·length·width / λ²。
 */
double AntennaGain(const AntennaParams& antenna, double wavelength_m);

/**
 * @brief 二维天线方向图评估（方位+俯仰）。
 * @param[in] antenna 天线物理参数（含方向图模型选择和旁瓣/后瓣电平）。
 * @param[in] wavelength_m 载波波长（单位：m）。
 * @param[in] off_boresight_az_rad 目标方位离轴角（单位：rad）。
 * @param[in] off_boresight_el_rad 目标俯仰离轴角（单位：rad），默认 0。
 * @return 线性幅度因子 [0, 1]，含主瓣方向图 + 旁瓣/后瓣限幅。
 * @note 方向图模型由 AntennaParams::pattern_model 选择：
 *       - kSincPattern: sinc²(π·L·sin(θ)/λ) · sinc²(π·W·sin(φ)/λ)
 *       - kGaussianMainLobe: exp(-ln2·(2θ/θ_bw)²)
 *       - kParabolicMainLobe: 10^(-3·(2θ/θ_bw)²/20)
 *       - kCosinePower: cos^k(θ)
 *       主瓣外使用 max_sidelobe_level_db，后瓣(>90°)使用 backlobe_level_db。
 */
double AntennaPattern(const AntennaParams& antenna, double wavelength_m,
                      double off_boresight_az_rad, double off_boresight_el_rad = 0.0);

/**
 * @brief 合成孔径雷达方位方向图（仅方位向，兼容旧接口）。
 *        内部调用 AntennaPattern(antenna, wavelength_m, off_boresight_rad, 0.0)。
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
