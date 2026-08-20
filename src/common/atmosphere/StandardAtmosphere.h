/**
 * @file StandardAtmosphere.h
 * @brief ISA 1976 标准大气模型实现。
 */

#ifndef COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_
#define COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_

#include "1q/environment/IAtmosphereProvider.h"

namespace oneq {
namespace common {
namespace atmosphere {

/**
 * @brief ISA 1976 标准大气模型（0–86 km）。
 */
class StandardAtmosphere : public environment::IAtmosphereProvider {
 public:
  /**
   * @brief 查询指定几何高度处的 ISA 1976 大气状态。
   * @param[in] altitude_m 几何高度（单位：m，ASL）。
   * @return 对应高度的温度、气压、密度与声速。
   * @note 输入高度被钳位到 ISA 1976 有效范围 [0, 86 km]：负值回退到海平面，
   *       超过 86 km 按上限计算。
   */
  environment::AtmosphericState GetState(float altitude_m) const override;

  /**
   * @brief 查询海平面（0 m）处的大气状态。
   * @return 海平面大气状态，等价于 GetState(0.0f)。
   */
  environment::AtmosphericState GetSeaLevelState() const override;
};

}  // namespace atmosphere
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_
