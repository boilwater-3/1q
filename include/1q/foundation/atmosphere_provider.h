/**
 * @file atmosphere_provider.h
 * @brief 定义大气模型抽象接口。
 */

#ifndef ONEQ_FOUNDATION_ATMOSPHERE_PROVIDER_H_
#define ONEQ_FOUNDATION_ATMOSPHERE_PROVIDER_H_

#include "1q/foundation/atmosphere_state.h"

namespace oneq {
namespace foundation {

/**
 * @brief 大气模型抽象接口。
 */
class ONEQ_API IAtmosphereProvider {
 public:
  virtual ~IAtmosphereProvider() = default;

  /**
   * @brief 查询指定几何高度处的大气状态。
   * @param altitude_m 几何高度（单位：m，ASL）
   * @return 大气状态
   */
  virtual AtmosphericState GetState(float altitude_m) const = 0;

  /**
   * @brief 查询海平面大气状态。
   * @return 海平面大气状态
   */
  virtual AtmosphericState GetSeaLevelState() const = 0;
};

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_ATMOSPHERE_PROVIDER_H_
