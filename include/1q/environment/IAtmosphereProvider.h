/**
 * @file IAtmosphereProvider.h
 * @brief 定义大气模型抽象接口。
 */

#ifndef ONEQ_ENVIRONMENT_I_ATMOSPHERE_PROVIDER_H_
#define ONEQ_ENVIRONMENT_I_ATMOSPHERE_PROVIDER_H_

#include "1q/api.hpp"
#include "1q/environment/AtmosphericState.h"

namespace oneq {
namespace environment {

/**
 * @brief 大气模型抽象接口。
 *
 * 查询指定高度处的大气状态（温度、气压、密度、声速）。
 * 实现类包括 ISA 1976 标准大气和 JSBSim 大气适配器。
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

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_I_ATMOSPHERE_PROVIDER_H_
