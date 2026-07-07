/**
 * @file JsbsimAtmosphereAdapter.h
 * @brief 将 JSBSim 的 FGAtmosphere 适配为 IAtmosphereProvider。
 *
 * 仅当 JSBSim 可用时编译（条件编译）。
 */

#ifndef ONEQ_ENVIRONMENT_JSBSIM_ATMOSPHERE_ADAPTER_H_
#define ONEQ_ENVIRONMENT_JSBSIM_ATMOSPHERE_ADAPTER_H_

#include "1q/api.hpp"
#include "1q/environment/IAtmosphereProvider.h"

// 前向声明，避免在公开头文件中引入 JSBSim 头文件
namespace JSBSim {
class FGFDMExec;
}  // namespace JSBSim

namespace oneq {
namespace environment {

/**
 * @brief 将 JSBSim 的 FGAtmosphere 适配为 IAtmosphereProvider。
 *
 * 查询 JSBSim 内部的 ISA 1976 大气模型，将 imperial 单位转换为 SI 单位。
 * 不持有 FGFDMExec 所有权，由调用方保证生命周期。
 */
class ONEQ_API JsbsimAtmosphereAdapter : public IAtmosphereProvider {
 public:
  /**
   * @brief 从 JSBSim FDM 执行器构造大气适配器。
   * @param[in] fdm_exec JSBSim FDM 执行器引用（调用方保证生命周期）。
   * @note 适配器按引用持有 fdm_exec，不承担所有权；调用期间须保证其存活。
   */
  explicit JsbsimAtmosphereAdapter(const JSBSim::FGFDMExec& fdm_exec);

  /**
   * @brief 查询指定几何高度处的大气状态（从 JSBSim 大气模型读取并转换为 SI 单位）。
   * @param[in] altitude_m 几何高度（单位：m，ASL）。
   * @return 对应高度的大气状态。
   */
  AtmosphericState GetState(float altitude_m) const override;

  /**
   * @brief 查询海平面大气状态。
   * @return 海平面大气状态。
   */
  AtmosphericState GetSeaLevelState() const override;

 private:
  const JSBSim::FGFDMExec& fdm_exec_;
};

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_JSBSIM_ATMOSPHERE_ADAPTER_H_
