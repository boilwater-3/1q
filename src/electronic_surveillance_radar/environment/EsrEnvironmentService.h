/**
 * @file EsrEnvironmentService.h
 * @brief 定义 ESR 环境服务默认实现。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_ENVIRONMENT_ESR_ENVIRONMENT_SERVICE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_ENVIRONMENT_ESR_ENVIRONMENT_SERVICE_H_

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrEnvironmentService 提供周期冻结与环境快照采样能力。
 */
class EsrEnvironmentService final : public IEsrEnvironmentService {
 public:
  /**
   * @brief 使用模型配置初始化环境服务。
   * @param[in] config 环境模型配置。
   */
  explicit EsrEnvironmentService(EsrEnvironmentModelConfig config = {});

  /**
   * @brief 冻结当前周期环境输入。
   * @param[in] cycle_context 单周期上下文。
   */
  void BeginCycle(const EsrEnvironmentCycleContext& cycle_context) override;

  /**
   * @brief 采样当前冻结快照。
   * @return 冻结的环境快照。
   */
  EsrEnvironmentSnapshot SampleEnvironment() const override;

  /**
   * @brief 更新环境模型配置。
   * @param[in] config 新的内部模型配置。
   */
  void UpdateModelConfig(EsrEnvironmentModelConfig config) override;

 private:
  EsrEnvironmentModelConfig config_{};
  EsrEnvironmentSnapshot frozen_snapshot_{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_ENVIRONMENT_ESR_ENVIRONMENT_SERVICE_H_
