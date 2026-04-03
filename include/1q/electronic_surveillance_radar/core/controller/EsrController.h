/**
 * @file EsrController.h
 * @brief 定义电子侦察核心调度控制器接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CORE_CONTROLLER_ESR_CONTROLLER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CORE_CONTROLLER_ESR_CONTROLLER_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace environment {
class IEsrEnvironmentService;
}
namespace pipeline {
class IInterceptPipeline;
}
namespace core {
namespace controller {

/**
 * @brief EsrController 负责调度环境冻结、侦察流水线执行与输出缓存。
 */
class ONEQ_API EsrController {
 public:
  /**
   * @brief 构造电子侦察控制器。
   * @param[in] pipeline 侦察流水线接口实现。
   * @param[in] environment_service 环境服务接口实现。
   */
  EsrController(pipeline::IInterceptPipeline& pipeline,
                environment::IEsrEnvironmentService& environment_service);
  ~EsrController();

  EsrController(const EsrController&) = delete;
  EsrController& operator=(const EsrController&) = delete;

  /**
   * @brief 执行一次电子侦察处理周期。
   * @param[in] input 当前周期输入。
   */
  void RunOnce(const context::EsrCycleInput& input);

  /**
   * @brief 判断当前是否有可读取的最新输出帧。
   * @return 若已有输出帧则返回 `true`。
   */
  bool HasLatestOutputFrame() const;

  /**
   * @brief 获取最新输出帧。
   * @return 最新电子侦察输出帧。
   */
  const common::EsrOutputFrame& GetLatestOutputFrame() const;

  /**
   * @brief 获取最近一次输入校验结果。
   * @return 最近一次输入校验问题列表。
   */
  const context::EsrValidationIssueList& GetLastValidationIssues() const;

  /**
   * @brief 获取当前控制器绑定的流水线实例。
   */
  pipeline::IInterceptPipeline& GetPipeline();

  /**
   * @brief 获取当前控制器绑定的环境服务实例。
   */
  environment::IEsrEnvironmentService& GetEnvironmentService();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace controller
}  // namespace core
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CORE_CONTROLLER_ESR_CONTROLLER_H_
