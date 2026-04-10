/**
 * @file EosController.h
 * @brief 定义光学传感器核心调度控制器接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_CONTROLLER_EOS_CONTROLLER_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_CONTROLLER_EOS_CONTROLLER_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"

namespace electro_optical_sensor {
namespace extension {
class IEosPipeline;
}
namespace extension {

/**
 * @brief EosController 负责调度输入校验、核心管线执行与输出缓存。
 */
class ONEQ_API EosController {
 public:
  /**
   * @brief 构造光学传感器控制器。
   * @param[in] pipeline 核心管线接口实现。
   */
  explicit EosController(extension::IEosPipeline& pipeline);
  ~EosController();

  EosController(const EosController&) = delete;
  EosController& operator=(const EosController&) = delete;

  /**
   * @brief 执行一次光学传感器处理周期。
   * @param[in] input 当前周期输入。
   */
  void RunOnce(const session::EosCycleInput& input);

  /**
   * @brief 判断当前是否有可读取的最新输出帧。
   * @return 若已有输出帧则返回 true。
   */
  bool HasLatestOutputFrame() const;

  /**
   * @brief 获取最新输出帧。
   * @return 最新输出帧。
   */
  const common::EosOutputFrame& GetLatestOutputFrame() const;

  /**
   * @brief 获取最近一次输入校验结果。
   * @return 最近一次输入校验问题列表。
   */
  const session::EosValidationIssueList& GetLastValidationIssues() const;

  /**
   * @brief 判断最近一次输入校验是否存在 error 级问题。
   * @return 若存在 error 级问题则返回 true。
   */
  bool HasValidationError() const;

  /**
   * @brief 最近一次 RunOnce 是否执行了核心 pipeline。
   * @return 若执行了核心 pipeline 则返回 true。
   */
  bool ExecutedLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 是否复用了上一有效输出。
   * @return 若复用了上一有效输出则返回 true。
   */
  bool ReusedPreviousOutputLatestCycle() const;

  /**
   * @brief 获取当前控制器绑定的核心管线实例。
   */
  extension::IEosPipeline& GetPipeline();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension

namespace core {
namespace controller {
using ::electro_optical_sensor::extension::EosController;
}  // namespace controller
}  // namespace core

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_CONTROLLER_EOS_CONTROLLER_H_
