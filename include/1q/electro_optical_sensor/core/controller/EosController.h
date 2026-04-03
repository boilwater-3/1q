/**
 * @file EosController.h
 * @brief 定义光学传感器核心调度控制器接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_CONTROLLER_EOS_CONTROLLER_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_CONTROLLER_EOS_CONTROLLER_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"

namespace electro_optical_sensor {
namespace pipeline {
class IEosPipeline;
}
namespace core {
namespace controller {

/**
 * @brief EosController 负责调度输入校验、核心管线执行与输出缓存。
 */
class ONEQ_API EosController {
 public:
  /**
   * @brief 构造光学传感器控制器。
   * @param[in] pipeline 核心管线接口实现。
   */
  explicit EosController(pipeline::IEosPipeline& pipeline);
  ~EosController();

  EosController(const EosController&) = delete;
  EosController& operator=(const EosController&) = delete;

  /**
   * @brief 执行一次光学传感器处理周期。
   * @param[in] input 当前周期输入。
   */
  void RunOnce(const context::EosCycleInput& input);

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
  const context::EosValidationIssueList& GetLastValidationIssues() const;

  /**
   * @brief 判断最近一次输入校验是否存在 error 级问题。
   * @return 若存在 error 级问题则返回 true。
   */
  bool HasValidationError() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace controller
}  // namespace core
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_CONTROLLER_EOS_CONTROLLER_H_
