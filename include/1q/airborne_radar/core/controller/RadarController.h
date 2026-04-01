/**
 * @file RadarController.h
 * @brief 定义核心处理层的雷达调度控制器接口。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_

#include <cstddef>
#include <memory>

#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/controller/IRadarOutputReader.h"
#include "1q/airborne_radar/decision/pipeline/ControlReducerTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace core {
namespace context {
class IRadarContext;
}
}  // namespace core
}  // namespace airborne_radar

namespace airborne_radar {
namespace decision {
namespace pipeline {
class ITacticalDecisionEngine;
}
}  // namespace decision
}  // namespace airborne_radar

namespace airborne_radar {
namespace environment {
class IEnvironmentService;
}
}  // namespace airborne_radar

namespace airborne_radar {
namespace signal {
namespace pipeline {
class ISignalPipeline;
}
}  // namespace signal
}  // namespace airborne_radar

namespace airborne_radar {
namespace core {
namespace controller {

/**
 * @brief RadarController 负责调度信号处理、行为决策与指令下发。
 * @details 采用 PIMPL 模式隐藏实现细节，保证 ABI 稳定性；
 *          内部状态变更不会触发外部项目重编。
 */
class ONEQ_API RadarController : public IRadarOutputReader {
 public:
  ~RadarController() override;

  /**
   * @brief 构造函数，使用默认战术协调器。
   * @param[in] radar_context 雷达上下文引用。
   * @param[in] signal_pipeline 信号处理流水线引用。
   * @param[in] environment_service 环境服务引用。
   */
  RadarController(core::context::IRadarContext& radar_context,
                  signal::pipeline::ISignalPipeline& signal_pipeline,
                  environment::IEnvironmentService& environment_service);

  /**
   * @brief 构造函数，显式注入新的决策引擎。
   * @param[in] radar_context 雷达上下文引用。
   * @param[in] signal_pipeline 信号处理流水线引用。
   * @param[in] decision_engine 战术决策引擎引用。
   * @param[in] environment_service 环境服务引用。
   */
  RadarController(core::context::IRadarContext& radar_context,
                  signal::pipeline::ISignalPipeline& signal_pipeline,
                  decision::pipeline::ITacticalDecisionEngine& decision_engine,
                  environment::IEnvironmentService& environment_service);

  /** @brief 执行一次雷达处理循环 */
  void RunOnce();

  /**
   * @brief 执行指定次数的处理循环（用于仿真或测试）。
   * @param[in] cycles 循环次数。
   */
  void RunCycles(std::size_t cycles);

  /**
   * @brief 更新控制归并器配置。
   * @param[in] config 控制归并器配置。
   */
  void UpdateControlReducerConfig(const decision::pipeline::ControlReducerConfig& config);

  /**
   * @brief 判断当前是否已有可读取的最新轨迹输出帧。
   * @return 若已完成至少一次输出帧装配则返回 true。
   */
  bool HasLatestTrackOutputFrame() const override;

  /**
   * @brief 获取最近一次已缓存的轨迹输出帧。
   * @return 最近一次运行周期产生的轨迹输出帧引用。
   */
  const common::output::TrackOutputFrame& GetLatestTrackOutputFrame() const override;

  /**
   * @brief 获取最近一次输入校验问题列表。
   * @return 最近一次 RunOnce 记录的校验问题。
   */
  const context::ValidationIssueList& GetLastValidationIssues() const;

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
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
