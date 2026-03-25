/**
 * @file RadarController.h
 * @brief 定义核心处理层的雷达调度控制器接口。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_

#include <cstddef>
#include <memory>

#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/core/output/IRadarOutputReader.h"
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
class ONEQ_API RadarController : public core::output::IRadarOutputReader {
 public:
  ~RadarController() override;

  /**
   * @brief 构造函数，使用默认战术协调器。
   */
  RadarController(core::context::IRadarContext& radar_context,
                  signal::pipeline::ISignalPipeline& signal_pipeline,
                  environment::IEnvironmentService& environment_service);

  /**
   * @brief 构造函数，显式注入新的决策引擎。
   */
  RadarController(core::context::IRadarContext& radar_context,
                  signal::pipeline::ISignalPipeline& signal_pipeline,
                  decision::pipeline::ITacticalDecisionEngine& decision_engine,
                  environment::IEnvironmentService& environment_service);

  /** @brief 执行一次雷达处理循环 */
  void RunOnce();

  /** @brief 执行指定次数的处理循环（用于仿真或测试） */
  void RunCycles(std::size_t cycles);

  /** @brief 更新控制归并器配置 */
  void UpdateControlReducerConfig(const decision::pipeline::ControlReducerConfig& config);

  /** @brief 判断当前是否已有可读取的最新轨迹输出帧 */
  bool HasLatestTrackOutputFrame() const override;

  /** @brief 获取最近一次已缓存的轨迹输出帧 */
  const common::TrackOutputFrame& GetLatestTrackOutputFrame() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace controller
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
