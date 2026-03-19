/**
 * @file RadarController.h
 * @brief 定义核心处理层的雷达调度控制器接口。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/core/output/IRadarOutputReader.h"
#include "1q/airborne_radar/decision/pipeline/ControlReducerTypes.h"

namespace airborne_radar {
namespace core {
namespace context {
class IRadarContext;
}
}
}

namespace airborne_radar {
namespace core {
namespace output {
class IDataOutputManager;
}
}
}

namespace airborne_radar {
namespace decision {
namespace pipeline {
class ITacticalDecisionEngine;
class ControlReducer;
struct TacticalStateStore;
}
}
}

namespace airborne_radar {
namespace environment {
class IEnvironmentService;
}
}

namespace airborne_radar {
namespace signal {
namespace pipeline {
class ISignalPipeline;
}
}
}

namespace airborne_radar {
namespace core {
namespace controller {

/**
 * @brief RadarController 负责调度信号处理、行为决策与指令下发。
 */
class RadarController : public core::output::IRadarOutputReader {
 public:
  ~RadarController();

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
  void UpdateControlReducerConfig(
      const decision::pipeline::ControlReducerConfig& config);

  /** @brief 判断当前是否已有可读取的最新轨迹输出帧 */
  bool HasLatestTrackOutputFrame() const override;

  /** @brief 获取最近一次已缓存的轨迹输出帧 */
  const common::TrackOutputFrame& GetLatestTrackOutputFrame() const override;

 private:
  void ExecuteCommands(const std::vector<common::ControlDirective>& directives);

  core::context::IRadarContext& radar_context_;
  signal::pipeline::ISignalPipeline& signal_pipeline_;
  decision::pipeline::ITacticalDecisionEngine* decision_engine_{nullptr};
  std::unique_ptr<decision::pipeline::ITacticalDecisionEngine>
      owned_decision_engine_;
  environment::IEnvironmentService& environment_service_;
  common::RadarControlProfile* control_profile_;
  std::unique_ptr<common::RadarControlProfile> owned_control_profile_;
  std::unique_ptr<decision::pipeline::TacticalStateStore> tactical_state_store_;
  std::unique_ptr<decision::pipeline::ControlReducer> control_reducer_;
  std::unique_ptr<core::output::IDataOutputManager> output_manager_;
  common::TrackOutputFrame latest_track_output_frame_{};
  bool has_latest_track_output_frame_{false};
  std::uint32_t cycle_index_{1};
  std::uint64_t batch_id_{1};
};

} // namespace controller
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
