// Copyright 2026. All Rights Reserved.
//
// Description: 定义核心处理层的雷达调度控制器接口。

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
namespace signal {
namespace tracking {
class ITrackLifecycleManager;
}
}
}

namespace airborne_radar {
namespace core {
namespace controller {

/// @brief RadarController 负责调度信号处理、行为决策与指令下发。
class RadarController : public core::output::IRadarOutputReader {
public:
	~RadarController();

	/// @brief 构造函数，使用默认战术协调器。
	RadarController(
			core::context::IRadarContext &radar_context,
			signal::pipeline::ISignalPipeline &signal_pipeline,
			environment::IEnvironmentService &environment_service);

	/// @brief 构造函数，显式注入新的决策引擎。
	RadarController(
			core::context::IRadarContext &radar_context,
			signal::pipeline::ISignalPipeline &signal_pipeline,
			decision::pipeline::ITacticalDecisionEngine &decision_engine,
			environment::IEnvironmentService &environment_service);

	/// @brief 执行一次雷达处理循环。
	void RunOnce();

	/// @brief 执行指定次数的处理循环（用于仿真或测试）。
	void RunCycles(std::size_t cycles);

	/// @brief 运行时绑定轨迹生命周期管理器。
	/// @param lifecycle_manager 生命周期管理器，可为 nullptr。
	void SetTrackLifecycleManager(
			signal::tracking::ITrackLifecycleManager *lifecycle_manager);

	/// @brief 更新控制归并器配置。
	/// @param config 新的 reducer 配置。
	void UpdateControlReducerConfig(
			const decision::pipeline::ControlReducerConfig &config);

	/// @brief 判断是否已有可读取的最新轨迹输出帧。
	/// @return 若已完成至少一次输出帧装配则返回 true。
	bool HasLatestTrackOutputFrame() const override;

	/// @brief 获取最近一次已缓存的轨迹输出帧。
	/// @return 最近一次运行周期产生的中性轨迹输出帧。
	const common::TrackOutputFrame &GetLatestTrackOutputFrame() const override;

private:
	/// @brief 若尚未绑定 Lifecycle，则尝试通过 Pipeline 自动装配。
	void EnsureAutoLifecycleManager();

	/// @brief 将控制意图映射为外围命令并提交到雷达上下文。
	void ExecuteCommands(
			const std::vector<common::ControlDirective> &directives);

	/// @brief 雷达上下文抽象，提供输入状态与命令下发接口。
	core::context::IRadarContext &radar_context_;

	/// @brief 信号处理流水线抽象。
	signal::pipeline::ISignalPipeline &signal_pipeline_;

	/// @brief 决策引擎抽象。
	decision::pipeline::ITacticalDecisionEngine *decision_engine_{nullptr};

	/// @brief 若由 Controller 自持有，则保存默认决策引擎实例。
	std::unique_ptr<decision::pipeline::ITacticalDecisionEngine> owned_decision_engine_;

	/// @brief 环境建模服务抽象。
	environment::IEnvironmentService &environment_service_;

	/// @brief 轨迹生命周期管理器（可选注入）。
	signal::tracking::ITrackLifecycleManager *track_lifecycle_manager_{nullptr};

	/// @brief Controller 自动装配并托管的生命周期管理器实例。
	std::unique_ptr<signal::tracking::ITrackLifecycleManager>
			auto_track_lifecycle_manager_;

	/// @brief 当前持有的下一周期控制真值。
	common::RadarControlProfile *control_profile_;

	/// @brief 控制真值存储。
	std::unique_ptr<common::RadarControlProfile> owned_control_profile_;

	/// @brief 战术跨周期内存。
	std::unique_ptr<decision::pipeline::TacticalStateStore> tactical_state_store_;

	/// @brief 控制归并器。
	std::unique_ptr<decision::pipeline::ControlReducer> control_reducer_;

	/// @brief 数据输出管理服务，负责装配中性输出帧与决策输入帧。
	std::unique_ptr<core::output::IDataOutputManager> output_manager_;

	/// @brief 最近一次已缓存的中性轨迹输出帧。
	common::TrackOutputFrame latest_track_output_frame_{};

	/// @brief 是否已经缓存过至少一次有效输出帧。
	bool has_latest_track_output_frame_{false};

	/// @brief 当前处理周期号。
	std::uint32_t cycle_index_{1};

	/// @brief 当前探测批号。
	std::uint64_t batch_id_{1};
};

} // namespace controller
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
