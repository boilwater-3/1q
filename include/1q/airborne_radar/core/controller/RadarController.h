// Copyright 2026. All Rights Reserved.
//
// Description: 定义核心处理层的雷达调度控制器接口。

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_

#include <cstddef>
#include <cstdint>

namespace airborne_radar {
namespace core {
namespace context {
class IRadarContext;
struct DecisionContext;
}
}
}

namespace airborne_radar {
namespace core {
namespace event {
class IEventBus;
}
}
}

namespace airborne_radar {
namespace decision {
namespace pipeline {
class ITacticalProcessor;
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
class RadarController {
public:
	/// @brief 构造函数，注入核心依赖组件。
	RadarController(
			core::context::IRadarContext &radar_context,
			signal::pipeline::ISignalPipeline &signal_pipeline,
			decision::pipeline::ITacticalProcessor &decision_pipeline,
			environment::IEnvironmentService &environment_service);

	/// @brief 构造函数，注入核心依赖组件与事件总线。
	RadarController(
			core::context::IRadarContext &radar_context,
			signal::pipeline::ISignalPipeline &signal_pipeline,
			decision::pipeline::ITacticalProcessor &decision_pipeline,
			environment::IEnvironmentService &environment_service,
			core::event::IEventBus &event_bus);

	/// @brief 执行一次雷达处理循环。
	void RunOnce();

	/// @brief 执行指定次数的处理循环（用于仿真或测试）。
	void RunCycles(std::size_t cycles);

	/// @brief 运行时绑定事件总线。
	/// @param event_bus 事件总线指针，可为 nullptr。
	void SetEventBus(core::event::IEventBus *event_bus);

	/// @brief 运行时绑定轨迹生命周期管理器。
	/// @param lifecycle_manager 生命周期管理器，可为 nullptr。
	void SetTrackLifecycleManager(
			signal::tracking::ITrackLifecycleManager *lifecycle_manager);

private:
	/// @brief 将决策管线输出的指令批量提交给雷达上下文。
	void ExecuteCommands(const core::context::DecisionContext &context);

	/// @brief 雷达上下文抽象，提供输入状态与命令下发接口。
	core::context::IRadarContext &radar_context_;

	/// @brief 信号处理流水线抽象。
	signal::pipeline::ISignalPipeline &signal_pipeline_;

	/// @brief 决策处理流水线抽象。
	decision::pipeline::ITacticalProcessor &decision_pipeline_;

	/// @brief 环境建模服务抽象。
	environment::IEnvironmentService &environment_service_;

	/// @brief 事件总线（可选注入）。
	core::event::IEventBus *event_bus_{nullptr};

	/// @brief 轨迹生命周期管理器（可选注入）。
	signal::tracking::ITrackLifecycleManager *track_lifecycle_manager_{nullptr};

	/// @brief 当前处理周期号。
	std::uint32_t cycle_index_{1};

	/// @brief 当前探测批号。
	std::uint64_t batch_id_{1};
};

} // namespace controller
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
