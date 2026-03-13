// Copyright 2026. All Rights Reserved.
//
// Description: 定义信号处理层的流水线抽象接口。

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace environment {
class IEnvironmentService;
}
}

namespace airborne_radar {
namespace signal {
namespace pipeline {

/// @brief ISignalPipeline 定义单周期内的探测与跟踪处理流程。
class ISignalPipeline {
public:
	virtual ~ISignalPipeline() = default;

		/// @brief 执行一次信号处理循环。
		/// @param input_state 本周期输入状态。
		/// @param environment 只读环境服务，用于物理建模查询。
		/// @return 返回更新后的雷达状态。
		virtual common::TargetFeatureList
		RunCycle(const common::TargetFeatureList &input_state,
						 const environment::IEnvironmentService &environment) = 0;

		/// @brief 导出最近一次处理周期生成的跟踪量测。
		virtual std::vector<tracking::TrackMeasurement>
		GetLastTrackMeasurements() const = 0;

		/// @brief 设置本周期关联阶段应使用的上一周期轨迹种子。
		virtual void SetAssociationSeeds(
				const std::vector<tracking::AssociationTrackSeed> &seeds) = 0;

		/// @brief 启用或关闭内部关联 history fallback 兼容模式。
		/// @param enabled 为 true 时允许在无 Lifecycle seeds 时使用内部兼容缓存。
		virtual void SetInternalAssociationHistoryFallbackEnabled(bool enabled) = 0;

		/// @brief 清理外部 seeds 状态并恢复 fallback-history 关联模式。
		/// @details 用于确保 external seeds 仅由控制器+Lifecycle 链路注入。
		virtual void ResetAssociationSeedModeToFallbackHistory() = 0;
};

} // namespace pipeline
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
