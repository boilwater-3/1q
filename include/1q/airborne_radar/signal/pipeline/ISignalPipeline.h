// Copyright 2026. All Rights Reserved.
//
// Description: 定义信号处理层的流水线抽象接口。

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_

#include <memory>

#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
class ITrackLifecycleManager;
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

/// @brief 关联质量观测指标（Pipeline 对外公开版本）。
struct AssociationQualityMetrics {
	/// @brief 进入关联阶段的历史先验轨迹数。
	std::size_t prior_track_count{0};
	/// @brief 本周期探测成功并参与关联的量测数。
	std::size_t detection_count{0};
	/// @brief 命中已有轨迹的关联数。
	std::size_t matched_count{0};
	/// @brief 触发新建轨迹键的量测数。
	std::size_t new_track_count{0};
	/// @brief 未命中任何量测的历史轨迹数。
	std::size_t missed_track_count{0};
	/// @brief 命中率（matched_count / detection_count）。
	float match_rate{0.0f};
	/// @brief 新生率（new_track_count / detection_count）。
	float new_track_rate{0.0f};
	/// @brief 漏失率（missed_track_count / prior_track_count）。
	float missed_track_rate{0.0f};
	/// @brief 命中关联代价均值（仅统计 matches）。
	float mean_match_cost{0.0f};
	/// @brief 命中关联代价 P95（仅统计 matches）。
	float p95_match_cost{0.0f};
	/// @brief 当前周期关联质量对应的主导干扰摘要类型。
	common::JammingSemantic dominant_jamming_semantic{
			common::JammingSemantic::kNone};
	/// @brief 当前周期关联质量对应的残余干扰强度摘要，范围 [0, 1]。
	float jamming_severity{0.0f};
	/// @brief 当前周期的归一化关联压力，范围 [0, 1]，值越大表示越容易受干扰导致关联抖动。
	float association_stress{0.0f};
};

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

		/// @brief 导出最近一次处理周期的关联质量观测指标。
		virtual AssociationQualityMetrics
		GetLastAssociationQualityMetrics() const = 0;

		/// @brief 设置本周期关联阶段应使用的上一周期轨迹种子。
		virtual void SetAssociationSeeds(
				const std::vector<tracking::AssociationTrackSeed> &seeds) = 0;

		/// @brief 清理外部 seeds 状态并恢复无先验（stateless）关联模式。
		/// @details 用于确保 external seeds 仅由控制器+Lifecycle 链路注入。
		virtual void ResetAssociationSeedModeToStateless() = 0;

		/// @brief 按当前 Pipeline 配置创建生命周期管理器（自动装配入口）。
		/// @return 若未启用自动装配则返回空指针；否则返回可直接挂载到 Controller 的生命周期服务。
		virtual std::unique_ptr<tracking::ITrackLifecycleManager>
		CreateAutoLifecycleManager() const = 0;

		/// @brief 更新当前搭载平台姿态。
		/// @param platform_attitude_deg 平台姿态角（单位：度）。
		virtual void UpdatePlatformAttitude(
				const common::PlatformAttitudeDeg &platform_attitude_deg) = 0;

		/// @brief 获取当前搭载平台姿态。
		/// @return 当前缓存的平台姿态角。
		virtual common::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

		/// @brief 设置下一周期生效的控制真值。
		virtual void SetControlProfile(
				const common::RadarControlProfile &control_profile) = 0;

		/// @brief 获取当前缓存的控制真值。
		virtual common::RadarControlProfile GetControlProfile() const = 0;
};

} // namespace pipeline
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
