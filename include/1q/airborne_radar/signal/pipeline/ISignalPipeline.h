// Copyright 2026. All Rights Reserved.
//
// Description: 定义信号处理层的流水线抽象接口。

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_

#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar::environment {
class IEnvironmentService;
}

namespace airborne_radar::signal::pipeline {

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
};

} // namespace airborne_radar::signal::pipeline

#endif // AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
