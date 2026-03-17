// Copyright 2026. All Rights Reserved.
//
// Description: 定义环境建模层对外暴露的只读服务接口。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_

namespace airborne_radar {
namespace environment {

/// @brief EnvironmentSnapshot 用于封装单个处理周期内的环境快照。
struct EnvironmentSnapshot {
	/// @brief 传播损耗（单位：dB）。
	float propagation_loss_db{0.0f};
	/// @brief 杂波功率估计（单位：dB）。
	float clutter_power_db{0.0f};
	/// @brief 干扰功率估计（单位：dB）。
	float jammer_power_db{0.0f};
	/// @brief 干扰与当前工作频率的重叠度，范围 [0, 1]。
	float jammer_frequency_overlap_ratio{0.0f};
	/// @brief 干扰对当前 PRF 锁定的风险度，范围 [0, 1]。
	float jammer_prf_lock_risk{0.0f};
	/// @brief 干扰是否主要经由旁瓣进入。
	bool jammer_in_sidelobe{false};
	/// @brief 是否检测到干扰。
	bool jamming_detected{false};
};

/// @brief IEnvironmentService 为信号处理与决策层提供只读环境查询接口。
class IEnvironmentService {
public:
	virtual ~IEnvironmentService() = default;

	/// @brief 采样并返回当前处理周期的环境条件。
	virtual EnvironmentSnapshot SampleEnvironment() const = 0;
};

} // namespace environment
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
