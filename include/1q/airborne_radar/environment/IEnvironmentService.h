// Copyright 2026. All Rights Reserved.
//
// @file IEnvironmentService.h
// @brief 定义环境建模层对外暴露的只读服务接口。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_

#include <cstdint>
#include <vector>

namespace airborne_radar {
namespace environment {

/// @brief JammingTechnique 表示干扰技术类型。
enum class JammingTechnique {
  /// @brief 未知或未分类干扰。
  kUnknown = 0,
  /// @brief 压制式/噪声式干扰。
  kNoiseSuppression,
  /// @brief 欺骗式干扰。
  kDeception,
  /// @brief 转发式/重复器式干扰。
  kRepeater
};

/// @brief JammerSourceFact 表示单个干扰源的事实描述。
struct JammerSourceFact {
  /// @brief 干扰技术类型。
  JammingTechnique technique{JammingTechnique::kUnknown};
  /// @brief 干扰功率估计（单位：dB）。
  float power_db{0.0f};
  /// @brief 干扰与信号比估计（单位：dB）。
  float js_db{0.0f};
  /// @brief 干扰与当前工作频率的重叠度，范围 [0, 1]。
  float frequency_overlap_ratio{0.0f};
  /// @brief 干扰对当前 PRF 锁定的风险度，范围 [0, 1]。
  float prf_lock_risk{0.0f};
  /// @brief 干扰来向方位角（单位：deg）。
  float azimuth_deg{0.0f};
  /// @brief 干扰来向俯仰角（单位：deg）。
  float elevation_deg{0.0f};
  /// @brief 干扰角域宽度（单位：deg）。
  float angular_span_deg{0.0f};
  /// @brief 干扰是否主要经由旁瓣进入。
  bool in_sidelobe{false};
  /// @brief 干扰事实置信度，范围 [0, 1]。
  float confidence{1.0f};
};

/// @brief 单周期内可见的干扰源列表。
typedef std::vector<JammerSourceFact> JammerSourceFactList;

/// @brief EnvironmentCycleContext 描述环境层周期冻结上下文。
struct EnvironmentCycleContext {
  /// @brief 当前周期号。
  std::uint32_t cycle_index{0U};
  /// @brief 当前周期步长（单位：s）。
  float dt_sec{0.0f};
};

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
	/// @brief 当前周期可见的多源干扰事实。
	JammerSourceFactList jammer_sources{};
	/// @brief 是否检测到干扰。
	bool jamming_detected{false};
};

/// @brief IEnvironmentService 为信号处理与决策层提供只读环境查询接口。
class IEnvironmentService {
public:
	virtual ~IEnvironmentService() = default;

	/// @brief 冻结当前周期环境事实，供后续只读采样复用。
	virtual void BeginCycle(
	    const EnvironmentCycleContext& cycle_context) = 0;

	/// @brief 采样并返回当前处理周期的环境条件。
	virtual EnvironmentSnapshot SampleEnvironment() const = 0;
};

} // namespace environment
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
