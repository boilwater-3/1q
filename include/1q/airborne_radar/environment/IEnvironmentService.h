// Copyright 2026. All Rights Reserved.
//
// @file IEnvironmentService.h
// @brief 定义环境建模层对外暴露的只读服务接口。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"

namespace airborne_radar {
namespace environment {

/// @brief IEnvironmentService 为信号处理与决策层提供只读环境查询接口。
class IEnvironmentService {
 public:
  virtual ~IEnvironmentService() = default;

  /// @brief 冻结当前周期环境事实，供后续只读采样复用。
  virtual void BeginCycle(const EnvironmentCycleContext& cycle_context) = 0;

  /// @brief 采样并返回当前处理周期的环境条件。
  virtual EnvironmentSnapshot SampleEnvironment() const = 0;
};

} // namespace environment
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
