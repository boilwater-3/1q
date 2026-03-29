/**
 * @file MockISignalPipeline.h
 * @brief ISignalPipeline 的 GMock mock 实现，供单元测试隔离使用。
 */

#ifndef TESTS_MOCK_I_SIGNAL_PIPELINE_H_
#define TESTS_MOCK_I_SIGNAL_PIPELINE_H_

#include <gmock/gmock.h>

#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

class MockISignalPipeline : public ISignalPipeline {
 public:
  MOCK_METHOD(SignalCycleResult, RunCycle,
              (const common::model::TargetFeatureList& input_state,
               const environment::IEnvironmentService& environment),
              (override));
  MOCK_METHOD(void, UpdatePlatformAttitude,
              (const common::config::PlatformAttitudeDeg& platform_attitude_deg), (override));
  MOCK_METHOD(common::config::PlatformAttitudeDeg, GetPlatformAttitude, (), (const, override));
  MOCK_METHOD(void, SetControlProfile, (const common::control::RadarControlProfile& control_profile),
              (override));
  MOCK_METHOD(common::control::RadarControlProfile, GetControlProfile, (), (const, override));
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // TESTS_MOCK_I_SIGNAL_PIPELINE_H_
