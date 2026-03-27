/**
 * @file MockIEnvironmentService.h
 * @brief IEnvironmentService 的 GMock mock 实现，供单元测试隔离使用。
 */

#ifndef TESTS_MOCK_I_ENVIRONMENT_SERVICE_H_
#define TESTS_MOCK_I_ENVIRONMENT_SERVICE_H_

#include <gmock/gmock.h>

#include "1q/airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {

class MockIEnvironmentService : public IEnvironmentService {
 public:
  MOCK_METHOD(void, BeginCycle, (const EnvironmentCycleContext& cycle_context), (override));
  MOCK_METHOD(EnvironmentSnapshot, SampleEnvironment, (), (const, override));
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // TESTS_MOCK_I_ENVIRONMENT_SERVICE_H_
