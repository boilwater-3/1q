/**
 * @file SbirsSession.h
 * @brief 定义 SBIRS-inspired 会话门面。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/foundation/SensorContract.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

class ONEQ_API SbirsSession {
 public:
  SbirsSession();
  ~SbirsSession() noexcept;

  SbirsSession(const SbirsSession&) = delete;
  SbirsSession& operator=(const SbirsSession&) = delete;
  SbirsSession(SbirsSession&&) noexcept;
  SbirsSession& operator=(SbirsSession&&) noexcept;

  SbirsOutputFrame Step(const SbirsCycleInput& input);
  SbirsCycleResult StepWithResult(const SbirsCycleInput& input);
  void ApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);
  bool TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);

  static SbirsSession Create(const config::SbirsSessionConfig& config = {});
  static SbirsSession CreateWithValidation(const config::SbirsSessionConfig& config,
                                           config::ValidationIssueList* issues);

 private:
  struct Impl;
  explicit SbirsSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

ONEQ_SENSOR_SESSION_CONTRACT(session::SbirsSession, session::SbirsCycleInput,
                             session::SbirsOutputFrame, session::SbirsCycleResult);

}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_
