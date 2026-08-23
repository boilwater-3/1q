/**
 * @file EcmRecordingSession.h
 * @brief 定义 ECM Replay 记录包装器。
 */

#ifndef ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_RECORDING_SESSION_H_
#define ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_RECORDING_SESSION_H_

#include <memory>

#include "1q/electronic_countermeasure/EcmSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
}  // namespace oneq

namespace electronic_countermeasure {
namespace session {

/** @brief ECM Replay 记录端配置。 */
struct ONEQ_API EcmRecordingSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool record_config_on_construct{true};
};

/** @brief 包装 EcmSession 并向 ReplayTraceWriter 记录配置、补丁、周期输入和周期结果。 */
class ONEQ_API EcmRecordingSession {
 public:
  explicit EcmRecordingSession(config::EcmSessionConfig config = {},
                           EcmRecordingSessionOptions options = {});
  ~EcmRecordingSession();
  EcmRecordingSession(const EcmRecordingSession&) = delete;
  EcmRecordingSession& operator=(const EcmRecordingSession&) = delete;
  EcmRecordingSession(EcmRecordingSession&&) noexcept;
  EcmRecordingSession& operator=(EcmRecordingSession&&) noexcept;

  EcmCycleResult StepWithResult(const EcmCycleInput& input);
  EcmRuntimeConfigApplyResult ApplyRuntimeConfig(
      const config::EcmRuntimeConfigPatch& patch);
  EcmSession& session();
  const EcmSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_RECORDING_SESSION_H_
