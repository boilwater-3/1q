/**
 * @file RadarTraceSession.h
 * @brief 为机载雷达提供独立的中间层记录包装器。
 */

#ifndef AIRBORNE_RADAR_TOOLS_RADAR_TRACE_SESSION_H_
#define AIRBORNE_RADAR_TOOLS_RADAR_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarTraceSessionOptions 描述记录包装器配置。
 */
struct ONEQ_API RadarTraceSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true}; /**< 构造时是否记录配置 */

  RadarTraceSessionOptions() = default;
  RadarTraceSessionOptions(std::shared_ptr<oneq::replay::ReplayTraceWriter> writer,
                           bool trace_config)
      : replay_writer(std::move(writer)), trace_config_on_construct(trace_config) {}
};

/**
 * @brief RadarTraceSession 作为 RadarSession 的独立中间层记录包装器。
 */
class ONEQ_API RadarTraceSession {
 public:
  explicit RadarTraceSession(const RadarSessionConfig& config = {},
                             RadarTraceSessionOptions options = {});

  session::TrackOutputFrame Step(const RadarCycleInput& input);
  RadarCycleResult StepWithResult(const RadarCycleInput& input);

  void ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch);

  const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const;
  bool HasLatestControlProfile() const;
  const extension::control::RadarControlProfile& GetLatestControlProfile() const;
  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  RadarSession& session();
  const RadarSession& session() const;

 private:
  RadarSession session_;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer_;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_TOOLS_RADAR_TRACE_SESSION_H_
