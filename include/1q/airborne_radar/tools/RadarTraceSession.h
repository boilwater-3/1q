/**
 * @file RadarTraceSession.h
 * @brief 为机载雷达提供独立的中间层记录包装器。
 */

#ifndef AIRBORNE_RADAR_TOOLS_RADAR_TRACE_SESSION_H_
#define AIRBORNE_RADAR_TOOLS_RADAR_TRACE_SESSION_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/common/trace/TraceSink.h"

namespace airborne_radar {
namespace tools {

/**
 * @brief RadarTraceSessionOptions 描述记录包装器配置。
 */
struct ONEQ_API RadarTraceSessionOptions {
  std::shared_ptr<oneq::common::trace::TraceSink> sink{}; /**< 记录输出 sink */
  bool trace_config_on_construct{true};                   /**< 构造时是否记录配置 */

  RadarTraceSessionOptions() = default;
  RadarTraceSessionOptions(std::shared_ptr<oneq::common::trace::TraceSink> trace_sink,
                           bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
};

/**
 * @brief RadarTraceSession 作为 RadarSession 的独立中间层记录包装器。
 */
class ONEQ_API RadarTraceSession {
 public:
  explicit RadarTraceSession(const core::session::RadarSessionConfig& config = {},
                             RadarTraceSessionOptions options = {});

  common::output::TrackOutputFrame Step(const core::context::RadarCycleInput& input);
  common::output::TrackOutputFrame Step(const core::context::RadarCycleInput& input,
                                        const environment::EnvironmentSceneState& scene_state);
  core::session::RadarCycleResult StepWithResult(const core::context::RadarCycleInput& input);
  core::session::RadarCycleResult StepWithResult(
      const core::context::RadarCycleInput& input,
      const environment::EnvironmentSceneState& scene_state);

  void UpdateSignalPipelineConfig(const common::config::SignalPipelineConfig& config);
  void UpdateEnvironmentModelConfig(const environment::EnvironmentModelConfig& config);
  void SetJammingDetectionThresholdDb(float threshold_db);
  void ApplyRuntimeConfig(const common::config::RadarRuntimeConfigPatch& patch);

  const std::vector<common::control::RadarCommand>& GetSubmittedCommands() const;
  bool HasLatestControlProfile() const;
  const common::control::RadarControlProfile& GetLatestControlProfile() const;
  signal::pipeline::AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  core::session::RadarSession& session();
  const core::session::RadarSession& session() const;

 private:
  void Record(const std::string& phase, const std::string& payload_json) const;

  core::session::RadarSession session_;
  std::shared_ptr<oneq::common::trace::TraceSink> sink_;
};

}  // namespace tools
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_TOOLS_RADAR_TRACE_SESSION_H_
