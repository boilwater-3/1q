/**
 * @file EosTraceSession.h
 * @brief 为光电传感器模块提供独立的中间层记录包装器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_TOOLS_EOS_TRACE_SESSION_H_
#define ELECTRO_OPTICAL_SENSOR_TOOLS_EOS_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosTraceSessionOptions 描述记录包装器配置。
 */
struct ONEQ_API EosTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{}; /**< 记录输出 sink */
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true};                   /**< 构造时是否记录配置 */

  EosTraceSessionOptions() = default;
  EosTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink,
                         bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  EosTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink,
                         bool trace_config,
                         std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief EosTraceSession 作为 EosSession 的独立中间层记录包装器。
 */
class ONEQ_API EosTraceSession {
 public:
  explicit EosTraceSession(EosSessionConfig config = {},
                           EosTraceSessionOptions options = {});

  /**
   * @brief 执行单周期并返回输出帧（输出便捷入口）。
   * @note 若需要区分本周期是否执行或是否复用上一输出，请使用
   *       `StepWithResult()` 读取结构化状态字段。
   */
  output::EosOutputFrame Step(const EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @note 返回值包含 `executed_this_cycle` / `reused_previous_output` 等状态语义。
   */
  model::EosCycleResult StepWithResult(const EosCycleInput& input);
  void ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch);

  EosSession& session();
  const EosSession& session() const;

 private:
  void Record(const std::string& phase, const std::string& payload_json) const;
  void RecordReplay(const std::string& event_type, const std::string& payload_type,
                    const std::string& payload_json) const;
  void RecordReplay(const std::string& event_type, const std::string& payload_type,
                    const std::string& payload_json, std::uint32_t cycle_index) const;

  EosSession session_;
  std::shared_ptr<oneq::trace::TraceSink> sink_;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer_;
};

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_TOOLS_EOS_TRACE_SESSION_H_
