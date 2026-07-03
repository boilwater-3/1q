/**
 * @file TraceSink.h
 * @brief 定义用于会话中间层记录的通用 sink 接口与文件实现。
 */

#ifndef ONEQ_TRACE_TRACE_SINK_H_
#define ONEQ_TRACE_TRACE_SINK_H_

#include <fstream>
#include <mutex>
#include <string>

#include "1q/api.hpp"

namespace oneq {
namespace trace {

/**
 * @brief TraceSink 定义结构化记录写入接口。
 *
 * @note TraceSink 面向调试与观测日志，不是 replay 输入格式。若需要生成可被
 *       `ReplayXxxTrace()` 回放的目录，请使用 `oneq::replay::ReplayTraceWriter`
 *       并传入对应 `*TraceSessionOptions::replay_writer`。
 */
class ONEQ_API TraceSink {
 public:
  virtual ~TraceSink();

  /**
   * @brief 写入一条记录。
   * @param[in] module 模块标识。
   * @param[in] phase 记录阶段，例如 config/input/output。
   * @param[in] payload_json 已构造好的 JSON 对象文本。
   */
  virtual void Record(const std::string& module, const std::string& phase,
                      const std::string& payload_json) = 0;
};

/**
 * @brief FlatbufferFileTraceSink 将记录以 FlatBuffers(FlexBuffers) 二进制帧写入文件。
 *
 * 约定：
 *   - 每条记录按 `uint32_le length + payload bytes` 顺序写入。
 *   - payload 为一条 FlexBuffers map，键包括 timestamp_ms/module/phase/payload_json。
 *   - 跨平台统一使用此实现。
 *   - 该文件不能直接被 `ReplayXxxTrace()` 回放；可回放 trace 使用
 *     `ReplayTraceWriter` 生成 `manifest.json` 与 events JSONL 目录。
 */
class ONEQ_API FlatbufferFileTraceSink final : public TraceSink {
 public:
  explicit FlatbufferFileTraceSink(std::string file_path, bool append = true);

  void Record(const std::string& module, const std::string& phase,
              const std::string& payload_json) override;

  const std::string& file_path() const;
  bool is_open() const;

 private:
  std::string file_path_;
  std::ofstream output_;
  std::mutex mutex_;
};

}  // namespace trace
}  // namespace oneq

#endif  // ONEQ_TRACE_TRACE_SINK_H_
