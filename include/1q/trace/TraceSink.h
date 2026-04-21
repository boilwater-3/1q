/**
 * @file TraceSink.h
 * @brief 定义用于会话中间层记录的通用 sink 接口与文件实现。
 */

#ifndef ONEQ_TRACE_TRACE_SINK_H_
#define ONEQ_TRACE_TRACE_SINK_H_

#include <memory>
#include <string>

#include "1q/api.hpp"

namespace oneq {
namespace trace {

/**
 * @brief TraceSink 定义结构化记录写入接口。
 */
class ONEQ_API TraceSink {
 public:
  virtual ~TraceSink();

  /**
   * @brief 写入一条 JSON record。
   * @param[in] module 模块标识。
   * @param[in] phase 记录阶段，例如 config/input/output。
   * @param[in] payload_json 已构造好的 JSON 对象文本。
   */
  virtual void Record(const std::string& module, const std::string& phase,
                      const std::string& payload_json) = 0;
};

/**
 * @brief JsonlFileTraceSink 将记录以 JSON Lines 追加写入文件。
 */
class ONEQ_API JsonlFileTraceSink final : public TraceSink {
 public:
  explicit JsonlFileTraceSink(std::string file_path, bool append = true);
  ~JsonlFileTraceSink() override;

  void Record(const std::string& module, const std::string& phase,
              const std::string& payload_json) override;

  const std::string& file_path() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * @brief FlatbufferFileTraceSink 将记录以 FlatBuffers(FlexBuffers) 二进制帧写入文件。
 *
 * 约定：
 *   - 每条记录按 `uint32_le length + payload bytes` 顺序写入。
 *   - payload 为一条 FlexBuffers map，键包括 timestamp_ms/module/phase/payload_json。
 */
class ONEQ_API FlatbufferFileTraceSink final : public TraceSink {
 public:
  explicit FlatbufferFileTraceSink(std::string file_path, bool append = true);
  ~FlatbufferFileTraceSink() override;

  void Record(const std::string& module, const std::string& phase,
              const std::string& payload_json) override;

  const std::string& file_path() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * @brief PlatformFileTraceSink 根据平台自动选择数据采集后端。
 *
 * - Windows: FlatbufferFileTraceSink
 * - 其他平台（含 macOS）: JsonlFileTraceSink
 */
class ONEQ_API PlatformFileTraceSink final : public TraceSink {
 public:
  explicit PlatformFileTraceSink(std::string file_path, bool append = true);
  ~PlatformFileTraceSink() override;

  void Record(const std::string& module, const std::string& phase,
              const std::string& payload_json) override;

  const std::string& file_path() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace trace
}  // namespace oneq

#endif  // ONEQ_TRACE_TRACE_SINK_H_
