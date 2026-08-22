/**
 * @file AcceptanceFileLog.cpp
 * @brief 五通道验收文件 sink（红外 / 雷达 / 融合 / 推演 / 精度）。
 */

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/LogDirectory.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>

#ifndef ONEQ_SBIRS_ACCEPTANCE_LOG_PATH
#define ONEQ_SBIRS_ACCEPTANCE_LOG_PATH "log/sbirs_acceptance.log"
#endif
#ifndef ONEQ_RIR_ACCEPTANCE_LOG_PATH
#define ONEQ_RIR_ACCEPTANCE_LOG_PATH "log/rir_acceptance.log"
#endif
#ifndef ONEQ_FUSION_ACCEPTANCE_LOG_PATH
#define ONEQ_FUSION_ACCEPTANCE_LOG_PATH "log/fusion_acceptance.log"
#endif
#ifndef ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH
#define ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH "log/inference_acceptance.log"
#endif
#ifndef ONEQ_PRECISION_ACCEPTANCE_LOG_PATH
#define ONEQ_PRECISION_ACCEPTANCE_LOG_PATH "log/precision_acceptance.log"
#endif

namespace oneq {
namespace logging {
namespace {

const char* EnvName(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSbirs:
      return "ONEQ_SBIRS_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kRir:
      return "ONEQ_RIR_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kFusion:
      return "ONEQ_FUSION_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kInference:
      return "ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH";
    case AcceptanceChannel::kPrecision:
      return "ONEQ_PRECISION_ACCEPTANCE_LOG_PATH";
  }
  return "ONEQ_SBIRS_ACCEPTANCE_LOG_PATH";
}

const char* CompileDefaultPath(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSbirs:
      return ONEQ_SBIRS_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kRir:
      return ONEQ_RIR_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kFusion:
      return ONEQ_FUSION_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kInference:
      return ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH;
    case AcceptanceChannel::kPrecision:
      return ONEQ_PRECISION_ACCEPTANCE_LOG_PATH;
  }
  return ONEQ_SBIRS_ACCEPTANCE_LOG_PATH;
}

const char* ChannelLabel(AcceptanceChannel channel) {
  switch (channel) {
    case AcceptanceChannel::kSbirs:
      return "sbirs";
    case AcceptanceChannel::kRir:
      return "rir";
    case AcceptanceChannel::kFusion:
      return "fusion";
    case AcceptanceChannel::kInference:
      return "inference";
    case AcceptanceChannel::kPrecision:
      return "precision";
  }
  return "sbirs";
}

std::string ResolveDefaultPath(AcceptanceChannel channel) {
#if defined(_MSC_VER)
  char* buffer = nullptr;
  if (_dupenv_s(&buffer, nullptr, EnvName(channel)) == 0 && buffer != nullptr) {
    std::string result(buffer);
    std::free(buffer);
    if (!result.empty()) {
      return result;
    }
  }
#else
  const char* env = std::getenv(EnvName(channel));
  if (env != nullptr && *env != '\0') {
    return std::string(env);
  }
#endif
  return std::string(CompileDefaultPath(channel));
}

class ChannelSink {
 public:
  explicit ChannelSink(AcceptanceChannel channel) : channel_(channel) {}

  void Open(const char* path) {
    std::lock_guard<std::mutex> lock(mutex_);
    DoOpen(path);
  }

  void Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }
    open_failed_ = false;
    warned_ = false;
  }

  void Flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
      stream_.flush();
    }
  }

  bool IsOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stream_.is_open();
  }

  void Write(const std::string& text) {
    if (text.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_.is_open() && !open_failed_) {
      const std::string path = ResolveDefaultPath(channel_);
      DoOpen(path.c_str());
    }
    if (!stream_.is_open()) {
      return;
    }
    stream_ << text;
    if (text[text.size() - 1U] != '\n') {
      stream_ << '\n';
    }
  }

 private:
  void DoOpen(const char* path) {
    if (path == nullptr || *path == '\0') {
      return;
    }
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }
    oneq::logging::EnsureParentDirectory(path);
    stream_.open(path, std::ios::out | std::ios::app | std::ios::binary);
    if (stream_.is_open()) {
      open_failed_ = false;
      warned_ = false;
    } else {
      open_failed_ = true;
      if (!warned_) {
        warned_ = true;
        std::fprintf(stderr, "[1q %s acceptance log] cannot open log file: %s\n",
                     ChannelLabel(channel_), path);
        std::fflush(stderr);
      }
    }
  }

  AcceptanceChannel channel_;
  mutable std::mutex mutex_;
  std::ofstream stream_;
  bool open_failed_ = false;
  bool warned_ = false;
};

ChannelSink& SinkOf(AcceptanceChannel channel) {
  static ChannelSink sbirs(AcceptanceChannel::kSbirs);
  static ChannelSink rir(AcceptanceChannel::kRir);
  static ChannelSink fusion(AcceptanceChannel::kFusion);
  static ChannelSink inference(AcceptanceChannel::kInference);
  static ChannelSink precision(AcceptanceChannel::kPrecision);
  switch (channel) {
    case AcceptanceChannel::kRir:
      return rir;
    case AcceptanceChannel::kFusion:
      return fusion;
    case AcceptanceChannel::kInference:
      return inference;
    case AcceptanceChannel::kPrecision:
      return precision;
    case AcceptanceChannel::kSbirs:
    default:
      return sbirs;
  }
}

}  // namespace

void OpenAcceptanceLog(AcceptanceChannel channel, const char* path) {
  SinkOf(channel).Open(path);
}

void CloseAcceptanceLog(AcceptanceChannel channel) { SinkOf(channel).Close(); }

void FlushAcceptanceLog(AcceptanceChannel channel) { SinkOf(channel).Flush(); }

bool IsAcceptanceLogOpen(AcceptanceChannel channel) { return SinkOf(channel).IsOpen(); }

void WriteAcceptanceLog(AcceptanceChannel channel, const char* text) {
  if (text == nullptr || *text == '\0') {
    return;
  }
  SinkOf(channel).Write(std::string(text));
}

void WriteAcceptanceLog(AcceptanceChannel channel, const std::string& text) {
  SinkOf(channel).Write(text);
}

}  // namespace logging
}  // namespace oneq
