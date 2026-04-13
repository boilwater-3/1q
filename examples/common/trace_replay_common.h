// Copyright 2026. All Rights Reserved.
//
// @file trace_replay_common.h
// @brief Trace 回放通用工具：JSONL 读取、模块过滤与归一化比对。

#ifndef ONEQ_EXAMPLES_COMMON_TRACE_REPLAY_COMMON_H_
#define ONEQ_EXAMPLES_COMMON_TRACE_REPLAY_COMMON_H_

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace oneq {
namespace examples {
namespace replay {

using Json = nlohmann::ordered_json;

/**
 * @brief TraceRecord 表示 JSONL 中单条记录。
 */
struct TraceRecord {
  Json line{};
  std::string phase{};
  Json payload{};
};

inline bool LoadTraceRecords(const std::string& trace_path, const std::string& module,
                             std::vector<TraceRecord>* records, std::string* error_message) {
  if (records == nullptr) {
    if (error_message != nullptr) {
      *error_message = "records 输出参数为空";
    }
    return false;
  }
  records->clear();

  std::ifstream input(trace_path.c_str());
  if (!input.is_open()) {
    if (error_message != nullptr) {
      *error_message = "无法打开 trace 文件: " + trace_path;
    }
    return false;
  }

  std::string line;
  std::size_t line_index = 0U;
  while (std::getline(input, line)) {
    ++line_index;
    if (line.empty()) {
      continue;
    }
    Json parsed;
    try {
      parsed = Json::parse(line);
    } catch (const std::exception& e) {
      if (error_message != nullptr) {
        std::ostringstream oss;
        oss << "trace 解析失败，行号=" << line_index << ", 原因=" << e.what();
        *error_message = oss.str();
      }
      return false;
    }

    if (!parsed.is_object()) {
      continue;
    }
    if (parsed.value("module", std::string()) != module) {
      continue;
    }

    TraceRecord record;
    record.line = parsed;
    record.phase = parsed.value("phase", std::string());
    record.payload = parsed.value("payload", Json::object());
    records->push_back(record);
  }

  return true;
}

inline Json NormalizeForCompare(Json value) {
  value.erase("timestamp_ms");
  return value;
}

inline bool CompareTraceRecords(const std::vector<TraceRecord>& expected,
                                const std::vector<TraceRecord>& actual,
                                std::string* diff_message) {
  if (expected.size() != actual.size()) {
    if (diff_message != nullptr) {
      std::ostringstream oss;
      oss << "记录条数不一致，expected=" << expected.size() << " actual=" << actual.size();
      *diff_message = oss.str();
    }
    return false;
  }

  for (std::size_t i = 0; i < expected.size(); ++i) {
    const Json lhs = NormalizeForCompare(expected[i].line);
    const Json rhs = NormalizeForCompare(actual[i].line);
    if (lhs != rhs) {
      if (diff_message != nullptr) {
        std::ostringstream oss;
        oss << "第 " << i << " 条记录不一致, phase(expected)=" << expected[i].phase
            << ", phase(actual)=" << actual[i].phase;
        *diff_message = oss.str();
      }
      return false;
    }
  }
  return true;
}

}  // namespace replay
}  // namespace examples
}  // namespace oneq

#endif  // ONEQ_EXAMPLES_COMMON_TRACE_REPLAY_COMMON_H_
