/**
 * @file batch_csv_writer.h
 * @brief 批量场景验证共享工具：流式 CSV 写入器。
 *
 * @par 设计目标
 * 仿照 examples/flight_dynamic/orbit_quality_csv.cpp 的 fprintf 风格，
 * 提供一个"打开文件 → 写表头 → 逐行追加 → 关闭"的最小工具，
 * 供四个模块的批量验证程序统一产出周期级 / 场景汇总级 CSV。
 *
 * 不引入新依赖，仅用标准库；进度信息一律写 stderr，避免污染 CSV。
 */

#ifndef EXAMPLES_BATCH_VALIDATION_BATCH_CSV_WRITER_H_
#define EXAMPLES_BATCH_VALIDATION_BATCH_CSV_WRITER_H_

#include <cstdio>
#include <cstring>
#include <string>

namespace batch_validation {

/**
 * @brief 流式 CSV 写入器。
 *
 * 构造时打开文件并立即写入表头；后续调用 WriteRow 追加数据行；
 * 析构自动关闭。文件打开失败时 fatal() 终止程序（批量程序无此文件无法继续）。
 */
class CsvWriter {
 public:
  /**
   * @brief 构造并打开 CSV 文件。
   * @param[in] path  输出文件路径；为 "-" 时写到 stdout。
   * @param[in] header  逗号分隔的表头行（不含换行）。
   */
  CsvWriter(const std::string& path, const std::string& header) : path_(path), owns_file_(true) {
    if (path == "-") {
      file_ = stdout;
      owns_file_ = false;
    } else {
      file_ = std::fopen(path.c_str(), "w");
    }
    if (file_ == nullptr) {
      std::fprintf(stderr, "[batch_validation] FATAL: 无法打开 CSV 文件 %s\n", path.c_str());
      std::abort();
    }
    std::fprintf(file_, "%s\n", header.c_str());
  }

  ~CsvWriter() {
    if (owns_file_ && file_ != nullptr) {
      std::fclose(file_);
    }
  }

  CsvWriter(const CsvWriter&) = delete;
  CsvWriter& operator=(const CsvWriter&) = delete;

  /// 已打开的底层文件指针（供调用方 fprintf 定制格式）。
  std::FILE* file() const { return file_; }

  /// 追加一行（content 不含换行，由本方法补 \n）。
  void WriteRow(const std::string& content) { std::fprintf(file_, "%s\n", content.c_str()); }

  /// 立即把缓冲区刷到磁盘（在 trace 回放前确保 CSV 落盘）。
  void Flush() {
    if (file_ != nullptr) std::fflush(file_);
  }

 private:
  std::string path_;
  std::FILE* file_{nullptr};
  bool owns_file_{false};
};

/**
 * @brief 将 CSV 字段中的逗号 / 换行 / 双引号转义，保证单列完整性。
 *
 * 按 RFC 4180：含逗号、双引号或换行的字段用双引号包裹，内部双引号翻倍。
 * 不含这些字符的字段原样返回（批量程序的枚举名 / 数值不触发转义，但仍安全）。
 */
inline std::string EscapeCsvField(const std::string& raw) {
  bool need_quote = false;
  for (char c : raw) {
    if (c == ',' || c == '"' || c == '\n' || c == '\r') {
      need_quote = true;
      break;
    }
  }
  if (!need_quote) return raw;
  std::string out;
  out.reserve(raw.size() + 2);
  out.push_back('"');
  for (char c : raw) {
    if (c == '"') {
      out.push_back('"');
      out.push_back('"');
    } else {
      out.push_back(c);
    }
  }
  out.push_back('"');
  return out;
}

}  // namespace batch_validation

#endif  // EXAMPLES_BATCH_VALIDATION_BATCH_CSV_WRITER_H_
