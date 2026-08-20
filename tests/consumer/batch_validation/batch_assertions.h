/**
 * @file batch_assertions.h
 * @brief 批量场景验证共享工具：软断言收集器 + 统计辅助。
 *
 * @par 设计目标
 * 批量验证采用"数据采集 + 物理合理性软断言"策略：对关键物理趋势做检查，
 * 违反时记录 kWarning 而不中断程序（既证明泛用性又能自动捕捉异常）。
 * 收集的告警会写入场景汇总 CSV 的 warning 列，供分析脚本高亮。
 *
 * @par 软断言 vs 硬断言
 * - 软断言（本文件）：物理趋势合理性，违反记 warning，程序继续。
 * - 硬断言（回放分叉检测）：replay_ok==false 属严重错误，但仍只记入 CSV，
 *   不终止批量运行——以便一次性看到所有场景的状况。
 */

#ifndef EXAMPLES_BATCH_VALIDATION_BATCH_ASSERTIONS_H_
#define EXAMPLES_BATCH_VALIDATION_BATCH_ASSERTIONS_H_

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace batch_validation {

/// 告警级别（与 SAR 诊断层语义对齐：Info/Warning/Error）。
enum class Severity { kInfo = 0, kWarning = 1, kError = 2 };

/**
 * @brief 软断言收集器。
 *
 * 收集各场景运行期间的告警条目，提供汇总查询（按级别计数、拼接为单字符串写 CSV）。
 * 不抛异常；所有方法可在仿真热点路径之外安全调用。
 */
class WarningCollector {
 public:
  /// 记录一条告警。
  void Add(Severity sev, const std::string& message) {
    items_.push_back({sev, message});
  }

  /// 便捷：记录一条 kWarning。
  void Warn(const std::string& message) { Add(Severity::kWarning, message); }

  /// 便捷：记录一条 kError。
  void Error(const std::string& message) { Add(Severity::kError, message); }

  /// 是否存在指定级别及以上的告警。
  bool HasAtLeast(Severity sev) const {
    for (const auto& it : items_) {
      if (static_cast<int>(it.sev) >= static_cast<int>(sev)) return true;
    }
    return false;
  }

  /// 指定级别的条目数。
  std::size_t Count(Severity sev) const {
    std::size_t n = 0;
    for (const auto& it : items_) {
      if (it.sev == sev) ++n;
    }
    return n;
  }

  /// 将所有告警拼接为单行字符串（分号分隔），供写入 CSV 单元格；级别用 [W]/[E] 前缀。
  std::string JoinForCsv() const {
    std::ostringstream oss;
    for (std::size_t i = 0; i < items_.size(); ++i) {
      if (i > 0) oss << " ; ";
      const char* tag = (items_[i].sev == Severity::kError)   ? "[E]"
                        : (items_[i].sev == Severity::kWarning) ? "[W]"
                                                                 : "[I]";
      oss << tag << " " << items_[i].message;
    }
    return oss.str();
  }

  /// 打印所有告警到 stderr。
  void DumpToStderr(const std::string& prefix) const {
    for (const auto& it : items_) {
      const char* tag = (it.sev == Severity::kError)   ? "ERROR"
                        : (it.sev == Severity::kWarning) ? "WARN "
                                                          : "INFO ";
      std::fprintf(stderr, "    [%s] %s%s\n", tag, prefix.c_str(), it.message.c_str());
    }
  }

 private:
  struct Item {
    Severity sev;
    std::string message;
  };
  std::vector<Item> items_;
};

// =============================================================================
// 统计辅助（周期级指标聚合为场景级摘要）
// =============================================================================

/**
 * @brief 计算向量的百分位数（线性插值）。
 *
 * @param[in] values  样本（将被拷贝排序，原顺序不影响）。
 * @param[in] pct     百分位 [0,100]，如 50=中位数、95=P95。
 * @return 百分位值；空样本返回 0。
 */
inline double Percentile(std::vector<double> values, double pct) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  if (pct <= 0.0) return values.front();
  if (pct >= 100.0) return values.back();
  const double rank = (pct / 100.0) * static_cast<double>(values.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(rank);
  const std::size_t hi = (lo + 1 < values.size()) ? lo + 1 : lo;
  const double frac = rank - static_cast<double>(lo);
  return values[lo] * (1.0 - frac) + values[hi] * frac;
}

/// 计算向量均值；空样本返回 0。
inline double Mean(const std::vector<double>& values) {
  if (values.empty()) return 0.0;
  double sum = 0.0;
  for (double v : values) sum += v;
  return sum / static_cast<double>(values.size());
}

/// 计算向量求和。
inline double Sum(const std::vector<double>& values) {
  double s = 0.0;
  for (double v : values) s += v;
  return s;
}

/**
 * @brief 检查序列是否（非严格）单调递减。
 *
 * 用于跨场景趋势软断言：如"距离↑ 时检出率应单调↓"。NaN 视为违反。
 * @param[in] values  与自变量同序的因变量序列。
 * @return true 若对所有 i，values[i] >= values[i+1]（允许相等）。
 */
inline bool IsMonotonicNonIncreasing(const std::vector<double>& values) {
  for (std::size_t i = 1; i < values.size(); ++i) {
    if (!(values[i - 1] >= values[i])) return false;
  }
  return values.size() >= 2;  // 单点无趋势可言，返回 false
}

/**
 * @brief 检查序列是否（非严格）单调递增。
 * @sa IsMonotonicNonIncreasing
 */
inline bool IsMonotonicNonDecreasing(const std::vector<double>& values) {
  for (std::size_t i = 1; i < values.size(); ++i) {
    if (!(values[i - 1] <= values[i])) return false;
  }
  return values.size() >= 2;
}

}  // namespace batch_validation

#endif  // EXAMPLES_BATCH_VALIDATION_BATCH_ASSERTIONS_H_
