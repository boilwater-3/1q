/**
 * @file PrecisionAcceptanceRecords.cpp
 * @brief 精度评估验收行：关键精度指标派生 + AHP 等级/贡献排序。
 */

#include "precision_evaluation/PrecisionAcceptanceRecords.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "1q/precision_evaluation/PrecisionEvaluationMetrics.h"
#include "common/logging/AcceptanceText.h"
#include "precision_evaluation/PrecisionEvaluationLog.h"

namespace precision_evaluation {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatPairDeg;
using oneq::logging::FormatVec3;

double RmseOf(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double sumsq = 0.0;
  for (double value : values) {
    sumsq += value * value;
  }
  return std::sqrt(sumsq / static_cast<double>(values.size()));
}

const char* GradeOf(double score) {
  if (score >= 0.85) {
    return "A";
  }
  if (score >= 0.70) {
    return "B";
  }
  if (score >= 0.55) {
    return "C";
  }
  return "D";
}

const char* MetricName(std::size_t index) {
  static const char* kNames[kPrecisionMetricCount] = {"测角", "双星", "速度", "落点", "发射点"};
  return index < kPrecisionMetricCount ? kNames[index] : "未知";
}

}  // namespace

void WritePrecisionKeyMetrics(const PrecisionEvaluationReport& report,
                              const std::vector<double>& east_m, const std::vector<double>& north_m,
                              const std::vector<double>& up_m, const std::vector<double>& az_deg,
                              const std::vector<double>& el_deg) {
  if (!PRECISION_EVAL_LOG_ENABLED()) {
    return;
  }
  const double east_rmse = RmseOf(east_m);
  const double north_rmse = RmseOf(north_m);
  const double up_rmse = RmseOf(up_m);
  const double horiz_rmse = std::hypot(east_rmse, north_rmse);
  const double range_rmse = report.metrics[static_cast<std::size_t>(PrecisionMetric::kDualSatFix)].rmse;
  const double az_rmse = RmseOf(az_deg);
  const double el_rmse = RmseOf(el_deg);
  const double composite_rmse =
      std::sqrt(east_rmse * east_rmse + north_rmse * north_rmse + up_rmse * up_rmse);
  const double cep50 = 0.5887 * horiz_rmse;
  const std::size_t n = east_m.size();
  const double mean_range = report.metrics[static_cast<std::size_t>(PrecisionMetric::kDualSatFix)].mean;
  const double ci_half =
      n > 0U ? 1.96 * range_rmse / std::sqrt(static_cast<double>(n)) : 0.0;

  std::string content = "东/北/天RMSE=" + FormatVec3(east_rmse, north_rmse, up_rmse, 1) + "m";
  content += " 距离RMSE=" + FormatF(range_rmse, 1) + "m";
  content += " 方位/俯仰RMSE=" + FormatPairDeg(az_rmse, el_rmse, 4) + "°";
  content += " 合成RMSE=" + FormatF(composite_rmse, 1) + "m";
  content += " CEP50=" + FormatF(cep50, 1) + "m";
  if (n > 0U) {
    content += " 位置误差95%CI=[" + FormatF(mean_range - ci_half, 1) + "," +
               FormatF(mean_range + ci_half, 1) + "]m";
  } else {
    content += " 位置误差95%CI=无";
  }
  // 误差源贡献率按 2026-08-22 甲方批注「不需要，删了」移除（无分源灵敏度模型）。
  PRECISION_EVAL_ITEM(0.0f, 0U, "关键精度指标", content);
}

void WritePrecisionAhp(const PrecisionEvaluationReport& report) {
  if (!PRECISION_EVAL_LOG_ENABLED()) {
    return;
  }
  if (!report.ahp_valid) {
    PRECISION_EVAL_ITEM(0.0f, 0U, "层次分析法", "暂无");
    return;
  }
  std::string weights = "(";
  std::string scores = "(";
  for (std::size_t i = 0; i < kPrecisionMetricCount; ++i) {
    if (i != 0U) {
      weights += ",";
      scores += ",";
    }
    weights += FormatF(report.ahp.weights[i], 3);
    scores += FormatF(report.metric_scores[i], 3);
  }
  weights += ")";
  scores += ")";

  std::vector<std::pair<double, std::size_t>> order;
  for (std::size_t i = 0; i < kPrecisionMetricCount; ++i) {
    order.push_back(std::make_pair(report.metric_contributions[i], i));
  }
  std::sort(order.begin(), order.end(),
            [](const std::pair<double, std::size_t>& lhs, const std::pair<double, std::size_t>& rhs) {
              return lhs.first > rhs.first;
            });
  std::string rank;
  for (std::size_t i = 0; i < order.size(); ++i) {
    if (i != 0U) {
      rank += ">";
    }
    rank += MetricName(order[i].second);
  }

  std::string content = "准则层权重=" + weights;
  content += " 底层分=" + scores;
  content += " 综合分=" + FormatF(report.composite_score, 3);
  content += std::string(" 等级=") + GradeOf(report.composite_score);
  content += " 贡献排序=" + rank;
  content += " CR=" + FormatF(report.ahp.consistency_ratio, 3);
  content += " 独立多层树=无";
  PRECISION_EVAL_ITEM(0.0f, 0U, "层次分析法", content);
}

}  // namespace precision_evaluation
