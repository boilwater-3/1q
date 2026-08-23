/**
 * @file AcceptanceText.h
 * @brief 验收内容字段的轻量文本拼装（内部，不进公开 API）。
 */

#ifndef ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_TEXT_H_
#define ONEQ_SRC_COMMON_LOGGING_ACCEPTANCE_TEXT_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace oneq {
namespace logging {

inline std::string FormatF(double value, int precision) {
  char buf[64];
  switch (precision) {
    case 0:
      std::snprintf(buf, sizeof(buf), "%.0f", value);
      break;
    case 1:
      std::snprintf(buf, sizeof(buf), "%.1f", value);
      break;
    case 2:
      std::snprintf(buf, sizeof(buf), "%.2f", value);
      break;
    case 3:
      std::snprintf(buf, sizeof(buf), "%.3f", value);
      break;
    case 4:
      std::snprintf(buf, sizeof(buf), "%.4f", value);
      break;
    case 5:
      std::snprintf(buf, sizeof(buf), "%.5f", value);
      break;
    default:
      std::snprintf(buf, sizeof(buf), "%.6f", value);
      break;
  }
  return std::string(buf);
}

inline std::string FormatSci(double value) {
  if (value == 0.0) {
    return "0.000";  // 零不进科学计数，避免 0.000e+00 与 0.000 混排
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.3e", value);
  return std::string(buf);
}

inline const char* YesNo(bool value) { return value ? "是" : "否"; }

inline std::string FormatPairDeg(double first, double second, int precision) {
  return "(" + FormatF(first, precision) + "," + FormatF(second, precision) + ")";
}

inline std::string FormatVec3(double x, double y, double z, int precision) {
  return "(" + FormatF(x, precision) + "," + FormatF(y, precision) + "," + FormatF(z, precision) +
         ")";
}

inline std::string FormatCovDiag6(const std::array<double, 36U>& cov) {
  return "(" + FormatF(cov[0], 3) + "," + FormatF(cov[14], 3) + "," + FormatF(cov[28], 3) + "," +
         FormatF(cov[7], 3) + "," + FormatF(cov[21], 3) + "," + FormatF(cov[35], 3) + ")";
}

inline double CovarianceTrace6(const std::array<double, 36U>& cov) {
  return cov[0] + cov[7] + cov[14] + cov[21] + cov[28] + cov[35];
}

inline std::string FormatCov6x6(const std::array<double, 36U>& cov) {
  std::string text = "[";
  for (int row = 0; row < 6; ++row) {
    if (row != 0) {
      text += ";";
    }
    for (int col = 0; col < 6; ++col) {
      if (col != 0) {
        text += ",";
      }
      text += FormatF(cov[static_cast<std::size_t>(row) * 6U + static_cast<std::size_t>(col)], 6);
    }
  }
  text += "]";
  return text;
}

}  // namespace logging
}  // namespace oneq

#endif
