/**
 * @file CalendarUtils.h
 * @brief 日历转换内部工具函数。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_CALENDAR_UTILS_H_
#define AIRBORNE_RADAR_ENVIRONMENT_CALENDAR_UTILS_H_

#include <cstdint>

namespace airborne_radar {
namespace environment {

/**
 * @brief 向下取整的整数除法（对负数结果向负无穷方向舍入）。
 * @param[in] numerator 被除数。
 * @param[in] denominator 除数（不可为零）。
 * @return 向下取整后的整商。
 */
inline std::int64_t FloorDiv(std::int64_t numerator, std::int64_t denominator) {
  const std::int64_t quotient = numerator / denominator;
  const std::int64_t remainder = numerator % denominator;
  if (remainder != 0 && ((remainder > 0) != (denominator > 0))) {
    return quotient - 1;
  }
  return quotient;
}

/**
 * @brief 由 Unix 时间戳（秒）推算当年内的儒略日序号（day-of-year）。
 *
 * 采用 Howard Hinnant 的 civil_from_days 算法将天换算为年/月/日，
 * 再结合闰年判定累计当年 1 月 1 日起的天数。
 * @param[in] unix_seconds 距 Unix 纪元（1970-01-01 00:00:00 UTC）的秒数。
 * @return 当年内的天序号（1 月 1 日为 1）。
 */
inline std::int32_t ResolveDayOfYearFromUnixSeconds(std::int64_t unix_seconds) {
  const std::int64_t days_since_epoch = FloorDiv(unix_seconds, 86400);
  std::int64_t z = days_since_epoch + 719468;
  const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const std::int64_t doe = z - era * 146097;
  const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const std::int64_t mp = (5 * doy + 2) / 153;
  const std::int32_t day = static_cast<std::int32_t>(doy - (153 * mp + 2) / 5 + 1);
  const std::int32_t month = static_cast<std::int32_t>(mp + (mp < 10 ? 3 : -9));
  const std::int32_t year = static_cast<std::int32_t>(yoe + era * 400 + (month <= 2 ? 1 : 0));

  const bool is_leap_year = (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
  static const std::int32_t cumulative_days_before_month[12] = {0,   31,  59,  90,  120, 151,
                                                                181, 212, 243, 273, 304, 334};
  const std::int32_t base = cumulative_days_before_month[month - 1];
  const std::int32_t leap_day = (is_leap_year && month > 2) ? 1 : 0;
  return base + day + leap_day;
}

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_CALENDAR_UTILS_H_
