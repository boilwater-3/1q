/**
 * @file CalendarUtils.h
 * @brief 日历转换内部工具函数。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_CALENDAR_UTILS_H_
#define AIRBORNE_RADAR_ENVIRONMENT_CALENDAR_UTILS_H_

#include <cstdint>

namespace airborne_radar {
namespace environment {

inline std::int64_t FloorDiv(std::int64_t numerator, std::int64_t denominator) {
  const std::int64_t quotient = numerator / denominator;
  const std::int64_t remainder = numerator % denominator;
  if (remainder != 0 && ((remainder > 0) != (denominator > 0))) {
    return quotient - 1;
  }
  return quotient;
}

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
