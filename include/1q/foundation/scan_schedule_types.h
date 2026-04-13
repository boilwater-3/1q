/**
 * @file scan_schedule_types.h
 * @brief 定义跨雷达模块复用的扫描调度轻量原语。
 */

#ifndef ONEQ_FOUNDATION_SCAN_SCHEDULE_TYPES_H_
#define ONEQ_FOUNDATION_SCAN_SCHEDULE_TYPES_H_

#include "1q/api.hpp"

namespace oneq {
namespace foundation {

/**
 * @brief ScanStartPosition 描述二维扫描的起始象限。
 */
enum class ONEQ_API ScanStartPosition {
  kLeftTop = 0, /**< 左上起始 */
  kRightTop,    /**< 右上起始 */
  kRightBottom, /**< 右下起始 */
  kLeftBottom   /**< 左下起始 */
};

/**
 * @brief ScanSequence 描述二维扫描推进顺序。
 */
enum class ONEQ_API ScanSequence {
  kAzimuthFirst = 0, /**< 先方位后俯仰 */
  kElevationFirst    /**< 先俯仰后方位 */
};

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_SCAN_SCHEDULE_TYPES_H_
