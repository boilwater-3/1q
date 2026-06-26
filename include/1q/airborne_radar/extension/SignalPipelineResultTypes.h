/**
 * @file SignalPipelineResultTypes.h
 * @brief 兼容转发头——真源定义已迁移至 session/RadarOutputTypes.h。
 *
 * @deprecated 新代码请直接使用 `session/RadarOutputTypes.h` 与
 *             `airborne_radar::session` 命名空间下的类型。
 *             本文件提供的 `extension::` using 别名将在后续大版本移除。
 */

#ifndef ONEQ_AIRBORNE_RADAR_EXTENSION_SIGNAL_PIPELINE_RESULT_TYPES_H_
#define ONEQ_AIRBORNE_RADAR_EXTENSION_SIGNAL_PIPELINE_RESULT_TYPES_H_

#include "1q/airborne_radar/session/RadarOutputTypes.h"

namespace airborne_radar {
namespace extension {

using session::SignalCycleAbortReason;
using session::AssociationQualityMetrics;
using session::SignalCycleResult;

}  // namespace extension
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_EXTENSION_SIGNAL_PIPELINE_RESULT_TYPES_H_
