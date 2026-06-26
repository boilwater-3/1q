/**
 * @file EosPipelineTypes.h
 * @brief 兼容转发头——真源定义已迁移至 session/EosOutputTypes.h。
 *
 * @deprecated 新代码请直接使用 `session/EosOutputTypes.h`。
 *             `output::EosDetectionRecord` 和 `attribution::EosDetectionAttributionRecord`
 *             命名空间不变；`extension::EosPipelineAbortReason` 已迁至
 *             `session::EosPipelineAbortReason`，本文件以 using 别名保持兼容。
 *             本文件提供的 `extension::` 别名将在后续大版本移除。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_EXTENSION_EOS_PIPELINE_TYPES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_EXTENSION_EOS_PIPELINE_TYPES_H_

#include "1q/electro_optical_sensor/session/EosOutputTypes.h"

namespace electro_optical_sensor {
namespace extension {

using session::EosPipelineAbortReason;

}  // namespace extension
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_EXTENSION_EOS_PIPELINE_TYPES_H_
