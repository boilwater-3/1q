/**
 * @file InferenceAcceptanceLog.h
 * @brief 推演层验收日志宏：按项写入 inference_acceptance.log（四段同一行）。
 */

#ifndef ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_LOG_H_

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#if defined(ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG) && ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG

#define INFERENCE_ACCEPTANCE_LOG_ENABLED() true
#define INFERENCE_ACCEPTANCE_RECORD(text) \
  ::oneq::logging::WriteAcceptanceLog(::oneq::logging::AcceptanceChannel::kInference, (text))
#define INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  INFERENCE_ACCEPTANCE_RECORD(                                    \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define INFERENCE_ACCEPTANCE_LOG_ENABLED() false
#define INFERENCE_ACCEPTANCE_RECORD(text) ((void)0)
#define INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#endif  // ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_LOG_H_
