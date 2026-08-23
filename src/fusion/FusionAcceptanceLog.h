/**
 * @file FusionAcceptanceLog.h
 * @brief 融合层验收日志宏：按项写入 fusion_acceptance.log（四段同一行）。
 */

#ifndef ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_LOG_H_

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#if defined(ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG) && ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG

#define FUSION_ACCEPTANCE_LOG_ENABLED() true
#define FUSION_ACCEPTANCE_RECORD(text) \
  ::oneq::logging::WriteAcceptanceLog(::oneq::logging::AcceptanceChannel::kFusion, (text))
#define FUSION_ACCEPTANCE_ITEM(sim_time, cycle, item, content) \
  FUSION_ACCEPTANCE_RECORD(                                    \
      ::oneq::logging::FormatAcceptanceLine((sim_time), (cycle), (item), (content)))

#else

#define FUSION_ACCEPTANCE_LOG_ENABLED() false
#define FUSION_ACCEPTANCE_RECORD(text) ((void)0)
#define FUSION_ACCEPTANCE_ITEM(sim_time, cycle, item, content) ((void)0)

#endif

#endif  // ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_LOG_H_
