/**
 * @file PrecisionEvaluationLog.h
 * @brief 精度评估验收日志宏：把需求 3.2.1.6.3 章节的误差样本与 AHP 评分写入项目日志。
 *
 * 编译期由 CMake 开关 `ONEQ_ENABLE_PRECISION_EVALUATION_LOG` 门控（默认 OFF）：
 *  - ON：`PRECISION_EVAL_LOG(...)` 展开为 `PROJECT_LOG_INFO("[PrecisionEval] " ...)`，
 *    走既有 spdlog / 文件日志后端（1q_library.log），前缀 `[PrecisionEval]` 便于验收
 *    grep；格式串只允许 `{}` 与 `{:.Nf}` 两种占位符（文件后端迷你格式化约束）。
 *  - OFF：宏为空操作，`PRECISION_EVAL_LOG_ENABLED()` 为 false，调用点外层
 *    `if (PRECISION_EVAL_LOG_ENABLED())` 内的派生计算一并被编译器剪除，零开销。
 *
 * 事件类型（event= 字段）：angular_error（3.2.1.6.3 角度误差）、dual_sat_fix（双星
 * 交会位置误差+几何残差）、velocity_error（融合速度误差，附位置误差）、
 * keypoint_error（落点/发射点预测误差）、metric_summary（五指标汇总）、ahp_score
 * （AHP 权重/一致性/贡献/综合分）。日志仅人读验收材料，不是机器契约（三写约束不变）。
 */

#ifndef ONEQ_SRC_PRECISION_EVALUATION_PRECISION_EVALUATION_LOG_H_
#define ONEQ_SRC_PRECISION_EVALUATION_PRECISION_EVALUATION_LOG_H_

#include "common/logging/ProjectLog.h"

#if defined(ONEQ_ENABLE_PRECISION_EVALUATION_LOG) && ONEQ_ENABLE_PRECISION_EVALUATION_LOG

#define PRECISION_EVAL_LOG_ENABLED() true
// 字符串字面量拼接注入统一前缀；首参恒为格式串（编译期可查）。
#define PRECISION_EVAL_LOG(...) PROJECT_LOG_INFO("[PrecisionEval] " __VA_ARGS__)

#else

#define PRECISION_EVAL_LOG_ENABLED() false
#define PRECISION_EVAL_LOG(...) ((void)0)

#endif

#endif  // ONEQ_SRC_PRECISION_EVALUATION_PRECISION_EVALUATION_LOG_H_
