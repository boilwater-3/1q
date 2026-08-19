/**
 * @file RirAcceptanceLog.h
 * @brief RIR 验收信息日志宏：把《红外载荷与远程识别雷达需求源码对应关系》第 4 章
 *        （3.2.2 远程识别雷达需求映射）要求的验收量按周期写入项目日志。
 *
 * 编译期由 CMake 开关 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` 门控（默认 OFF）：
 *  - ON：`RIR_ACCEPTANCE_LOG(...)` 展开为 `PROJECT_LOG_INFO("[RirAccept] " ...)`，
 *    走既有 spdlog / 文件日志后端（1q_library.log），消息前缀 `[RirAccept]` 便于
 *    验收时 grep；格式串只允许 `{}` 与 `{:.Nf}` 两种占位符（文件后端迷你格式化约束）。
 *  - OFF：宏展开为空操作，`RIR_ACCEPTANCE_LOG_ENABLED()` 为 false，调用点外层
 *    `if (RIR_ACCEPTANCE_LOG_ENABLED()) { ... }` 内的派生计算一并被编译器剪除，
 *    零运行时开销、对既有测试零影响。
 *
 * 验收事件类型（event= 字段）：detection_cell（SNR/SINR/Pd 与回波/噪声/干扰/杂波
 * 功率及处理增益，3.2.2.1.1.1–3.2.2.1.1.8）、beam_scan（波位排列表与扫描轨迹序列，
 * 3.2.2.1.2/3.2.2.4.2.1）、measurement（角距量测与误差采样，3.2.2.1.3）、track
 * （滤波值/预测值/协方差/关联结果与多目标航迹状态，3.2.2.2/3.2.2.3.1）、
 * recognition（四维特征量与类别/型号/置信度结论，3.2.2.3.3）、schedule（驻留计数
 * 与调度执行统计，3.2.2.4.2.2）。
 *
 * 现状（2026-08-20）：本头文件先落地宏与开关基础设施；调用点按验收输出统计清单
 * （docs/review/acceptance_output_inventory_2026-08-20.md）逐项接线，未接线前
 * 开启开关不产生任何输出。日志仅人读验收材料，不是机器契约（三写约束不变，
 * 见 docs/common/session_contract.md）。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_

#include "common/logging/ProjectLog.h"

#if defined(ONEQ_ENABLE_RIR_ACCEPTANCE_LOG) && ONEQ_ENABLE_RIR_ACCEPTANCE_LOG

#define RIR_ACCEPTANCE_LOG_ENABLED() true
// 字符串字面量拼接注入统一前缀；首参恒为格式串（编译期可查）。
#define RIR_ACCEPTANCE_LOG(...) PROJECT_LOG_INFO("[RirAccept] " __VA_ARGS__)

#else

#define RIR_ACCEPTANCE_LOG_ENABLED() false
#define RIR_ACCEPTANCE_LOG(...) ((void)0)

#endif

#endif  // ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_LOG_H_
