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
 * 验收事件类型（event= 字段，2026-08-20 已接线）：detection_cell（逐目标方向图
 * 离轴增益/波束宽度/离轴角、回波/热噪/干扰/杂波功率、脉压增益、SINR/SNR、Pd 与
 * 判决，3.2.2.1.1.1–3.2.2.1.1.8/3.2.2.1.2）、interference_link（逐干扰源到达
 * 接收端入射功率，3.2.2.1.1.3）、association/association_match/association_missed
 * （全局最优关联结果，3.2.2.3.1.1）、track（滤波航迹全量状态含 6×6 协方差，
 * 3.2.2.2/3.2.2.3.1）、measurement（四维特征量测全量字段，3.2.2.3.3.x）、
 * recognition（类别/型号/置信度结论，3.2.2.3.3）、schedule（驻留计数与识别效能
 * 摘要，3.2.2.4.2.2）、beam_pattern/beam_pattern_wave（波位排列表，一次性输出、
 * mission 配置变更后重发，3.2.2.4.2.1）、beam_scan（逐周期驻留波束中心与来源，
 * 3.2.2.1.2/3.2.2.4.2.1）。缺失子项（航向、舰船/车辆类型、MTI/MTD 通道级量等）
 * 经 2026-08-20 验收裁定不新增，说明见
 * docs/review/acceptance_output_inventory_2026-08-20.md §5/§6。
 *
 * 日志仅人读验收材料，不是机器契约（三写约束不变，
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
