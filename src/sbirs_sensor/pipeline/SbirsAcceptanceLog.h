/**
 * @file SbirsAcceptanceLog.h
 * @brief SBIRS 验收信息日志宏：把《红外载荷与远程识别雷达需求源码对应关系》3.2.1.3
 *        章节（OPIR 宽视场扫描探测与窄视场跟踪探测）要求的验收量按周期写入项目日志。
 *
 * 编译期由 CMake 开关 `ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG` 门控（默认 OFF）：
 *  - ON：`SBIRS_ACCEPTANCE_LOG(...)` 展开为 `PROJECT_LOG_INFO("[SbirsAccept] " ...)`，
 *    走既有 spdlog / 文件日志后端（1q_library.log），消息前缀 `[SbirsAccept]` 便于
 *    验收时 grep；格式串只允许 `{}` 与 `{:.Nf}` 两种占位符（文件后端迷你格式化约束）。
 *  - OFF：宏展开为空操作，`SBIRS_ACCEPTANCE_LOG_ENABLED()` 为 false，调用点外层
 *    `if (SBIRS_ACCEPTANCE_LOG_ENABLED()) { ... }` 内的派生计算（覆盖区投影等）
 *    一并被编译器剪除，零运行时开销、对既有测试零影响。
 *
 * 验收事件类型（event= 字段）：scan_footprint（覆盖区+驻留时间，3.2.1.3.1/3.2.1.3.2.2）、
 * wfov_candidate（疑似目标+信号能量+连续命中，3.2.1.3.2/3.2.1.3.2.1/3.2.1.3.2.2）、
 * wfov_hit_gate（命中数不足被挡，3.2.1.3.2.1）、nfov_acquisition（首次捕获判定，
 * 3.2.1.3.2.1）、nfov_track（跟踪状态+焦平面脱靶量，3.2.1.3.2.3）、nfov_schedule /
 * nfov_release（通道协同，3.2.1.3.2.4 传感器侧；威胁评分联合决策归决策层，不在此输出）。
 * 日志仅人读验收材料，不是机器契约（三写约束不变，见 docs/common/session_contract.md）。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_

#include "common/logging/ProjectLog.h"

#if defined(ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG) && ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG

#define SBIRS_ACCEPTANCE_LOG_ENABLED() true
// 字符串字面量拼接注入统一前缀；首参恒为格式串（编译期可查）。
#define SBIRS_ACCEPTANCE_LOG(...) PROJECT_LOG_INFO("[SbirsAccept] " __VA_ARGS__)

#else

#define SBIRS_ACCEPTANCE_LOG_ENABLED() false
#define SBIRS_ACCEPTANCE_LOG(...) ((void)0)

#endif

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_LOG_H_
