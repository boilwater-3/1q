---
Status: final
Date: 2026-08-15
Completed: 2026-08-21（迁移主体 + 阶段 3/3b common 化收口）
Review-Baseline: `main` @ `96de367c`（merge: SBIRS 2026-08 正式设计变更与审计修复落地）
Authority: 审计已完成使命，转为历史判定记录；装备关系裁定（远程识别雷达为独立装备）
  仍被 `docs/common/open_questions.md` 与 RIR design.md 引用。不得替代各模块
  `design.md`/`boundaries.md`；若本文与库实现冲突，以库为准。
Related-Authority:
  - 迁移唯一状态文档（含逐阶段落地 commit）：`remote_identification_radar_migration_status_2026-08-15.md`
---

# 机载雷达（AR）模块归属审计：远程识别雷达耦合识别与解耦边界（已收口）

## 0. 定位与结论

> **后续状态（迁移已全部执行）**：审计清单已由 RIR 迁移阶段 1 / 2-M / 2-T / 2-S / 2-C
> 与 common 化阶段 3 / 3b 全部落地，逐阶段 commit 见迁移状态文档 §1。
> 拆分验收标准（原 §9）已达成（见 §3）。本文正文收口为判定记录，
> 实施前的文件级清单与 file:line 证据见 git 历史（基线 `96de367c`）。

审计范围：AR 的 public 头、src、schemas、tools、tests、examples、docs 中与识别
相关的全部内容。

结论：AR 模块内混入的"非机载雷达"功能**只有一块**——远程识别雷达子系统（kLrr）。
未发现 ESR/SAR/SBIRS/EOS 或其他雷达功能侵入。

**装备关系前提（已确认，仍为现行裁定）**：远程识别雷达是与机载雷达相互独立的
另一部雷达装备，不是 AR 的工作模式或子能力。审计当时的 AR 文档把 kLrr 定位为
"AR 的并行输出子能力"，该定位被本审计推翻并已在拆分中修正。

## 1. 边界判定标准（三条）

1. **装备前提**：独立装备——自带硬件配置域、自管波束、独立输入输出、独立
   replay/trace；与 AR 只存在"航迹供给"这一模块间接口（阶段 2-S 后连该接口
   也已删除，RIR 完全自持）。
2. **库内证据信号**：模块解剖完整度（"全套器官、长在别人身上"即寄生耦合）、
   数据流独立性（识别是后挂平行链，不进 AR 主链决策）、物理口径异质性
   （识别专用高保真观测路径与探测链口径分裂）。
3. **功能归属清单**：探测/关联/滤波/调度/决策归 AR；驻留观测、四维特征提取、
   多周期积累、模板匹配、特征库管理归识别雷达。

## 2. 三分类处置与落地

| 分类 | 内容 | 处置 | 落地 |
|---|---|---|---|
| 一：纯属识别雷达 | `src/airborne_radar/recognition/` 17 文件、`ArRecognition*` public 类型、特征库 DDL/建库工具/交付库、7 个识别测试 | 整体迁出，`ArRecognition*` 前缀废弃（一次性，无兼容层） | 迁移阶段 1 + 2-C（`1ac346ca`） |
| 二：AR 本体耦合点 | `ArWorkMode::kLrr`、policy 第七子域、场景真值三字段、航迹快照/结果帧识别字段、Controller/Session/Codec 识别段、replay fbs 识别表、SQLite 链接、文档四章节 | 全部删除/归还；replay 字节兼容断裂按审计建议一次性接受 | 2-C（`1ac346ca`） |
| 三：AR 本体 | 探测链、关联/航迹、决策、环境、输出、干扰观察、trace/replay 基础设施 | 保持不动 | 无动作 |

三条接缝的处置决策（现行有效）：

| 接缝 | 处置 |
|---|---|
| 波束（Path A） | AR 全删（kLrr 指向三件套 + passthrough 分支）；优先航迹选择逻辑迁入新模块波束调度；`ArWorkMode` 值域收紧为 kStby/kTas/kTws/kStt |
| 硬件 | 新模块自带 hardware 域；链路预算自持（可经 `src/common/` 共享雷达方程，不引 AR internal） |
| 航迹供给 | 原设计"消费 AR 公开航迹输出"；阶段 2-S 裁定 AR/RIR 完全独立后该接缝整体退役，RIR 自持检测-跟踪 |

命名与治理（迁移前定稿，已执行）：模块 `remote_identification_radar`、public 前缀
`Rir*`、issue code 前缀 `rir.*`；`check_public_api_boundary.cmake` 白名单同步收敛。

## 3. 拆分验收标准执行结果（原 §9）

| 验收条 | 结果 |
|---|---|
| AR 全域识别关键词 grep 零命中（include/src/schemas/tests/examples） | ✅ 2-C 验收记录 |
| `airborne_engine` 不再链接 SQLite3；`ArWorkMode` 值域与 replay 上界收紧 | ✅ `1ac346ca` |
| AR public 白名单/配置/快照/结果帧/issue code 无识别残留 | ✅ |
| AR 测试套件删除识别段后全绿；7 个识别测试迁入新模块全绿 | ✅（RIR 分区 115/115 等，见迁移状态 §1） |
| 新模块具备全套标准解剖（四域配置/会话/validation/replay/trace/assets/tests/docs） | ✅ |
| public_api_boundary 与 cross_domain_naming 守护通过 | ✅ |
| AR 文档四篇移除 kLrr 章节；examples 与 i18n 收敛 | ✅ |

## 4. 审计方法

以代码/构建/测试/资产为证据，不以文档声明代替；`Recognition`/`kLrr`/
`aspect_rcs` 等关键词全域检索，`surveillance`/`esm`/`passive` 反查确认无其他
雷达功能侵入。行号引用以审计基线 `96de367c` 为准（收口后不再维护）。
