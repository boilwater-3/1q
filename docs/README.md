# 文档区索引

本目录收纳 1Q 仿真模型库的全部非代码文档，按用途分子目录。

## 目录结构

| 目录 | 用途 | 文档数 |
|---|---|---|
| [`public/`](public/) | 跨模块公共 API 手册（配置 / 输入 / 输出） | 3 |
| [`review/`](review/) | 模块评审（算法能力 / 开发过程） | 2 |
| [`sar/`](sar/) | SAR 模块全部文档（设计 · 合约 · 验收 · 决策 · 审计） | 141 |
| [`worklog/`](worklog/) | planning 工作日志（任务计划 / 进度 / 发现） | 3 |

## SAR 模块文档

SAR 模块文档归一到 [`sar/`](sar/)，按生命周期分层：

- [`sar/design/`](sar/design/) — 核心设计（4 份，权威入口）
- [`sar/contracts/`](sar/contracts/) — 子能力合约（44 份）
- [`sar/acceptance/`](sar/acceptance/) — 验收报告（51 份）
- [`sar/decisions/`](sar/decisions/) — 决策记录（32 份）
- [`sar/audits/`](sar/audits/) — 审计 / 研究 / 基线 / 收尾（10 份）

子能力「合约 / 验收 / 决策」三件套速查表见 [`sar/README.md`](sar/README.md)。

## 跨模块公共手册

- [`public/model_config_manual.md`](public/model_config_manual.md) — 模型配置手册
- [`public/module_output_manual.md`](public/module_output_manual.md) — 模块输出手册
- [`public/target_input_manual.md`](public/target_input_manual.md) — 目标输入手册

## 其它

- [`directory_restructure_plan.md`](directory_restructure_plan.md) — 2026-06-22 文档目录重组方案与执行记录
