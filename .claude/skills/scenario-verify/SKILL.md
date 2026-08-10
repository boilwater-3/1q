---
name: scenario-verify
description: Scenario-driven verification of 1Q simulation library modules through the examples/component_attachment integration demo. Use whenever the user wants to 拟定/设计战场场景, 用场景验证模块正确性, 写预期事件表, or debug why 探测不上/没有检测/融合不对/指令没下发 — deciding whether a failure is a library bug, a scenario design problem, or a wrong expectation. Covers scene JSON authoring (scenes/), expectation tables, log triage (integration_events.log / integration_views.log / 1q_library.log), and regression closure. Also use when iterating scenario suites to verify AR / ESR / EOS / SBIRS / SAR / fusion / flight behavior end-to-end.
---

# Scenario Verify — 场景驱动的集成级验证工作流

## 定位与边界

用**场景**（战场原型 → 可判定几何）驱动 `examples/component_attachment` 演示，验证
1Q 库模块在**集成链路**（探测 → 融合 → 决策）上的正确性。与既有资产的分工：

- `tests/consumer/batch_validation`（230 场景）验证**库级单会话**参数扫描/跨周期序列——本
  skill 的"库问题最小复现"复用其模式，但不重复其工作；
- 本 skill 验证 batch_validation 够不到的**集成级**：多通道同时驱动、融合聚合、
  决策事件链、飞行/目标几何耦合。

验证结论三分类：**库问题 / 场景设计问题 / 预期表错误**。每个结论必须有日志证据
（引用具体行），不允许凭感觉下结论。

## 前置条件

- `examples/component_attachment` demo 已构建（release preset；场景验证用
  `llvm-ninja-release-local`，FD 开关按场景需要，见步骤 5）；
- 场景文件目录 `examples/component_attachment/scenes/`（git 跟踪，场景即配置记录）；
- 日志三文件：`integration_events.log`（事件行）/ `integration_views.log`（视图行）/
  `1q_library.log`（库内部日志）——每场景独立输出目录（`--output-dir`）。

## 工作流

### 1. 确认被测通道与场景意图

- 读 `examples/component_attachment/README.md`（场景设计/周期语义/已知行为记载）、
  `scenes/` 现有场景、`docs/common/contract.md` 相关章节；
- **声明三要素**（写进预期表，缺一不可）：
  - 被测通道：AR / ESR / EOS / SBIRS / SAR / Fusion / Flight 之一或组合；
  - 被测行为：探测成立 / 跟踪生命周期 / 分选聚簇 / 成像窗口 / 融合关联 / 决策链 /
    机动编排等具体行为；
  - 验证深度：L1 冒烟（事件存在）/ L2 预期表（周期窗+量级+计数）/ L3 物理一致性
    （测量值对几何先验）。

### 2. 拟定场景（先选原型，再裁剪几何）

- 从 `references/scenario_archetypes.md` 选原型，或自建；**战场场景只取材、不完整
  复刻**——真实战场多因素耦合，失败时无法归因，必须裁剪成"单变量可判定"的几何；
- 难度阶梯：先单通道 + 简单几何（基线探测成立）→ 叠加机动/多目标 → 叠加多通道 →
  再谈环境效应；每次只改一个变量；
- **边界条件检查**（写预期表前必做）：被测通道的 `docs/<module>/boundaries.md` veto
  阈值——EOS 距离窗 ≈ 高度/sin(俯仰角)、SAR squint ≤ 门限、SBIRS 目标在 FOV 内、
  ESR 中心频率分离、AR 径向速度/距离门；
- 写场景 JSON：`scenes/<name>.json`（schema 见 README「场景描述文件」节；几何字段
  必填、调参字段可省——缺省值 = 基线行为）。场景文件即**配置记录**，随预期表归档；
- 手算**几何先验**作为独立预期（例：目标纬度 ≈ 平台纬度 + range/111 km；EOS 距离窗
  ≈ 高度/sin(min/max 俯仰)；SBIRS 星下点 el = asin(z/r)）——用于 L3 核对。

### 3. 写预期事件表（必填，场景可判定的关键）

- 模板见 `references/expectation_template.md`（含基线试跑样本）；
- 每个通道一行：行为 + **预期周期窗** + **量级** + **计数**；
- 标注**日志模式可观测性**：默认 KEY 事件模式只落关键事件，ESR 假设（DUP）不落盘、
  SAR 持续类事件不落盘——预期表注明"仅 ALL/AGGREGATE 模式可观测"，避免把模式门控
  误判为事件缺失；
- 区分**按设计拒绝**（SAR squint 拒绝、EOS 扫描间隙导致的"首发现→丢失"交替、SBIRS
  视场外）与**意外失败**——按设计拒绝写进预期表作为"预期出现"项；
- 零产出场景（如无目标）显式把场景 `smoke` 块下限置 0，否则 ctest 冒烟会红。

### 4. 参数配置

- 场景 JSON 覆盖的层：平台飞行脚本 / 目标脚本 / ESR 波形 / 天基平台 / EOS 扫描 /
  SAR 任务几何与链路 / 融合配置 / 决策门限 / 冒烟下限；
- 会话基线配置（`examples/configs/*.json`）一般不动；场景级的业务调参进场景文件
  （`eos_scan`/`sar` 块经 `ApplySceneOverrides` 应用）；
- 日志模式（编译期宏，见 `demo_log_modes.h`）：默认 `delta + key`；需要全量事件时
  `-DCA_EVENT_LOG_MODE=all`（重 configure + 编译一次）；预期表核对适合 `summary` 视图。
  模式切换记录在预期表"运行配置"栏。

### 5. 运行与采集

```bash
# 构建（场景文件改动无需重编译；代码改动才需要）
cmake --build --preset llvm-ninja-release-local --target component_attachment_demo
# 运行（每场景独立输出目录；--cycles 可缩短 triage 迭代）
./build/llvm-ninja-release-local/bin/component_attachment_demo \
    --scene examples/component_attachment/scenes/<name>.json \
    --output-dir /tmp/1q/scenes/<name>
```

- **确定性**：场景内种子固定（ESR `timing_seed`、SBIRS/SAR JSON 种子）→ 同场景
  同输出，可重复取证；
- **FD 模式**：默认验证模式 FD 关（运动学回退，几何干净可手算先验）；FD 专项
  （起飞/转弯/机动）再开 `ONEQ_ENABLE_FLIGHT_DYNAMIC`——注意 FD 起飞段平台向西北
  爬升（README 记载 hdg 293°→358°），EOS 探测 cycle 101 才成立，属预期；
- 单跑 demo（约 20 s），不要全量 ctest 并行（饿死超时是环境负载问题，非回归）。

### 6. 日志验证与三分类判定

按 `references/triage_guide.md` 的决策树逐级排查，每级有明确判据：

1. **库日志先行**：`1q_library.log` 的 `PROJECT_LOG_ERROR/WARNING`——库内错误多数
   会说人话（如 `squint_angle_exceeds_limit`）。先判断是"按设计拒绝"还是真错误；
2. **边界条件核对**：通道无探测且库日志正常 → 核对场景几何是否满足
   `boundaries.md` veto 阈值——多数"探测不上"是几何不满足（场景设计问题），
   不是库 bug；
3. **预期表自身复查**：手算先验与预期矛盾（人写的预期也会错）；
4. **库问题**：库日志异常或 边界满足仍行为异常 → 用 `batch_validation` 风格最小
   复现（单会话、固定参数）定性，再决定修复；
5. 每个判定附**日志证据**（文件 + 行内容 + 周期号）。

### 7. 闭环与回归

- 判定为**库问题** → 最小复现 → 修复 → 补回归（单元测试或 batch_validation
  sequence）→ **重跑本场景确认**；高风险模块（SAR/ESR/EOS/radar/flight-dynamic）
  的修复先走 `evidence-first-freeze-contract`（证据矩阵），改动后走
  `completeness-review` 门禁；
- 判定为**场景问题/预期问题** → 改场景/改预期表 → 重跑；
- **场景通过 → 归档**：场景 JSON + 填好的预期表（含结论列）随代码提交；场景集成为
  回归资产——库改动后重跑全部场景（逐个单跑，勿并行）；
- 场景发现的行为新事实（如某通道在特定几何下的生命周期语义）→ 回写模块
  `algorithms.md` 的 `[evidence: ...]`。

### 8. 停止条件

- 判定收敛到三分类之一，且有日志证据引用；
- 场景通过并归档（预期表结论列=通过）；
- 无法判定的悬挂项 → 记入 `docs/common/open_questions.md`（走
  `open-questions-doc-standard`），不阻塞其余场景。

## 命令速查

```bash
# 单测（场景加载器/组件层）
./build/llvm-ninja-release-local/bin/1q_examples_unit_tests --gtest_filter='SceneDataTest.*'

# 冒烟（ctest，模式无关断言）
ctest --preset llvm-ninja-release-local -R "examples::component_attachment_demo"

# 日志模式切换（重 configure + 编译）
cmake --preset llvm-ninja-release-local -DCA_EVENT_LOG_MODE=all -DCA_VIEW_LOG_MODE=summary

# 常用统计（预期表核对）
grep -c "事件:eos_detection" log/<name>/integration_events.log
grep "视图:sar" log/<name>/integration_views.log | grep -c "L1图像=有"   # 成像周期数
grep -oP "fused=\d+" log/<name>/console.txt | sort -t= -k2 -rn | head -1  # 融合峰值
```

## 参考文档

- [references/scenario_archetypes.md](references/scenario_archetypes.md) — 场景原型库
  （拦截/机动/饱和/无目标等：通道组合、被测行为、边界条件检查清单）
- [references/expectation_template.md](references/expectation_template.md) — 预期事件表
  模板 + 基线场景填好的试跑样本
- [references/triage_guide.md](references/triage_guide.md) — 症状 → 日志 → 判定规则表
- `examples/component_attachment/README.md` — demo 结构、场景设计记载、日志模式说明
- `docs/common/contract.md` — 跨模块契约（运行期配置提交策略等）
- 模块边界阈值：`docs/<module>/boundaries.md`（veto 量化阈值）
