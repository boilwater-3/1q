---
Status: draft
Date: 2026-08-15
Baseline: `feature/remote-identification-radar-phase1` @ `67947078`
Authority: RIR 迁移唯一状态与下一步迁移计算文档
Supersedes:
  - `docs/review/ar_remote_identification_decoupling_phase1_plan_2026-08-15.md`
  - `docs/review/remote_identification_radar_phase2_plan_2026-08-15.md`
Related-Authority:
  - AR 侧耦合审计：`docs/review/ar_remote_identification_radar_coupling_audit_2026-08-15.md`
  - 九项信号链能力归属：`docs/review/rir_signal_chain_capability_boundary_2026-08-15.md`
---

# RIR 迁移状态与下一步迁移计算（唯一迁移文档）

本文件是当前分支关于 RIR 迁移的**唯一状态与计划文档**。历史阶段计划已归档删除，
后续所有迁移决策、范围变更与执行进度只在此文件登记。

## 0. 结论

1. RIR 已完成从“消费 AR 航迹供给的识别后处理模块”到“自持检测 → 跟踪 →
   识别的独立雷达模块”的切换；`RirTrackFeed` 公共供给面与 AR↔RIR 等价性
   测试已删除。
2. 跟踪能力升级（突破 2-T 轻量边界，N1-N7）**已完成**：LAPJV 全局最优
   关联、航迹池 + 复用代次、IMM 双路径（confirmed 命中激活）全部落地；
   public policy `enable_imm_lifecycle`/`model_count_hint` 已接线。
3. “全天候”不做内部天气模型迁移；天气衰减继续由调用方以
   `RirEnvironmentSnapshot.weather_attenuation_db` 输入字段提供。
4. 跟踪波束/成波作为独立议题暂缓，不在当前迁移计算内展开。

## 1. 当前迁移状态

| 段 | 内容 | 状态 | 提交/说明 |
|---|---|---|---|
| 阶段 1 | RIR 模块骨架、public 类型、识别链路、replay 编解码、AR↔RIR 等价性验证 | 完成 | 已被阶段 2-S 破坏性替换供给面；等价性测试已删除 |
| 2-M | 检测链与环境子集迁移 | 完成 | M1 `e6e073b9`、M2 `87bd2ae1`、M3 `99b53792`、M4 `f28cb874`、M5 `950625ed`、M6 `e66d5293` |
| 2-T | 轻量跟踪子集 | 完成 | T4 `6fa38268`/`2020f739`、T1 `0b3e8d70`、T2 `79bef1d6`、T3 `ce995bda` |
| 2-S | 自持化重构 | 完成 | 核心 `c09dd870`、四件套文档 `7809e5d6`、进度登记 `c38d59f9`、注释清理 `1d7f6eb5` |
| 2-C | AR 侧识别耦合收尾删除 | 完成 | `1ac346ca`（按耦合审计 §3/§9 全清单：识别实现目录/public 头/Controller-Session-Pipeline 胶合/replay fbs 表与 codec/测试 SQLite 接线/文档四件套与 i18n 收敛；`ArWorkMode` 值域收紧为 kStby/kTas/kTws/kStt，replay 工作模式上界 kStt，`airborne_engine` 解除 SQLite 链接；replay 字节兼容断裂按审计 §7.1 接受，一次性无兼容层） |
| 跟踪升级 N1-N7 | LAPJV/航迹池/IMM（突破 2-T 轻量边界） | 完成 | N1+N2 `46e495dd`（RirLapjvSolver + 方阵代价矩阵全局最优指派）；N3 `93bd3a4f`（RirTrackPool + generation 复用代次）；N4+N5 `6bfca646`（RirImmFilter 包装 common ImmFilter + 生命周期双路径）；N6+N7 public policy IMM 开关接线 + 四件套文档改写 |
| 阶段 3 | common 化收敛与四域归位 | 完成 | 四域归位见 §5；common 化（LAPJV/雷达方程/方向图）已完成并落 `src/common/`，评估见 `common_consolidation_assessment_2026-08-15.md`，执行计划见 `common_consolidation_execution_plan_2026-08-15.md` |

当前验证基线：
- `unit::remote_identification_radar` 115/115
- `integration::remote_identification_radar` 29/29
- `replay::remote_identification_radar` 3/3
- `integration::cross_domain` 6/6（等价性测试已删除）

## 2. 架构决策修订（2026-08-15）

| 编号 | 决策 | 内容 |
|---|---|---|
| D-A1 | 独立输入面 | 保持：场景目标 + RF 入射链路 + 环境快照 + 平台状态 |
| D-A2 | 自持链路 | 保持：检测 → 关联/滤波/生命周期 → 内部航迹 → 识别积累 |
| D-A3 | `RirTrackFeed` 退役 | 已执行，零 public 残留 |
| D-A4（修订） | **突破 2-T 轻量边界** | 原“单目标 KF + 门限最近邻 + 计数生命周期”升级为“多目标全局最优关联（LAPJV）+ 航迹池 + KF/IMM 多模型 + 计数生命周期”；战斗级之外的战术决策、ECCM、对外点迹/航迹输出仍否决 |
| D-A5 | 驻留排序 | 保持：未识别优先 + 斜距次近 |
| D-A6 | `target_name` 真值统计 | 已执行 |
| D-A7 | 场景目标补速度/Swerling | 已执行 |
| D-A8（新增） | 天气衰减口径 | 调用方提供 `weather_attenuation_db`；RIR 不迁/不自研天气物理模型，只叠加到传播损耗 |
| D-A9（新增） | 跟踪波束/成波 | 暂缓议题，不进入当前迁移计算；未来需先确定“驻留波束调度”与“beamforming 效能级口径”边界 |

## 3. 下一步迁移计算：跟踪能力升级（2-T 边界突破）

### 3.1 目标

- 多目标跟踪由“最近邻唯一分配”升级为**全局最优关联**；
- 航迹对象由 `std::map` 值语义升级为**航迹池 + 复用代次**；
- 单模型 CV KF 升级为 **CV KF / IMM 双路径**；
- 现有识别消费闭包字段（`association_key/status/hit_count/位置/速度/加速度/uncertainty`）不变。

### 3.2 迁移项与 AR 来源

| 步 | 内容 | AR 来源 | 落点 |
|---|---|---|---|
| N1 | LAPJV 全局最优指派 | `signal/association/LapjvSolver.*` | `src/remote_identification_radar/tracking/RirLapjvSolver.*` |
| N2 | 关联引擎改造成本矩阵 + LAPJV | `DataAssociation.cpp` 的 `cost_matrix + assignment_solver_` 路径 | `RirTrackAssociator` 增加 `SolveAssignment`，保留现有马氏门限/动态 R 门控；未分配门限沿用 `distance_gate_sigma` 映射 |
| N3 | 航迹池与键回收语义 | `ITrackPool.h`、`BoostTrackPool.*`、`TrackLifecycleManager` 的 `Acquire/Release/ResetForReuse` | `RirTrackPool.*`、`RirTrackLifecycle` 池化 + `generation` |
| N4 | IMM 多模型 | `common/estimation/ImmFilter.h`（AR 仅 facade，不可引 AR 头） | `RirImmFilter` 包装 `ImmFilter<6,3>`；默认双模型（高/低过程噪声 CV） |
| N5 | 生命周期 IMM 接线 | `TrackLifecycleManager` 的 `ShouldUseImmForMeasurement/Miss`、`GetOrCreateImmFilter` | `RirTrackLifecycle` 双路径；lost 重捕获仍重置 |
| N6 | 策略/配置/校验/replay | AR `LifecycleConfig::enable_imm_lifecycle/model_count_hint`、`TrackingConfig` | `RirPolicyConfig` 增 IMM 开关/模型数提示；配置校验与 replay 状态同步 |
| N7 | 测试迁移 | `ar_signal_association_test`、`ar_track_lifecycle_test`、IMM 测试 | `rir_track_associator_test`、`rir_track_lifecycle_test`、新增 `rir_track_pool_test`、`rir_imm_tracking_test` |

### 3.3 实施顺序与验证边界

1. N1+N2：先落地 LAPJV 并保持现有关联 API；验证多目标全局最优分配、门控拒绝、键单调不回收。
2. N3：航迹池接入生命周期；验证复用 `track_id` 递增、`generation` 递增、重复释放拒绝。
3. N4+N5：IMM 路径接入；验证高机动目标切换后位置/速度连续、未启用时与 CV KF 数值一致。
4. N6+N7：策略与 replay 闭合；更新场景测试与文档四件套。

每步验收：
- `unit::remote_identification_radar`
- `integration::remote_identification_radar`
- `replay::remote_identification_radar`
- `integration::cross_domain`（当前为多模型场景）
- 公共头/契约 guards 不回归

## 4. 天气衰减：调用方输入口径

- 已存在字段：`RirEnvironmentSnapshot::weather_attenuation_db`。
- 控制器已消费：`has_environment_data == true` 时叠加到
  `RirPropagationModel` 输出的传播损耗。
- 下一步无迁移动作；只补测试：
  - 正衰减降低 SNR/检测；
  - 0 衰减与不提供环境数据路径可区分；
  - 负值/非有限值被 `rir.validation.invalid_environment_snapshot` 拒绝。

## 5. 后续段（不在本次跟踪能力升级内展开）

- **2-C AR 侧收尾**：已完成（见 §1；AR 全域识别 grep 零命中、AR/RIR/cross_domain 分区与
  全部 guards 绿为验收证据）。
- **阶段 3 common 化**：已完成。
  - **LAPJV / 雷达方程 / 天线方向图**已收敛到 `src/common/`：
    - `src/common/optimization/LapjvSolver.{h,cpp}`
    - `src/common/radar/RadarEquations.{h,cpp}`
    - `src/common/radar/AntennaPatternRuntime.h`
  - AR/RIR 保留薄适配层，模块内类名/函数名不变。
  - 检测单元/CFAR 判决/传播环境/航迹池不动或缓。
  - 执行步骤见 `docs/review/common_consolidation_execution_plan_2026-08-15.md`。
  - `max_range_m`/`recognition_dwell_sec` 四域归位已执行：字段从
    `policy.recognition` 移至 `mission` 域（`RirMissionConfig::max_range_m` /
    `recognition_dwell_sec`），控制器消费与配置校验同步更新。
- **暂缓议题**：跟踪波束/成波、再入目标专项物理模型（气动/等离子/RCS 剖面）、
  全天候天气物理模型。

## 6. 验收标准

1. RIR include 闭包无 AR 头，公共面无 `RirTrackFeed*` 残留。
2. 多目标关联为 LAPJV 全局最优；门限拒绝与未分配语义正确。
3. 航迹池复用与 `generation` 语义正确，识别积累不因池复用错配目标。
4. IMM 未启用路径与现有 CV KF 数值一致；启用路径高机动场景测试通过。
5. 天气衰减调用方输入路径有测试与文档证据。
6. 全部 RIR 分区 + public API/header guards 通过；release `/completeness-review` 通过。
