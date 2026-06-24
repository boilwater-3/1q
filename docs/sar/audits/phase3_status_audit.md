# SAR Phase 3 现状审计报告

Date: 2026-06-23

## 1. 审计目的

`module_design.md` 历史把 Phase 3(BP 增强 + 运动补偿二阶 + 杂波建模深化)标为
`🟡 部分(BP 合并形式,运动补偿仅一阶)`。经代码库与契约文档检索,该标记与实际实现
状态存在显著偏差。本审计如实记录三项的实际完成情况与冻结边界,作为 v2.4 文档状态
修正的可审计依据。

本审计不修改任何代码,不重开任何冻结项,不构成新的实现审批。

## 2. 审计结论

Phase 3 三项中,**两项已实质闭环,一项被审计刻意冻结**:

| Phase 3 子项 | 实际代码状态 | 真实缺口 | 处置 |
|---|---|---|---|
| BP 增强 | ✅ 已完整落地(含 public Session 全链路) | 无实质缺口 | 视为闭环 |
| 杂波建模深化 | ✅ 生产 + 测试层均已闭环 | 仅设计层明确冻结的"生产级分布式杂波/绝对功率/地物语义" | 视为闭环(冻结项除外) |
| 二阶运动补偿 | ❌ 完全空白(零实现) | 唯一真空白,但被 5+ 份审计明确列为后置 | 保持冻结 |

因此 Phase 3 不应继续表述为"BP 合并形式,运动补偿仅一阶"——该描述未反映 BP/杂波
已闭环的事实,也混淆了"未实现"与"有意冻结后置"两种性质。

## 3. BP 增强 — 已闭环

### 代码证据

- `src/sar/imaging/SarGbp.cpp`:GBP 与 BP 共享 `FocusSmallSceneBackprojection`
  内核,通过 `BackprojectionTraversal{kPixelMajor/kPulseMajor}` 区分遍历顺序;
  `FocusSmallSceneGbp` / `FocusSmallSceneBp` 为两个公开入口;`kMaxApprovedDimension=128`。
- `src/sar/session/SarImagingExecutor.cpp::ExecuteL3BpImaging`:L3 BP public 执行路径。
- `src/sar/session/SarRuntimeConfigValidation.cpp`:`l3_bp_size_gate` 结构化拒绝
  (range/azimuth 上限 128),`invalid_l3_bp_config` 配置校验。
- `src/sar/session/SarReplayFlatbufferCodec.cpp` + `SarTraceSession.cpp`:
  `enable_l3_bp_imaging` / `has_l3_bp_image` / `kL3BpImage` 进入 replay 与 trace。

### 公共契约证据

- `include/1q/sar/config/SarPolicyConfig.h::enable_l3_bp_imaging`(默认 false)。
- `include/1q/sar/config/SarMissionConfig.h::l3_waypoints`(LLA,内部转 local Cartesian)。
- `include/1q/sar/session/SarCycleResult.h::SarProcessingStage::kL3BpImage` /
  `has_l3_bp_image`。

### 契约与验收

- 契约:`docs/sar/contracts/l3_bp_session_integration.md`(受控 public Session 接入,
  含互斥/尺寸门/replay 边界)。
- 验收:`docs/sar/acceptance/l3_bp_session_integration.md`、`l3_bp.md`。
- 边界:无并行/GPU、无时变 PRF public 调度、不扩大 128 上限(均按 Phase 5/后置处理)。

### 结论

BP 增强已超出"仅合并形式",实际包含共享内核 + public Session 接入 + replay/trace
闭环。本子项视为闭环。

## 4. 杂波建模深化 — 已闭环(冻结项除外)

### 代码证据

- 生产层 `src/sar/echo/SarEcho.cpp`:
  - `GammaClutterRcs`:`σ = γ·sin(θ_inc)·A_cell`。
  - `SeaClutterRcs`:GIT 经验模型简化版(海况/风速/入射角)。
  - `GenerateClutterScene`:点目标 + 规则网格杂波单元逐个叠加,复用 `ApplyFractionalDelay`。
- 测试层 `tests/support/sar_reference_scene.h`:
  - `BuildDeterministicDistributedClutter`:确定性规则网格分布式杂波 + 精确 SCR 缩放。
  - `BuildDeterministicJointInterference`:噪声 + 杂波联合矩阵(SNR+SCR)。
- 测试 `tests/unit/sar_reference_scenario_matrix_test.cpp`:
  `SarReferenceClutter*` / `SarReferenceSnrScrMatrixTest` 已覆盖 M1/M4 × 3x3/5x5 ×
  seed × SCR {30/20/10/0 dB} × RDA/GBP/BP。

### 契约与验收

- 契约:`docs/sar/contracts/deterministic_distributed_clutter.md`。
- 验收:`docs/sar/acceptance/deterministic_distributed_clutter.md`。

### 冻结边界(设计层明确不做,非遗漏)

- 真生产级分布式杂波(绝对功率/地物散射系数/极化/斑点统计)。
- 辐射定标、RCS 反演、真实 CNR。
- 海杂波/相关杂波/时变 PRF/运动目标扩展。

### 结论

在已批准范围内,杂波建模深化已闭环。超出范围的生产级杂波能力被设计层冻结,
不在 Phase 3 完成度判定之内。

## 5. 二阶运动补偿 — 真空白,且被审计刻意冻结

### 代码证据

- `grep -ril SecondOrder` / `second_order` 在 `src/`、`include/` 下**零命中**。
- `src/sar/imaging/SarMotionCompensation.{h,cpp}` 仅含 `ApplyFirstOrderMotionCompensation`。

### 审计冻结证据(非疏漏)

至少 5 份审计/验收文档明确把二阶补偿列为后置:

- `docs/sar/audits/l3_first_order_compensation.md:35`:不批准二阶补偿。
- `docs/sar/audits/l3_first_order_applicability_matrix.md:44`:
  "不直接实现二阶补偿;当前失效同时涉及非直线轨迹聚焦假设,**不应只归因于残余相位**。"
- `docs/sar/audits/l3_imaging_degradation_baseline.md:44`:
  "只有一阶补偿不足的证据成立后,才讨论 BP 或二阶补偿。"
- `docs/sar/audits/phase2_reference_closure.md:57`:
  "时变 PRF、全图 replay、二阶补偿、多参考点补偿和 Auto 均继续后置。"
- `docs/sar/contracts/l2_motion_compensation.md`、`l2_session_integration.md`、
  `l3_bp_session_integration.md` 均列"二阶补偿继续后置"。

### 工程判断依据

`l3_first_order_applicability_matrix.md` 扫描结果:

| 孔径末端横向偏移 | 一阶补偿后 NRMS | 相干相关系数 | 当前门 |
|---:|---:|---:|---|
| 0 m | 0.042218 | 0.999109 | 通过 |
| 6 m | 0.176972 | 0.984341 | 通过 |
| 12 m | 0.386100 | 0.925463 | 失败 |

12 m 失效区存在,但审计结论是该失效不能只归因于残余相位(还涉及非直线轨迹聚焦
假设),因此二阶补偿缺乏充分证据支持立即实现。

### 结论

二阶运动补偿是 Phase 3 唯一的真空白,但当前所有审计证据都指向"保持冻结"。
恢复前必须:

1. 先开新契约 `docs/sar/contracts/second_order_motion_compensation.md`,
   冻结残余相位误差模型、二阶项数学定义、参考点策略、验收场景(须在 12 m 失效区
   证明改善且改善归因可隔离)、与一阶/Auto/public 的边界。
2. 契约审批后再实现。

禁止直接实现以突破现有冻结决策。

## 6. 文档状态修正(本审计唯一落地动作)

`docs/sar/design/module_design.md` v2.4 已据本审计修正:

- Phase 3 行由 `🟡 部分(BP 合并形式,运动补偿仅一阶)` 改为反映实际状态:
  `🟡 部分(BP 增强与杂波深化已落地;二阶运动补偿按审计结论有意冻结后置)`。
- 运动补偿章节明确二阶/高阶补偿为审计冻结后置项而非疏漏,指向
  `l3_first_order_applicability_matrix.md` 与 `l3_first_order_compensation.md`。

## 7. 本审计的非目标

- 不重开二阶运动补偿审批。
- 不修改任何源代码。
- 不变更冻结清单(Auto/CSA/Omega-K/辐射定标继续冻结)。
- 不推断到 Phase 4(聚束/扫描/多视)或 Phase 5(OpenMP/GPU/实时)。
- 不构成通用质量阈值或算法选择授权。
