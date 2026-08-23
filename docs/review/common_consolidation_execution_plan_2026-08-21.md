---
Status: final
Date: 2026-08-21
Completed: 2026-08-21（`e702de90` refactor(common): extract AR/RIR shared detection and track kernels）
Authority: 阶段 3b common 化执行计划（大气/有效 RCS/检测单元账本/CFAR 编排/冻结波束/航迹池·关联核·生命周期计数）——已执行完毕
Related-Authority:
  - 审核决策：`ar_rir_shared_capability_extract_audit_2026-08-21.md`
  - 评估（#4/#5/#7 已推翻）：`common_consolidation_assessment_2026-08-15.md`
  - 阶段 3 执行完成记录：`common_consolidation_execution_plan_2026-08-15.md`
  - 迁移状态：`remote_identification_radar_migration_status_2026-08-15.md`
---

# AR/RIR common 化执行计划（阶段 3b）

## 0. 目标

将现实映射审核判定为「可直接提取」的双源收敛到 `src/common/`：

1. 逐目标大气损耗标量胶水
2. 有效 RCS 视角/频率混合
3. 检测单元 SINR 账本（含 `anti_rgpo_leading_edge` bool 钩子）
4. 统计级 CFAR 编排壳
5. 波束冻结 Resolve
6. 航迹池模板 + 关联代价/指派核 + PromoteState 计数 FSM

「可提取核心」发射/接收本轮不改。植被杂波内核已在 common，无代码动作。
RIR 6 dB 真值回退门、AR 挂架/驻留叠加、AR 反欺骗整段留模块侧。

## 1. 落点

| 切片 | 落点 | AR 适配 | RIR 适配 |
|---|---|---|---|
| 大气 | `common/atmosphere` 标量胶水 | `DetectionExecution` | `RirController` |
| RCS 混合 | `common/rcs` POD + mix | `DetectionExecution` | `RirEffectiveRcs` |
| 检测单元 | `common/radar/DetectionCellResolver.*` | `ArDetectionCellResolver` | `RirDetectionCellResolver` |
| CFAR 编排 | `common/radar/StatisticalCfarDetector.*` | `SignalDetector` | `RirSignalDetector` |
| 冻结波束 | `common/radar` frozen resolve | `BeamControlResolver::ResolveFrozen` | `RirResolveBeamStateForPointing` |
| 航迹栈 | `common/tracking/` | BoostTrackPool / DataAssociation 核 / lifecycle FSM | RirTrackPool / Associator / Lifecycle |

## 2. 接口约束

- common **仅标量 / 通用 POD / 模板 T**；禁止模块 config 类型。
- 已知语义差显式参数化：
  - 账本：`anti_rgpo_leading_edge: bool`（RIR 恒 false）
  - RCS：`carrier_hz <= 0` 回退策略留模块侧
- 缺省行为与收敛前逐位一致（等价回归）。

## 3. 验收

- 聚焦单测：`unit::airborne_radar` / `unit::remote_identification_radar` / `unit::common` 相关目标绿
- common 无 `airborne_radar` / `remote_identification_radar` include
- algorithms.md evidence 指向 common 单源

## 4. 完成记录（2026-08-21）

- 六个切片（大气标量胶水 / 有效 RCS 混合 / 检测单元账本含 `anti_rgpo_leading_edge` 钩子 /
  统计级 CFAR 编排壳 / 波束冻结 Resolve / 航迹池模板+关联核+PromoteState 计数 FSM）
  已全部落 `src/common/`，AR/RIR 薄适配层切换：`e702de90`。
- 「可提取核心」发射/接收按计划未改；RIR 6 dB 真值回退门、AR 挂架/驻留叠加、
  AR 反欺骗整段留模块侧。
- 等价回归门与验收三条（聚焦单测绿 / common 无模块 include / algorithms.md 指向单源）
  随 `e702de90` 达成；逐项验证数字见该提交与两侧模块 algorithms.md 登记表。
