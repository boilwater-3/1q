---
Status: draft
Date: 2026-08-22
Review-Baseline: `feature/acceptance-chinese-file-log`；最窄 L2 极化验收旁路
Authority: L2 极化验收旁路冻结核。不替代模块 design/boundaries。
  与库实现冲突时以代码为准。本档不改识别主链。
---

# 最窄 L2 极化验收旁路冻结（2026-08-22）

## 0. 结论

L2 **只走验收旁路**，写入 `rir_acceptance.log`，字段标明 **验收派生**，并写 **未进识别**。
不写「经处理后」。不进提取器 / 匹配 / 模板 / 置信度。
`PolarizationFeatureExtractor`、`polarization_templates`、`RirPolarizationObservation` 三能量不变。
不加 IQ，不改识别库 schema，不把 S 写进公开量测。

通道约定：

- `channel_1` = HH，`channel_2` = VV
- **HV = VH**
- **φ_hv = φ_vh = 0**（本档冻死，不可配）

公开样本只加两量 + 两个显式开关（0 dBsm 合法，不能靠 0 当没填）：

- `cross_rcs_dbsm` + `has_cross_pol`
- `phase_vv_rel_hh_deg` + `has_phase_vv`

最近邻样本两个 `has_*` 都为真才构造 S 并写五字段；否则五字段一律
`暂无（无交叉极化或HH-VV相位，无法按S派生）`。
**不回退 L1** 的 √(σ1σ2) / 恒 0。现有 L1 对角实矩阵派生从验收行删除。

## 1. 证据与判定

| Freeze item | 假设 | 判定 |
|---|---|---|
| 旁路不进识别 | 测试设计要 S 派生量，识别仍用三能量 | pass（用户裁定） |
| 日志用语 | 「验收派生」≠「经处理后」；写「未进识别」 | pass |
| 0 dBsm 合法 | 必须 `has_*`，不得把 0 当缺省 | pass |
| 缺字段不回退 L1 | 五行同句暂无 | pass（用户裁定） |
| HV 相位可配 / IQ | 仍为非目标 | reject 实施 |

## 2. 公式

最近邻插值与 F1 相同（视角欧氏距离）。样本值直接用（真值辅助，不加 SNR 噪声底）。

```text
Shh = √σ_hh
Svv = √σ_vv · exp(j φ_vv)
Shv = Svh = √σ_hv
```

- 功率迹：`σ_hh + σ_vv + 2 σ_hv`（不是 `energy_sum_db`）
- 行列式：写 `|det(S)|`，`det = Shh Svv − Shv Svh`
- 去极化：`2 σ_hv / Span`（Span=0 则失败 → 暂无）
- 本征极化：Graves `G = SᴴS`，取最大特征值特征向量，ρ = V/H，
  `ψ = ½ atan2(2 Re ρ, 1−|ρ|²)`，`τ = ½ asin(2 Im ρ / (1+|ρ|²))`（度）

## 3. 数据流

`polarization_rcs_samples` 仍喂 F1 提取器（只读 ch1/ch2）。
同批样本另喂 `PolarizationAcceptanceS`，只写验收行。
控制器按 `external_target_id` 从本周期 `targets` 取样本传入 `WriteRirTrackAndId`。
极化差 / 相对 / 和仍写识别观测。

## 4. 验收门

- `1q_remote_identification_radar_unit_tests` + `unit::remote_identification_radar`
- 现有识别单测分数与关开关时一致（旁路不回灌）
- 未给 has_* 的场：五行 `暂无`，仍有极化差/相对/和
- 给齐 has_* 的场：五行是 S 公式结果，含「验收派生」「未进识别」，不含「经处理后」

## 5. 非目标

- 把 det / Span / ψ / τ 写入匹配或模板
- 扩 `RirPolarizationFeatureObservation` / replay 量测块
- 估 IQ 协方差（L3）
- 把 HV 相位做成可配
- 把缺字段回退成 L1 恒 0

[evidence: docs/review/acceptance_item_catalog_2026-08-22.md]
[evidence: src/remote_identification_radar/recognition/PolarizationFeatureExtractor.cpp]
[evidence: src/remote_identification_radar/runtime/PolarizationAcceptanceS.cpp]
