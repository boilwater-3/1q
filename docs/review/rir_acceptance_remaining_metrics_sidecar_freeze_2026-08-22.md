---
Status: draft
Date: 2026-08-22
Review-Baseline: `feature/acceptance-chinese-file-log`；剩余原文指标验收旁路
Authority: 抑制比 / 检测门限 / 事件列表 / 波位全文冻结核。不替代模块 design/boundaries。
  不进探测 SINR、Pd、关联与调度器。
---

# 剩余原文指标验收旁路冻结（2026-08-22）

## 0. 结论

下列原文指标必须在 `rir_acceptance.log` 出现，标明 **验收派生**。允许公式糙，**不**回灌检测 / 调度。

## 1. 抑制比（复用 MTI/MTD 核）

- MTD 杂波抑制比 = `10 log10(clutter_in / Σ clutter_w)`
- MTD 干扰抑制比 = `10 log10(jam_in / Σ jam_w)`
- 总干扰抑制增益 = `10 log10(jam_in / mti_residual_jam_w)`

`jam_in` 为本周期入射链路到达功率之和（与核单音同一批）。输入或剩余非正则 `暂无`；无干扰单音写 `无`。主链偏置行保留，不顶替「处理后」。

## 2. 统计检测门限

`RadarEquations::ComputeThreshold(pfa, N)`，`N` = `effective_pulse_count`。
写「未进判决」。不改 `DetectResolvedCell`，不做 CA-CFAR。Pfa 非法或无 cell 写 `暂无`。

## 3. 事件执行列表

`[搜索×Ns,跟踪×Nt,识别×Ni]`，计数与现「调度策略」三字段相同。无分类配额。

## 4. 波位排列表 / 扫描轨迹

`BuildAbsoluteScanWaves` 整表写入 `rir_scan_pattern.csv`（`index,az_deg,el_deg`）。
日志写文件路径、本周期序号=`(cycle-1)%N`、下一波位。指定任务时加「本周期指向=指定，表为扫描序列」。不改实际驻留中心。

[evidence: src/common/radar/MtiMtdAcceptanceBank.cpp]
[evidence: src/common/radar/RadarEquations.cpp]
[evidence: src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp]
