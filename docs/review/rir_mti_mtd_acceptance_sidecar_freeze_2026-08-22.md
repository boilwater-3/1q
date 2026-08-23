---
Status: draft
Date: 2026-08-22
Review-Baseline: `feature/acceptance-chinese-file-log`；规划 L1 验收旁路
Authority: L1 MTI/MTD 验收旁路冻结核。不替代模块 design/boundaries。
  与库实现冲突时以代码为准。本档不改检测主链。
---

# L1 MTI/MTD 验收旁路冻结（2026-08-22）

## 0. 结论

L1 只在验收旁路求值，写入 `rir_acceptance.log`，字段标明 **验收派生**，并写 **未进SINR**。
不写「经处理后」。不改 `DetectionCellResolver` 的 SINR，不改 CFAR 的 Pd / 检出，不改关联与航迹。
不新增公开 hardware / patch / replay 字段。关 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` 时宏与派生一并剪除。

常量核内写死：

- 通道数 N = 8
- 2 脉冲 MTI
- 杂波谱宽 σ_v = 0.25 m/s

## 1. 证据与判定

| Freeze item | 假设 | 判定 |
|---|---|---|
| 旁路不进主链 | 测试设计要通道数，检测链保持偏置账本 | pass（用户裁定） |
| 日志用语 | 「验收派生」≠「经处理后」 | pass |
| 常量不进公开配置 | 较小实现；主链不消费 | pass（用户裁定） |
| 不均分 8 路 | 通道功率来自 H(f) 与谱 | pass |
| IQ / CA-CFAR | 仍为非目标 | reject 实施 |

## 2. 公式

输入：`echo_power_w`、`thermal_noise_power_w`、`clutter_power_w`、`two_way_doppler_shift_hz`、
`prf_hz`、`center_frequency_hz`；可选干扰单音（链路多普勒 + 到达功率）。不算四偏置。

- 多普勒折合到 `[-PRF/2, PRF/2)`。
- 2 脉冲 MTI 功率响应：`|H(f)|² = 4 sin²(π f / PRF)`。零多普勒用地板，避免 log 炸掉。
- λ = c / 载频；`σ_f = 2 σ_v / λ`。杂波谱为中心 0 的高斯密度。
- MTD：8 点 DFT，第 k 路中心 `k·PRF/8`。幅度用 `1/N · sin(Nθ/2)/sin(θ/2)`，θ = 2π(f/PRF − k/N)。
  目标作单音；`|X_k(f_d)|²` 对 k 求和为 1。
- 目标第 k 路 = 回波 × `|H_mti(f_d)|²` × `|X_k(f_d)|²`。
- 噪声第 k 路 = 热噪声 × 2 / 8（MTI 噪声因子 2，白噪声均分）。
- 杂波：细网格上对 `|H_mti X_k|² S(f)` 求和，再按 `∫S` 与 `clutter_power_w` 归一。
- 干扰：有单音则每条当单音摊到 8 路；无单音则 `has_jam_channels=false`，日志写 `无`，不均分聚合瓦数。
- 选中路 = 目标功率最大的路（并列取最小 k）。
- 验收派生 MTI 增益 dB = `10 log10(|H_mti(f_d)|²)`。
- 验收派生 MTD 增益 dB = `10 log10(|X_sel(f_d)|²)`。
- 验收派生 MTI 剩余杂波 = `clutter_power_w · ∫|H_mti|² S / ∫S`。
- 验收派生 MTD 等效噪声 = 选中路噪声。

缺 cell、PRF≤0、载频≤0、核失败：派生字段写 `暂无` 加原因，不编数字。

主链「MTI后剩余杂波 = 杂波/10^(偏置/10)」一行保留，标明主链偏置。

## 3. 验收门

- common / RIR unit 测试通过。
- 关验收开关：RIR 单测与现网一致。
- 开开关日志含「验收派生」「未进SINR」，不含「经处理后」。
- Pd 与关开关同一场景相同。

## 4. 非目标

- 选中路 SINR 回灌 CFAR。
- 公开配置、IQ、CA-CFAR、距离多普勒图。
- 把功率均分 8 路冒充通道。

[evidence: docs/review/acceptance_item_catalog_2026-08-22.md]
[evidence: src/common/radar/DetectionCellResolver.cpp]
[evidence: docs/remote_identification_radar/algorithms.md]
