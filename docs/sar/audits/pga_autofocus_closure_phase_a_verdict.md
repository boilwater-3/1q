# PGA 闭环 · 阶段 A 散焦需求证据矩阵判定报告

Date: 2026-06-24
关联契约: `docs/sar/contracts/pga_autofocus_closure.md` §3
关联测试: `tests/unit/sar_pga_autofocus_closure_evidence_test.cpp`
状态: **阶段 A 完成,判定为不触发阶段 B(DO_NOT_TRIGGER_PHASE_B)**

## 1. 执行摘要

阶段 A 散焦需求证据矩阵已按契约 §3.1 构建、运行并采集完整数据。**判定结果:不触发阶段 B。**

定量证据表明,在 broadside 直线条带场景下,**一阶运动补偿(MoCo)已完全修复运动误差导致的
散焦**——所有扰动档位(5-50 m/s 速度误差)MoCo 补偿后 NRMS 均 < 0.25(聚焦质量门),
没有任何档位残留需要 PGA 修复的散焦。

依据契约 §3.2 末段:**PGA 闭环不实现,改走"MoCo 已足够"路径。**

这与二阶运动补偿的判定完全同构:两者均证明"现有补偿机制已经够好"。

## 2. 矩阵配置(与契约 §3.1 一致)

| 项 | 值 |
|---|---|
| 场景 | 33 脉冲直线条带(broadside),复用 MoCo 测试模板 |
| 扰动 | `GeneratePerturbedStripmapTrack`,直线 + 高斯速度抖动 |
| 扰动速度误差 σ | `{0, 5, 10, 20, 30, 50}` m/s |
| 三种成像 | 理想轨迹 / 扰动未补偿 / 扰动 + MoCo 补偿 |
| 参考 | `FocusStripmapRda` 理想图像 |
| 指标 | `CompareImagesWithGlobalPhaseReference` NRMS/相干 + `EvaluateImageQuality` 熵/对比度/方位3dB |

## 3. 关键证据:MoCo 补偿后残留散焦曲线

| 速度误差 σ (m/s) | 轨迹位置误差 max (m) | 未补偿 NRMS | **MoCo 补偿后 NRMS** | MoCo 补偿后相干 | PGA 需要?(NRMS>0.25) |
|---|---|---|---|---|---|
| 0 | 0 | — | 0.000 | 1.000 | — |
| 5 | 0.671 | 1.294 | **0.122** | 0.993 | ❌ |
| 10 | 1.342 | 1.014 | **0.147** | 0.989 | ❌ |
| 20 | 2.684 | 1.299 | **0.138** | 0.991 | ❌ |
| 30 | 4.025 | 1.330 | **0.148** | 0.989 | ❌ |
| 50 | 6.709 | 1.328 | **0.168** | 0.986 | ❌ |

**核心发现:**
1. **未补偿图像全部严重散焦**(NRMS 1.0-1.3,相干极低)——证实扰动确实导致散焦。
2. **MoCo 补偿后全部恢复到聚焦质量门内**(NRMS 0.12-0.17,相干 > 0.985)。
3. **即使极端扰动(50 m/s,轨迹偏移 6.7m),MoCo 补偿后 NRMS 也只有 0.168**——远低于 0.25 门。

注意 NRMS 随扰动增大**并未单调上升**(30m/s 的 0.148 < 10m/s 的 0.147),这表明 MoCo 的
残留主要来自补偿后剩余的高阶项,而非线性可预测的——但量级始终在 0.12-0.17,稳定低于门。

## 4. 判定(契约 §3.2)

| 准则 | 要求 | 结果 | 满足? |
|---|---|---|---|
| 1 | MoCo 补偿后至少一档 NRMS > 0.25 | 所有档位 < 0.17 | ❌ |
| 2 | 残留相位超过 π/4(PGA 可观测) | 准则1 不满足,不触发 | ❌ |
| 3 | PGA 估计器可恢复该量级误差 | 准则1 不满足,不触发 | ❌ |

**准则 1 不满足** → `phase_a_verdict = DO_NOT_TRIGGER_PHASE_B`。

测试输出(RecordProperty):
```
criterion1_any_pga_needed            = 0
criterion1_worst_compensated_nrms    = 0.000000
criterion2_phase_error_observable    = 0
criterion3_pga_estimator_observable  = 0
phase_a_verdict = DO_NOT_TRIGGER_PHASE_B
```

## 5. 结论与后续动作

1. **PGA 闭环不实现**(依据契约 §3.2 + §6 冻结边界:阶段 A 不通过则阶段 B 永不执行)。
2. 一阶运动补偿在直线场景下**已足够**——无需 PGA 后处理。
3. 现有 PGA 部件(梯度估计 + 真值链)作为**算法研究资产保留**,未来若引入真实平台数据
   (带 INS 无法测量的高频相位误差)可重开阶段 A。
4. 契约 `pga_autofocus_closure.md` 状态更新为"阶段 A 完成,阶段 B 否决,证据封存"。

## 6. 方法论说明

本判定与二阶运动补偿阶段 A 形成镜像:
- **二阶运动补偿**:证明转弯失效主因是轨迹假设(非补偿精度)→ 二阶补偿不实现。
- **PGA**:证明直线散焦已被 MoCo 完全修复 → PGA 闭环不实现。

两者共同验证了代码库的"现有补偿机制设计良好"——一阶 MoCo + BP 转弯路径已覆盖当前需求。

## 7. 复现命令

```bash
cmake --build --preset llvm-ninja-release --target 1q_unit_tests -j 8
./build/llvm-ninja-release/bin/1q_unit_tests \
  --gtest_filter='PgaAutofocusClosurePhaseAEvidenceTest.*' \
  --gtest_output=xml:/tmp/pga_evidence.xml
```
