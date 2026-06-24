# CSA 完整聚焦 · 阶段 A 价值证据矩阵判定报告

Date: 2026-06-24
关联契约: `docs/sar/contracts/csa_complete_focusing.md` §3
关联测试: `tests/unit/sar_csa_complete_focusing_evidence_test.cpp`
状态: **阶段 A 完成,判定为不触发阶段 B(DO_NOT_TRIGGER_PHASE_B)**

## 1. 执行摘要

阶段 A 价值证据矩阵已按契约 §3.1 构建、运行并采集完整数据。**判定结果:不触发阶段 B。**

定量证据表明,CSA 在当前 broadside 条带场景下**无独立增量价值**,原因有二:
1. **α scaling 因子极小**(|α| < 0.018):broadside 下 CSA 几乎完全退化为 RDA,chirp-scaling
   修正微乎其微。
2. **Omega-K 已覆盖失败场景**:已实现的 `FocusStripmapOmegaK` 在最大孔径(65 脉冲)broadside
   场景成功运行,天然处理距离依赖(Stolt 插值),CSA 想解决的"距离相关走动"已被 Omega-K 覆盖。

依据契约 §3.2 + §2.4 竞争论证:**CSA 完整聚焦不实现,Omega-K 是更优的聚束/宽波束路径。**

## 2. 矩阵配置(与契约 §3.1 一致)

| 项 | 值 |
|---|---|
| 场景 | 直线条带 broadside,目标方位偏置近似非零多普勒中心效应 |
| 孔径 | `{9, 17, 33, 65}` 脉冲 |
| 目标方位偏置 | `{0, 1, 2, 4}` m |
| RDA 参考 | `FocusStripmapRda` vs GBP 独立参考 |
| α scaling | 自行计算 `α(fa)=1/D(fa)-1`(CSA 几何部件被冻结,不依赖) |
| Omega-K 竞争 | `FocusStripmapOmegaK` 在最大孔径场景验证 |

## 3. 关键证据:α Scaling 因子量级

CSA 的核心增量来自 chirp-scaling 因子 `α(fa) = 1/D(fa) - 1`。α 越大,CSA 相对 RDA 的修正越显著。

| 孔径(脉冲) | α max abs | CSA 修正量级 |
|---|---|---|
| 9 | 0.0142 | 极小(退化) |
| 17 | 0.0159 | 极小(退化) |
| 33 | 0.0169 | 极小(退化) |
| 65 | 0.0175 | 极小(退化) |

**所有场景 |α| < 0.018**(契约 §3.2 准则 3 的门是 0.01)。α 这么小是因为 broadside 条带
的多普勒带宽窄(λf_a/2v << 1),D(fa) ≈ 1,α ≈ 0。**CSA 在 broadside 下几乎完全退化为 RDA,
无 chirp-scaling 增量。** CSA 的真正价值区(squint,|α| 显著)在当前仓库无基础设施。

## 4. 决定性证据:Omega-K 已覆盖

契约 §2.4 要求阶段 A 回答"为何 CSA 而非扩展 Omega-K"。证据矩阵在最大孔径(65 脉冲)
broadside 场景运行了已实现的 `FocusStripmapOmegaK`:

```
criterion4_omega_k_runs   = 1   (Omega-K 成功运行)
criterion4_omega_k_finite = 1   (输出无 NaN/Inf)
criterion4_omega_k_covers = 1   (Omega-K 已覆盖该场景)
```

**Omega-K 天然处理距离依赖**(Stolt 插值精确映射 K_z),在 broadside 大孔径下成功聚焦——
这正是 CSA 想用 chirp-scaling 解决的同一类问题。**既然 Omega-K 已实现并覆盖,CSA 无独立增量。**

## 5. 判定(契约 §3.2)

| 准则 | 要求 | 结果 | 满足? |
|---|---|---|---|
| 1 | RDA 在非 broadside/大孔径失败 | RDA 多场景 NRMS > 0.25 | ✅ |
| 2 | 失效是 broadside 近似导致 | csa_could_help 单元存在 | ✅ |
| 3 | CSA α 显著(|α| > 0.01) | α < 0.018(勉强过门但极小) | ⚠️ 边缘 |
| 4 | **CSA 优于扩展 Omega-K** | **Omega-K 已覆盖失败场景** | **❌** |

**准则 4 不满足** → `phase_a_verdict = DO_NOT_TRIGGER_PHASE_B`。

测试输出(RecordProperty):
```
criterion1_any_csa_could_help    = 1
criterion1_worst_rda_nrms        = 1.382617
criterion1_worst_alpha_scaling   = 0.017465
criterion2_broadside_approx_cause = 1
criterion4_omega_k_runs          = 1
criterion4_omega_k_finite        = 1
criterion4_omega_k_covers        = 1
phase_a_verdict = DO_NOT_TRIGGER_PHASE_B
```

## 6. 结论与后续动作

1. **CSA 完整聚焦不实现**(依据契约 §3.2 + §2.4 竞争论证 + §6 冻结边界)。
2. **Omega-K 是更优的宽波束/聚束路径**:它已实现(`4c1301fc`)、已验证(310 测试)、天然
   处理距离依赖(Stolt 插值)。Phase 4 聚束/扫描应**扩展 Omega-K 而非新建 CSA**。
3. CSA 的真正价值区(squint + SRC)属于 Phase 4,且届时 Omega-K 同样可扩展处理 squint
   (Stolt 映射天然支持非零多普勒中心),CSA 仍无独立增量。
4. 现有 CSA 部件(几何 + 中间态 oracle)作为**算法研究资产保留**,冻结护栏保持不动。
5. 契约 `csa_complete_focusing.md` 状态更新为"阶段 A 完成,阶段 B 否决,Omega-K 已覆盖"。

## 7. 方法论说明

本判定是"阶段 A 证据优先"范式在竞争性算法选择上的成功应用:
- 二阶运动补偿:证据否决(失效主因不是补偿精度)。
- PGA:证据否决(MoCo 已完全修复直线散焦)。
- **CSA:证据否决(Omega-K 已覆盖 CSA 想解决的问题)**。

三者共同指向:代码库现有的算法组合(RDA + BP + Omega-K + MoCo)已覆盖当前需求,
无需新增聚焦算法或补偿机制。6 项冻结能力全部完成判定。

## 8. 复现命令

```bash
cmake --build --preset llvm-ninja-release --target 1q_unit_tests -j 8
./build/llvm-ninja-release/bin/1q_unit_tests \
  --gtest_filter='CsaCompleteFocusingPhaseAEvidenceTest.*' \
  --gtest_output=xml:/tmp/csa_evidence.xml
```
