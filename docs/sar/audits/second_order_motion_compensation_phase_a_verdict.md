# 二阶运动补偿 · 阶段 A 失效证据矩阵判定报告

Date: 2026-06-24
关联契约: `docs/sar/contracts/second_order_motion_compensation.md` §3
关联测试: `tests/unit/sar_second_order_motion_compensation_evidence_test.cpp`
状态: **阶段 A 完成,判定为不触发阶段 B(DO_NOT_TRIGGER_PHASE_B)**

## 1. 执行摘要

阶段 A 失效证据矩阵已按契约 §3.1 构建、运行并采集完整数据。**判定结果:不触发阶段 B。**

定量证据表明,L3 强转弯场景(横向偏移 ≥ 9 m)下的成像失效,**主因是非直线轨迹导致
RDA 聚焦假设崩溃,而非二阶空间变化残余相位**。这与 `l3_first_order_applicability_matrix.md`
第 44 行的结论("不应只归因于残余相位")定量一致。

依据契约 §3.2 末段:**二阶补偿不实现,改走 BP 路径(已就绪)。**

## 2. 矩阵配置(与契约 §3.1 一致)

| 项 | 值 |
|---|---|
| 场景 | 默认 9 脉冲参考场景(`ReferencePointScene` 默认值),9×9 GBP 成像网格 |
| 参考点 | `reference_delay = 20`(场景中心,固定) |
| 目标距离单元 | `{20(中心), 16(+4), 12(+8), 24(-4), 28(-8)}` |
| 孔径末端横向偏移 | `{6, 9, 12, 15, 18}` m |
| 一阶补偿参考点 | **固定为场景中心**(`reference_delay=20`),非目标自身 |
| 成像真值 | L3-GBP(逐像素对齐,GBP 方位像素数 = 脉冲数) |
| 比较 | RDA(一阶补偿后) vs L3-GBP,`CompareImagesWithGlobalPhaseReference` |

工况与 `l3_first_order_compensation_applicability_matrix` 测试完全一致(同一默认场景、
同一 `BuildTurningWaypointTrack`),满足契约 §5.1 验收 2"交叉核对"要求。

## 3. 关键证据:参考点自身的失效曲线

参考点(`target_delay = reference_delay = 20`)是二阶相位项严格为零的位置
(`φ₂ ≡ 0`,`spatial_residual ≡ 0`,已由不变量测试
`SecondOrderPhaseIsZeroWhenTargetEqualsReference` 验证)。因此参考点的成像质量
**完全不受二阶残余相位影响**,其失效只能归因于轨迹假设。

| 横向偏移 | 参考点 NRMS | 参考点相干 | 通过门?(NRMS<0.25 且 相干>0.97) |
|---|---|---|---|
| 6 m | 0.177 | 0.984 | ✅ 通过 |
| 9 m | 0.273 | 0.963 | ❌ 失效 |
| 12 m | 0.386 | 0.925 | ❌ 失效 |
| 15 m | 0.521 | 0.864 | ❌ 失效 |
| 18 m | 0.668 | 0.777 | ❌ 失效 |

**结论**:在二阶残余相位恒为零的参考点处,一阶补偿从 9 m 起就失效。这决定性地
排除了"残余相位是主要失效来源"的假设 —— 失效在残余相位完全不存在时就已经发生。

## 4. 关键证据:偏离参考点目标并非一致恶化

若残余相位是主因,则偏离参考点的目标应**一致地**比参考点差。实际数据(摘录):

| 横向偏移 | 目标 | NRMS | 相对参考点(delay=20) |
|---|---|---|---|
| 6 m | delay=24 (-4) | 0.144 | **优于** 参考点 0.177 |
| 6 m | delay=28 (-8) | 0.121 | **优于** 参考点 0.177 |
| 12 m | delay=24 (-4) | 0.304 | 优于 delay=16(+4)的 0.525 |
| 18 m | delay=24 (-4) | 0.514 | 远优于 delay=12(+8)的 1.360 |

偏离参考点的目标在某些距离方向上**反而成像更好**,这与"空间变化残余相位主导失效"
的假设直接矛盾。NRMS 的差异更可能源于 RDA 多普勒参数对不同距离的敏感度差异,
而非二阶残余。

## 5. 关键证据:空间变化残余斜距误差量级

绝大多数单元的空间变化残余斜距误差 `max_spatial_residual_range_m < 0.007 m`,
远小于相位门(NRMS=0.25 对应的等效距离误差为厘米级)。仅 18 m / delay=12(+8)的
极端单元达到 0.40 m,但该单元 NRMS=1.36 已是灾难性失效,残余不是唯一因素。

## 6. 判定(契约 §3.2)

| 准则 | 要求 | 结果 | 满足? |
|---|---|---|---|
| 1 | 存在失效档位(NRMS>0.25) | 9 m 起失效 | ✅ |
| 2 | 残余相位占主导(>50%) | 参考点零残余下已失效,主因是轨迹假设 | ❌ |
| 3 | 残余斜距随偏离单调增长 | 存在空间变化 | ✅ |

**准则 2 不满足** → `phase_a_verdict = DO_NOT_TRIGGER_PHASE_B`。

测试输出(RecordProperty):
```
criterion1_any_failure_band            = 1
criterion2_residual_phase_dominates    = 0
criterion2_reference_point_passes_all_bands = 0
criterion2_attribution = FAILURE_DOMINATED_BY_TRAJECTORY_ASSUMPTION_NOT_RESIDUAL_PHASE_DO_NOT_TRIGGER
criterion3_spatial_variation           = 1
phase_a_verdict = DO_NOT_TRIGGER_PHASE_B
```

## 7. 结论与后续动作

1. **二阶运动补偿不实现**(依据契约 §3.2 + §6 冻结边界:阶段 A 不通过则阶段 B 永不执行)。
2. L3 强转弯场景(横向偏移 ≥ 9 m)**应走 BP 路径**(GBP/BP 已就绪,`BpAndGbpProduceIdenticalL1AndL3Images` 测试验证 BP 与 GBP 在 L3 下一致),BP 不依赖 RDA 的平移不变聚焦假设。
3. 本判定**不关闭**二阶运动补偿冻结项的理论价值 —— 若未来 RDA 引入轨迹自适应聚焦(补偿多普勒中心/调频率漂移)后再失效,可重开阶段 A。
4. 契约 `second_order_motion_compensation.md` 状态更新为"阶段 A 完成,阶段 B 否决,证据封存"。

## 8. 复现命令

```bash
cmake --build --preset llvm-ninja-release --target 1q_unit_tests -j 8
./build/llvm-ninja-release/bin/1q_unit_tests \
  --gtest_filter='SecondOrderMotionCompensationEvidenceTest.*' \
  --gtest_output=xml:/tmp/2ndmc_evidence.xml
```
