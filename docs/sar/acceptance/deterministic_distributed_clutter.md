# SAR 确定性分布式杂波参考模型验收报告

## 1. 验收结论

测试侧确定性分布式杂波 helper 与首批 M1/M4 参考矩阵完成当前平台验收。

- 相同场景、网格、SCR 和 seed 的杂波及混合 raw history 逐样本一致。
- 不同 seed 产生不同杂波 realization。
- 杂波按 raw-history 总能量精确缩放，realized SCR 与 requested SCR 一致。
- RDA、GBP 与 BP 共同消费同一混合 raw history。
- BP 与 GBP 在所有首批杂波场景中继续逐样本一致。
- 首批网格没有产生回波裁剪。
- 本实现只位于测试支持层，不增加生产杂波、public API、Session 或 replay 能力。

## 2. 实现范围

新增测试支持能力：

- `ReferenceClutterGridConfig`
- `ReferenceClutterDiagnostics`
- `BuildDeterministicDistributedClutter`

杂波散射点使用规则二维网格，逐散射点复用现有点目标 raw echo 生成链路，再乘固定
seed 生成的零均值确定性复系数并叠加。候选杂波 raw history 按 requested SCR 精确缩放
后与目标 raw history 相加。

## 3. 首批矩阵

- 场景：M1 中心单点、M4 二维分离多目标。
- 网格：`3x3` sparse、`5x5` dense。
- seed：`17/29`。
- SCR：`30/20/10/0 dB`。
- 算法：RDA、GBP、BP，共同消费同一混合 raw history。

## 4. 关键结果

| 场景与配置 | 30 dB RDA/clean NRMS | 0 dB RDA/clean NRMS | 30 dB GBP/clean NRMS | 0 dB GBP/clean NRMS |
|---|---:|---:|---:|---:|
| M1 sparse seed 17 | 0.014621 | 0.619501 | 0.014217 | 0.606510 |
| M4 dense seed 29 | 0.019897 | 0.362255 | 0.019850 | 0.361540 |

首批场景中，`0 dB` 相对 `30 dB` 均表现出明显更高的 clean-reference NRMS。该结果只
用于证明确定性输入和总体退化趋势，不构成通用 SCR 质量阈值或算法失效结论。

## 5. 冻结边界

- 不增加生产杂波、绝对功率、辐射定标、地物散射或斑点统计语义。
- 不增加 public 配置、runtime patch、Session diagnostics 或 replay 字段。
- 不启用 Auto、质量警告、结构化拒绝、算法回退或尺寸扩展。
- 不把规则网格结果外推到海杂波、相关杂波、时变 PRF、真实动力学或运动目标。

## 6. 后续决策

下一阶段应单独判断：

1. 是否需要扩展随机位置、相关杂波或更多密度档位。
2. 是否需要建立噪声与杂波共同存在的 SNR/SCR 二维矩阵。
3. 是否有证据讨论目标可见性诊断或生产杂波语义。

在后续决策完成前，不批准通用 SCR 阈值、生产杂波或 Auto。
