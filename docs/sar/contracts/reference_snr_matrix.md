# SAR 确定性噪声与 SNR 鲁棒性参考矩阵契约

## 1. 目标

在不修改生产算法、public API、Session 或 replay 的前提下，为当前参考级成像闭环增加可重复的 raw pulse history 复高斯噪声与 SNR 鲁棒性证据。

本阶段只建设测试支持层、参考矩阵和审批报告，不生成 Auto、质量警告或结构化拒绝。

## 2. 确定性复噪声定义

输入为无噪声复数 raw pulse history。

1. 使用测试支持层自定义的固定 seed 伪随机生成器。
2. 禁止依赖 `std::normal_distribution` 的实现相关序列。
3. 通过 Box-Muller 或等价明确算法生成零均值复噪声候选。
4. 将候选噪声精确缩放到目标总能量：

$$
E_n = \frac{E_s}{10^{SNR_{dB}/10}}
$$

其中：

$$
E_s = \sum_i |s_i|^2
$$

5. 输出：
   - noisy raw pulse history
   - signal energy
   - noise energy
   - requested SNR
   - realized SNR

同一输入、SNR 和 seed 必须逐样本一致。空输入、零信号能量、非有限 SNR 或无效输出指针必须明确拒绝。

无噪声基线不调用噪声 helper。

## 3. 注入位置

噪声只注入 raw pulse history，位于距离压缩、RDA、GBP 和 BP 之前。

同一场景和 SNR 下，RDA、GBP 与 BP 必须消费同一 noisy raw history。禁止为不同算法分别生成噪声。

## 4. 首批矩阵

场景：

- M1 中心单点。
- M4 二维分离多目标。

SNR 档位：

- 无噪声基线。
- `30 dB`
- `20 dB`
- `10 dB`
- `0 dB`

固定至少两个 seed，用于验证：

- 同 seed 严格可重复。
- 不同 seed 的 noisy raw history 不同。
- 趋势结论不依赖单一 seed。

## 5. 比较口径

对每个 noisy 场景记录：

- RDA noisy 相对 RDA clean 的 NRMS 与相干相关系数。
- GBP noisy 相对 GBP clean 的 NRMS 与相干相关系数。
- RDA noisy 相对 GBP noisy 的 NRMS 与相干相关系数。
- RDA/GBP 图像质量指标与有效性。
- requested/realized raw-history SNR。
- BP 与 GBP 逐样本一致性。

首批审批只验证确定性、能量/SNR 定义和总体退化趋势。不得直接冻结通用 SNR 通过阈值。

## 6. 工程边界

- 只修改 `tests/support`、测试、合同和审批文档。
- 不增加生产噪声模型、杂波模型、辐射定标或绝对功率语义。
- 不增加 public 配置、runtime patch、Session diagnostics 或 replay 字段。
- 不扩大 RDA `1024x1024` 或 GBP/BP `128x128` 尺寸门。
- 不启用 Auto、算法回退、质量警告或结构化拒绝。
- 不外推到时变 PRF、真实动力学、斜视、聚束或分布式杂波。

## 7. 审批门

1. 固定 seed noisy raw history 逐样本一致。
2. 不同 seed noisy raw history 不同。
3. realized SNR 与 requested SNR 在数值容差内一致。
4. 同一 noisy raw history 被 RDA、GBP 和 BP 共同消费。
5. BP 与 GBP 在 noisy 输入下继续逐样本一致。
6. 双环境、C++11、SAR CI 与现有性能门不回退。
7. 审批报告明确记录低 SNR 失效或指标失去判别力的区域，不通过放宽阈值隐藏。

## 8. 后续决策

实现完成后再决定：

1. 是否需要增加更多 SNR 档位或场景。
2. 是否需要进入确定性分布式杂波参考模型。
3. 是否有证据形成输出质量有效性诊断。
4. Auto、public 噪声配置和生产噪声模型继续单独审批。
