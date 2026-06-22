# SAR 确定性噪声与分布式杂波 SNR/SCR 二维矩阵工程契约

## 1. 目标

在不修改生产算法、public API、Session、replay 和默认成像路径的前提下，联合使用已
验收的确定性复高斯噪声与确定性分布式杂波测试能力，建立可重复、可比较、可审计的
SNR/SCR 二维参考矩阵。

本阶段只冻结测试支持层、矩阵和验收边界，不批准生产噪声、生产杂波、质量阈值、
警告、结构化拒绝或 Auto。

## 2. 独立分量与共同能量参考

每个联合场景由三份独立 raw history 分量组成：

- 纯目标分量 `target`
- 分布式杂波分量 `clutter`
- 复高斯噪声分量 `noise`

SNR 与 SCR 必须共同以纯目标 raw-history 总能量为参考：

$$
E_t = \sum_i |t_i|^2
$$

$$
E_n = \frac{E_t}{10^{SNR_{dB}/10}},\qquad
E_c = \frac{E_t}{10^{SCR_{dB}/10}}
$$

禁止使用 `target + clutter` 的能量缩放噪声，也禁止使用 `target + noise` 的能量缩放
杂波。否则 requested SNR 或 SCR 会被另一分量隐式改变。

最终联合输入定义为：

$$
x_i = t_i + c_i + n_i
$$

噪声与杂波均通过复数加法注入，因此在相同独立分量下，先加噪声或先加杂波必须得到
逐样本一致的最终输入。

## 3. Seed 与确定性

- 噪声 seed 与杂波 seed 必须独立配置并分别记录。
- 固定目标场景、SNR、SCR、噪声 seed、杂波 seed 和网格配置时，三份分量及最终输入
  必须逐样本一致。
- 只改变噪声 seed 时，目标与杂波分量必须保持不变。
- 只改变杂波 seed 时，目标与噪声分量必须保持不变。
- 禁止使用实现相关的标准库随机分布序列。

## 4. Diagnostics

每个联合场景至少记录：

- target energy
- noise energy
- clutter energy
- requested / realized SNR
- requested / realized SCR
- noise seed
- clutter seed
- scatterer count
- grid rows / columns
- clutter clipped pulse / target / sample count

realized SNR 与 SCR 必须由独立分量能量计算，不得由最终混合输入反推。

## 5. 注入位置与算法共享

最终联合输入只注入 raw pulse history，位于距离压缩、RDA、GBP 和 BP 之前。

同一联合场景下，RDA、GBP 与 BP 必须消费同一份最终 raw history。禁止为不同算法
分别生成噪声、杂波或最终输入。

比较继续复用统一小场景窗口、全局常数相位对齐、单位能量形状 NRMS 和相干相关系数。
BP 与 GBP 必须继续逐样本一致。

## 6. 首批矩阵

### 6.1 M1 完整二维矩阵

- 场景：M1 中心单点。
- 杂波网格：已验收的 `3x3` sparse。
- SNR：无噪声、`20 dB`、`0 dB`。
- SCR：无杂波、`20 dB`、`0 dB`。
- seed 对：至少两组独立的 `(noise_seed, clutter_seed)`。

该矩阵用于区分：

- 仅噪声退化。
- 仅杂波退化。
- 噪声与杂波共同退化。

### 6.2 M4 哨兵矩阵

- 场景：M4 二维分离多目标。
- 杂波网格：已验收的 `5x5` dense。
- 档位：无干扰基线、`20 dB SNR + 20 dB SCR`、`0 dB SNR + 0 dB SCR`。
- seed 对：至少两组。

M4 只验证多目标趋势和确定性，不扩展为完整笛卡尔积。

## 7. 比较口径

每个联合场景记录：

- RDA joint 相对 RDA clean 的 NRMS 与相干相关系数。
- GBP joint 相对 GBP clean 的 NRMS 与相干相关系数。
- RDA joint 相对 GBP joint 的 NRMS 与相干相关系数。
- requested / realized SNR 与 SCR。
- BP 与 GBP 逐样本一致性。
- 适用时，联合场景相对相同 SNR-only 与 SCR-only 场景的趋势。

首批审批只验证确定性、独立能量定义、注入顺序无关、算法共享输入和总体趋势。不得
冻结通用 SNR/SCR 二维质量阈值。

## 8. 验收门

1. 相同全部输入参数时，三份分量与最终输入逐样本一致。
2. 噪声 seed 与杂波 seed 的影响彼此隔离。
3. realized SNR 与 requested SNR 在数值容差内一致。
4. realized SCR 与 requested SCR 在数值容差内一致。
5. 噪声与杂波加法顺序不改变最终输入。
6. RDA、GBP 与 BP 消费同一份最终输入。
7. BP 与 GBP 在所有联合输入下继续逐样本一致。
8. 无噪声/无杂波基线严格保持现有参考结果。
9. 首批通过场景不得发生未记录的回波裁剪。
10. 默认与 Conan Eigen 3.3.9 SAR 回归、`sar_ci`、`sar_performance` 和 C++11 兼容门
    不回退。

## 9. 冻结边界

- 只修改 `tests/support`、测试、合同和验收文档。
- 不增加生产噪声或杂波模型、public 配置、runtime patch、Session diagnostics 或
  replay 字段。
- 不声明绝对功率、辐射定标、真实 clutter-to-noise ratio 或地物统计语义。
- 不实现相关杂波、随机散射点布局、海杂波、运动目标或真实斑点模型。
- 不启用质量阈值、警告、结构化拒绝、算法回退、尺寸扩展或 Auto。

## 10. 后续审批

实现与首批矩阵完成后，再单独决定：

1. 是否需要扩展中间 SNR/SCR 档位或更多场景。
2. 是否有证据研究相关杂波或随机位置分布。
3. 是否有证据形成联合输入质量有效性诊断。
4. 是否继续后置生产噪声、生产杂波和 Auto。

上述方向均不得由本契约自动批准。
