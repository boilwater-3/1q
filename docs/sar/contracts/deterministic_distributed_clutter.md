# SAR 确定性分布式杂波参考模型工程契约

## 1. 目标

在不修改生产算法、public API、Session、replay 和默认成像路径的前提下，为 SAR
参考级成像闭环增加可重复、可比较、可审计的测试侧分布式杂波输入。

本阶段只建设测试支持层、参考矩阵和验收报告。该模型用于研究目标与分布式背景共同
存在时的成像趋势，不代表真实地物电磁散射、绝对功率、辐射定标或生产杂波能力。

## 2. 模型定义

分布式杂波由局部 Cartesian 成像平面上的确定性散射点集合表示。每个散射点复用现有
点目标 raw echo 生成链路，并具有：

- 唯一、稳定的网格索引。
- 明确的方位向与距离向局部坐标。
- 由固定 seed 和显式伪随机算法生成的复幅相。
- 在单次场景构造后保持不变的散射系数。

首批模型使用规则网格覆盖已批准的小场景输出窗口。散射点不得与参考目标位置重合，
不得越过 raw-history 可用距离范围，也不得产生未记录的回波裁剪。

禁止依赖 `std::uniform_*_distribution` 或 `std::normal_distribution` 的实现相关序列。
固定 seed、场景配置和网格遍历顺序必须生成逐样本一致的杂波 raw history。

## 3. 复幅相与能量归一化

每个散射点的候选复系数由显式固定 seed PRNG 产生，实部和虚部为零均值确定性序列。
候选系数集合必须去除样本均值，避免引入未声明的相干直流背景。

候选散射点通过现有点目标回波生成链路叠加为候选杂波 raw history。候选杂波随后按
总能量精确缩放，使目标信号与杂波能量比满足：

$$
SCR_{dB} = 10 \log_{10}\left(\frac{E_{target}}{E_{clutter}}\right)
$$

其中：

$$
E_{target} = \sum_i |s_i|^2,\qquad
E_{clutter} = \sum_i |c_i|^2
$$

目标 raw history 与杂波 raw history 分别生成，最终输入为两者逐样本相加。不得通过
修改参考目标 RCS 来间接实现 SCR。

输出 diagnostics 至少记录：

- target energy
- clutter energy
- requested SCR
- realized SCR
- seed
- scatterer count
- grid rows / columns
- clipped pulse / target / sample count

## 4. 注入位置与比较口径

杂波注入 raw pulse history，位于距离压缩、RDA、GBP 和 BP 之前。

同一场景、SCR 和 seed 下，RDA、GBP 与 BP 必须消费同一份 target-plus-clutter raw
history。禁止为不同算法分别重新生成杂波。

比较口径继续复用已批准的统一小场景窗口、全局常数相位对齐、单位能量形状 NRMS 和
相干相关系数。目标可见性只记录趋势，不在首批矩阵中冻结通用通过阈值。

## 5. 首批参考矩阵

场景：

- M1：中心单点目标。
- M4：二维分离多目标。

杂波布局：

- 规则二维网格。
- 至少两个网格密度档位，用于区分稀疏散射点集合与更接近分布式背景的趋势。
- 固定至少两个 seed，用于验证结论不依赖单一序列。

SCR 档位：

- 无杂波基线。
- `30 dB`
- `20 dB`
- `10 dB`
- `0 dB`

首批矩阵只评估确定性、能量定义、目标/杂波共同输入和总体退化趋势，不冻结通用 SCR
质量阈值。

## 6. 验收门

1. 相同场景、网格、SCR 和 seed 生成逐样本一致的杂波及混合 raw history。
2. 不同 seed 生成不同杂波 raw history。
3. realized SCR 与 requested SCR 在数值容差内一致。
4. 散射点位置、复系数和遍历顺序可由配置与 seed 完整复现。
5. RDA、GBP 和 BP 消费同一份混合 raw history。
6. BP 与 GBP 在相同混合输入下继续逐样本一致。
7. 无杂波基线严格保持现有参考矩阵结果。
8. 所有裁剪必须由 diagnostics 显式记录；首批通过场景不得发生裁剪。
9. 默认与 Conan Eigen 3.3.9 SAR 回归、`sar_ci`、`sar_performance` 和 C++11 兼容门不回退。

## 7. 冻结边界

- 只修改 `tests/support`、测试、合同和验收文档。
- 不增加生产杂波模型、public 配置、runtime patch、Session diagnostics 或 replay 字段。
- 不声明地物类型、后向散射系数、极化、入射角、斑点统计或绝对功率语义。
- 不声明辐射定标、RCS 反演或真实 clutter-to-noise ratio。
- 不启用 Auto、质量警告、结构化拒绝、算法回退或尺寸扩展。
- 不外推到时变 PRF、真实动力学、斜视、聚束、海杂波或运动目标。

## 8. 后续审批

实现与首批矩阵完成后，再单独决定：

1. 是否扩展随机散射点布局、相关杂波或更多网格密度。
2. 是否需要加入热噪声与杂波共同存在的 SNR/SCR 二维矩阵。
3. 是否有证据讨论生产杂波、绝对功率或辐射定标语义。
4. 是否需要形成目标可见性诊断或质量阈值。

上述方向均不得由本契约自动批准。
