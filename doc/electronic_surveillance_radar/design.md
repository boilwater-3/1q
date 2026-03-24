能用的有，但大多是“算法胚子能提纯”，不是这份实现可以直接搬。

先给结论：`ElecReconProcess` 里真正值得保留的，不是类设计和流程编排，而是几段可以拆成纯函数的小算法，以及两三个可重写成独立 solver 的数值过程。文件在 [ElecReconProcess.cpp](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp) 和 [ElecReconProcess.h](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.h)。

**可以提纯复用的算法**

- 频段归类。`JudgeWaveType` 按载频映射到 P/L/S/C/X/Ku/K/Ka/V/W 等波段，这类表驱动规则可以保留，但应改成枚举 + 边界表，不要保留魔法整数。见 [ElecReconProcess.cpp:611](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L611C1)。

- 扫描波束排布。`BeamArrange` 的核心思想是根据扫描起点、扫描方向、方位/俯仰步进生成 beam 序列，这对被动扇区扫描是有用的。实现很土，但“扫描栅格生成器”这个算法可保留。见 [ElecReconProcess.cpp:358](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L358C1)。

- 视场/可见性筛选链。多个流程都在做同一件事：几何变换到观测坐标系，判断波束内、频段覆盖、地平线/通视是否满足，再进入链路预算。这套判定顺序是对的，适合抽成 `InterceptGate`。代表位置见 [ElecReconProcess.cpp:915](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L915C1)、[ElecReconProcess.cpp:1572](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1572C1)。

- 干扰预计算。`PreJamPowerCal` 的思路是先把当前接收指向下的 jammer 贡献累加成接收端干扰功率，再给后续检测流程复用。这是合理的工程优化，也符合侦察机理。见 [ElecReconProcess.cpp:779](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L779C1)。

- 链路预算驱动的截获判定。这里反复使用 `FrequencyCover -> propagation loss -> ReceiveJamPowerCalculate -> SNR` 这条链，虽然函数名和上下文很乱，但“用接收功率预算做截获概率/门限判定”的主思路是能用的。代表位置见 [ElecReconProcess.cpp:929](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L929C1)、[ElecReconProcess.cpp:1583](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1583C1)。

- 概率检测抽样。`ThresholdDetector(Pd)` 就是一个 Bernoulli 抽样器，概念上没问题，应该保留为统一的检测随机化接口，只是现在用 `rand()` 太差。见 [ElecReconProcess.cpp:1199](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1199C1)。

- 测角误差模型。`AngleDiff = 0.51 * BeamWidth / sqrt(SNR_linear) * N(0,1)` 这种“角误差标准差随波束宽度增大、随 SNR 增大而减小”的模型，作为任务级仿真近似是值得保留的。需要重写成返回标准差和采样值分离的纯函数。见 [ElecReconProcess.cpp:1207](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1207C1)。

- 功率加权测向融合。`BeamAdvance` 里把同一波束内多个辐射源的方位/俯仰按接收功率加权平均，这个想法可以保留，用来生成“波束中心下的等效观测”。见 [ElecReconProcess.cpp:1485](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1485C1)。

- 基于二分法的覆盖边界求解。`ScopeAdvance2D2` 和 `ScopeAdvance3D` 本质上都在做“满足通视 + 满足截获门限的最远边界”搜索，算法上是可用的。这个很适合抽成独立的 `BoundarySearchSolver`。见 [ElecReconProcess.cpp:993](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L993C1)、[ElecReconProcess.cpp:1096](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1096C1)。

**勉强可借鉴，但必须重写的部分**

- `SimpleBeamAdvance` / `BeamAdvance` 的“被动波束截获”流程可借鉴，但只能保留处理顺序，不能保留实现。它们把目标真值直接变成结果、并发写共享状态、硬编码阈值，问题太多。见 [ElecReconProcess.cpp:1217](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1217C1)、[ElecReconProcess.cpp:1348](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1348C1)。

- `Advance` 的“对雷达源/通信源分别做截获判定”也只能借鉴大体框架，不能复用实现。它仍然是典型真值驱动过滤器，不是真实接收机链路。见 [ElecReconProcess.cpp:1517](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L1517C1)。

**基本不建议复用的东西**

- `TempTargetArrange(const RADIATEPLATFORMWORKPARA2D&)` 有明显表达式优先级错误，生成网格点本身就可能错。见 [ElecReconProcess.cpp:682](/Users/aurora/Code/1q/doc/electronic_surveillance_radar/ElecReconProcess.cpp#L682C1)。

- 所有 `#pragma omp parallel for` 下直接写成员变量或共享容器的逻辑都不可信，比如 `m_RadiateList[i].beProcessed`、`m_PassiveDetectList.push_back`、`m_ElecReconEquipList[0].m_AnteMBAz` 这类共享状态修改。算法思想可以看，实现不能信。

- 大量硬编码经验值不应继承，包括 `SNR >= 14`、`主瓣外增益 = AnteGain - 35`、`4120 * (sqrt(h1)+sqrt(h2))` 这类规则。它们最多能作为待校准初值，不能当正式算法。

**按提纯价值排序，我建议优先拆这 6 个纯算法/模块**

1. `BandClassifier`：来自 `JudgeWaveType`。
2. `BeamScanPatternGenerator`：来自 `BeamArrange`。
3. `InterceptGate`：几何视场 + 频段覆盖 + 通视联合判定。
4. `ReceiverLinkBudget`：接收功率、干扰功率、SNR 计算。
5. `AngleErrorModel`：来自 `AngleDiff`。
6. `PassiveCoverageBoundarySolver`：来自 `ScopeAdvance2D2/3D` 的二分边界搜索。

一句话概括：这份类实现里，**能用的是局部数值算法和判定顺序，不能用的是对象设计、状态管理、并发实现和输出语义**。

如果你要，我下一步可以直接把这些“可用算法”整理成一个拆解清单，按“可直接迁移 / 需重写 / 应丢弃”三类写成新的设计说明。