---
Status: active
Last-reviewed: 2026-08-10
Authority: threat_assessment 算法清单与实现边界
Answers: 威胁评估算法怎么实现、边界在哪、反直觉点是什么
---

# Threat Assessment 算法登记

本文是 `threat_assessment` 算法清单与实现边界的权威。逐步逻辑读代码（`src/threat_assessment/`）；模块级边界与非目标详见 [boundaries.md](boundaries.md)，算法选型决策见 `docs/review/threat_assessment_decision_2026-08-10.md`。

## 算法登记表

| 算法/部件 | 意图 / 核心转换 | 实现状态 | 证据与单测 |
|---|---|---|---|
| 归一化加权和（MADM）威胁评估 | 目标属性帧（距离/速度/加速度/RCS/类型/融合置信度） $\to$ 威胁分 $[0,1]$ + 等级 + 贡献分解 | session-wired | [evidence: tests/unit/threat_assessment/threat_evaluator_test.cpp] |

实现状态说明：
- **session-wired**：已接入主评估链路，覆盖配置校验、重放及批量执行。

---

## 核心算法详述

## 1. 归一化加权和威胁评估 (Normalized Weighted-Sum MADM Threat Evaluation)

### 1.1 算法意图与调用时机
- **业务意图**：针对多目标属性帧计算综合威胁分数、等级归类（HIGH / MEDIUM / LOW）及单项属性贡献分解，为火力/传感器分配提供排序决策依据。
- **调用时机与宿主**：由 `ThreatEvaluator::Evaluate` 逐输入帧按批次独立调用。
- **公共/私有归属**：模块私有实现，纯函数式设计。

### 1.2 数学与物理模型
- **输入与坐标系**：
  - 目标属性集合包含 6 维标量：距离 $r$ (m)、速度 $s$ (m/s)、加速度 $a$ (m/s²)、RCS $\sigma$ (m²)、类型概率 $P_{type} \in [0,1]$、融合置信度 $C_{fusion} \ge 0$。
  - 坐标系无关：距离由调用方计算给出（相对受保护资产/平台的斜距）。
- **计算管线（三级处理）**：
  1. **属性归一化**（分段线性映射到 $[0,1]$）：
     - 距离（越近越危险）：$f_r = \text{clamp}\left(\frac{r_{far} - r}{r_{far} - r_{near}}, 0, 1\right)$
     - 速度（越快越危险）：$f_s = \text{clamp}\left(\frac{s - s_{min}}{s_{max} - s_{min}}, 0, 1\right)$
     - 加速度（机动越大越危险）：$f_a = \text{clamp}\left(\frac{a}{a_{max}}, 0, 1\right)$
     - RCS（越大越危险）：$f_\sigma = \text{clamp}\left(\frac{\sigma - \sigma_{min}}{\sigma_{max} - \sigma_{min}}, 0, 1\right)$
     - 类型概率：直通并钳制 $f_p = \text{clamp}(P_{type}, 0, 1)$
     - 融合置信度：直通并钳制 $f_c = \text{clamp}(C_{fusion}, 0, 1)$
  2. **加权综合求和**：
     - 权重非负归一化：$w'_i = \frac{\max(0, w_i)}{\sum \max(0, w_k)}$（若全零或非有限值退化为均等权重 $1/6$）。
     - 综合分计算：$S_{threat} = \text{clamp}\left(\sum_{i=1}^6 w'_i \cdot f_i, 0, 1\right)$。
     - 贡献分解：第 $i$ 项属性的贡献值为 $c_i = w'_i \cdot f_i$。
  3. **等级映射**：
     - $S_{threat} \ge T_{high} \implies \text{HIGH}$
     - 否则若 $S_{threat} \ge T_{medium} \implies \text{MEDIUM}$
     - 其余 $\implies \text{LOW}$

- **关键参数与默认断点表**：
  | 属性 | 默认断点 / 阈值 | 默认权重 $w$ | 物理含义与语义 |
  |---|---|---|---|
  | 距离 `range_m` | near = 20 km $\to 1$, far = 200 km $\to 0$ | 0.30 | 距离 $\le 20\text{ km}$ 满分，$\ge 200\text{ km}$ 零分 |
  | 速度 `speed` | min = 50 m/s $\to 0$, max = 500 m/s $\to 1$ | 0.25 | 超速满分 |
  | 类型概率 `target_probability` | 0 $\to 0$, 1 $\to 1$ | 0.15 | 识别置信度直通 |
  | 加速度 `acceleration` | max = 50 m/s² $\to 1$ | 0.10 | 线性机动响应 |
  | RCS `rcs` | min = 0.5 m² $\to 0$, max = 10 m² $\to 1$ | 0.10 | 与雷达启发式阈值对齐 |
  | 融合置信度 `fusion_confidence`| 0 $\to 0$, 1 $\to 1$ | 0.10 | 先钳制后直通 |

### 1.3 实现边界与工程简化
- **复杂度**：时间复杂度 $\mathcal{O}(N)$（$N$ 为输入目标数），无堆内存频繁分配（预分配输出容器）。
- **计算语义**：严格纯函数设计，无跨周期状态，无成员变量可变状态，天然支持并发重入。
- **确定性保证**：输出列表严格保持输入顺序，同输入同输出。

### 1.4 边界保护与降级策略
- **数值清洗**：负值或 NaN 按属性缺失处理（归一化为 0，贡献记为 0）。
- **断点退化保护**：若配置参数出现断点退化（如 $r_{far} \le r_{near}$ 或 $s_{max} \le s_{min}$），归一化函数取退化端常量，严禁除零或输出 NaN。
- **阈值倒置容错**：若 $T_{high} \le T_{medium}$，MEDIUM 等级永不触发，按配置语义正常流转，不抛出异常。

### 1.5 反直觉点与工程陷阱
> [!IMPORTANT]
> - **属性缺失以 NaN 表达，0 是极端危险合法值**：距离为 0 代表零距离（最危险，归一化得 1.0）；若缺失距离数据必须传入 NaN，若误传 0 会导致误判为最高威胁。
> - **融合置信度可大于 1.0 但威胁分严格有界**：Fusion 模块采用累加置信度公式，其值可大于 1.0；在此处作为威胁特征时先执行 clamp 到 $[0,1]$，再做加权。
> - **权重按相对比例解释而非字面量**：用户配置 `[6, 6, 6, 6, 6, 6]` 等价于全均分；贡献分解中的权重是归一化后的 $w'_i$。
> - **距离语义由调用方定义**：算法内部不强行绑定坐标系或原点，距离由调用方自行计算斜距后输入。

### 1.6 证据链
- **源码入口**：`src/threat_assessment/ThreatEvaluator.cpp`
- **单测覆盖**：[evidence: tests/unit/threat_assessment/threat_evaluator_test.cpp]

---

## 非目标（刻意不实现的算法）

1. **时序滤波与威胁预测**：不维护航迹时序威胁历史，不作动态趋势推演；时序状态由上游航迹滤波层维护。
2. **坐标解算与几何投影**：不承担坐标转换职责，所有几何输入必须预先由上游转换为标量输入。
