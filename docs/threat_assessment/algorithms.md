---
Status: active
Last-reviewed: 2026-08-10
Authority: threat_assessment 算法清单与实现边界
Answers: 威胁评估算法怎么实现、边界在哪、反直觉点是什么
---

# Threat Assessment 算法

## 算法清单

| 算法 | 输入 | 输出 | 证据 |
|---|---|---|---|
| 归一化加权和（MADM）威胁评估 | 目标属性帧（速度/距离/加速度/RCS/类型概率/融合置信度） | 威胁分 [0,1] + 等级 + 每属性贡献 | `src/threat_assessment/ThreatEvaluator.cpp`；`tests/unit/threat_assessment/threat_evaluator_test.cpp` |

## 1. 归一化加权和威胁评估

逐输入独立计算，三级管线：

### 1.1 属性归一化（分段线性 → [0,1]）

| 属性 | 方向 | 归一化函数 | 默认断点 | 语义 |
|---|---|---|---|---|
| 距离 `range_m` | 越近越危险 | `f = clamp((far − r)/(far − near), 0, 1)` | near=20 km → 1；far=200 km → 0 | 距离 ≤ near 满分，≥ far 零分 |
| 速度 `speed` | 越快越危险 | `f = clamp((s − min)/(max − min), 0, 1)` | min=50 m/s → 0；max=500 m/s → 1 | 速度 ≥ max 满分 |
| 加速度 `acceleration` | 机动越大越危险 | `f = clamp(a/max, 0, 1)` | max=50 m/s² | 线性 0→1 |
| RCS `rcs` | 越大越危险 | `f = clamp((rcs − min)/(max − min), 0, 1)` | min=0.5 m² → 0；max=10 m² → 1 | 与既有 AR 启发式阈值（3 m²）同量级 |
| 类型概率 `target_probability` | 越大越危险 | 直通钳制 [0,1] | — | 识别置信度直通 |
| 融合置信度 `fusion_confidence` | 证据越强越危险 | 钳制 [0,1]（**先钳制后直通**） | — | fusion 冻结公式可 >1 |
| 辐射源威胁证据 `emitter_threat_evidence` | 越大越危险 | 直通钳制 [0,1] | —（默认权重 0） | 调用方组装的 ESM 证据（如模式基准 × 假设置信度）；TARGET-OQ-2 处置引入，默认不参与计分（opt-in） |

属性清洗：负值/NaN 按属性缺失处理（归一化 0，贡献为 0）。断点退化（如
`range_far ≤ range_near`）时归一化函数取退化端常量（不除零、不 NaN）。

### 1.2 加权求和

- **权重按相对值解释**：内部钳制非负后归一化（Σ 权重 = 1）；全零/非有限权重
  退化为均分（1/7，配置错误防呆）。
- 默认权重：距离 0.30 / 速度 0.25 / 类型概率 0.15 / 加速度 0.10 / RCS 0.10 /
  融合置信度 0.10 / 辐射源威胁证据 0.0（证据属性 opt-in，默认不参与计分）。
- `threat_score = Σ w_i · f_i`，钳制 [0,1] 兜底。

### 1.3 等级映射

- `threat_score ≥ high_threshold` → HIGH；`≥ medium_threshold` → MEDIUM；其余 LOW。
- 默认阈值：HIGH ≥ 0.70 / MEDIUM ≥ 0.40。

### 实现边界

- 复杂度 **O(n)**（n = 输入目标数），无动态分配依赖（`std::vector` 输出预分配）。
- 输出顺序与输入一致；同输入同输出（确定性，可复现可测试）。
- **纯函数式**：无跨周期状态、无成员可变状态，`Evaluate` 可重入可并发。

### 反直觉点

- **融合置信度可 >1 但威胁分必有界**：fusion 置信度是冻结的精确求和公式
  （`docs/fusion/algorithms.md`），作为威胁属性时先钳制 [0,1]——"证据强度"
  语义，不是概率；威胁分上界由权重归一化保证（Σw=1），钳制仅为兜底。
- **权重按相对值解释而非字面值**：配置时无需保证和为 1（如全设 6.0 等价于均分）；
  因此"权重 × 归一化值 = 贡献"等式中的权重是**归一化后**的值。
- **等级阈值倒置不报错**：若 `high_threshold ≤ medium_threshold`，MEDIUM 永不触发
  （先判 HIGH），属配置语义而非缺陷。
- **属性缺失以 NaN/负值表达，0 是合法属性值**：距离 0 = 零距离满分（最危险），
  缺失距离必须传 NaN 而非 0（否则误判为最高威胁）；inf 按超界钳制
  （≥ 上断点 → 满分）。缺失与"真的为 0"在贡献分解中不可区分（都记 0），
  缺失标记属调用方职责。
- **距离语义是调用方给的**：算法不假设坐标帧与参考点，"距离"是目标到受保护
  资产/平台的斜距——威胁评估的威胁对象由调用方定义。

## 非目标（刻意不实现的算法）

见 [boundaries.md](boundaries.md) 非目标清单；选型对比表见决策记录
`docs/review/threat_assessment_decision_2026-08-10.md` §4。
