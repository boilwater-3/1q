---
Status: active
Last-reviewed: 2026-08-05
Authority: fusion 设计权威入口
Answers: fusion 是什么、关联键策略为何冻结为纯库内身份、设计文档怎么导航
---

# Fusion 设计

`fusion` 是**多源关联 + 置信度融合算法面**（行为组件层的两个跨业务可复用算法面之一，
决策记录见 `docs/review/Bahavior.md` §4）。它把异构探测记录（泛型：位置/方位/特征向量/
判决值/质量/库内身份键）聚合为融合目标态势（各源探测状态、融合置信度、各通道量测）。

心智模型：**探测 → 航迹**。每周期把一批探测记录交给 `FusionEngine::Update`，
取回当前全部航迹的融合态势；引擎保持增量航迹状态与滑窗，算法不感知 ESR/EOS/AR
具体类型，由业务层适配。

## 关键定位

- **关联键策略冻结**（`docs/review/Bahavior.md` §4.1）：无外部身份通道，纯库内身份键
  + 特征相似度门限 + 空间门限，守去真值化纪律。
- 空间门限分层：带位置记录 → nanoflann KD-tree 半径搜索（先例：ESR
  `KdTreeClusterer` 内部 nanoflann 用法）；仅方位记录 → `src/common/geometry/
  BearingCluster.h` 方位相干门限。
- 置信度 = Σ 判决值(0/1) × 质量归一化 × 权重（滑窗内**精确求和、不归一化**，随证据
  单调累积——冻结公式，见 [algorithms.md](algorithms.md) 反直觉点）。
- 纯算法、无 Session 三元组、无构建门、不引入新依赖（仅 nanoflann，已在依赖清单）。

## 文档导航

- 模块边界、非目标、关联键与 SAR 输入约束、设计变更规则 → [boundaries.md](boundaries.md)
- 算法清单（关联分层、置信度融合、滑窗与失跟）、实现边界与反直觉点 → [algorithms.md](algorithms.md)

注：fusion 不含 data-flow.md——它是状态机式算法面（每周期 `Update(detections, cycle) →
targets`），没有数据流管线，架构图内聚在下方。

## 架构分层

```mermaid
flowchart TB
  subgraph Public["Public API：include/1q/fusion"]
    Engine["FusionEngine\n融合引擎（PIMPL，状态持有）"]
    Detection["DetectionRecord\n泛型探测记录"]
    Config["FusionConfig\n门限 / 权重 / 滑窗"]
    Target["FusedTarget\n融合态势输出"]
  end

  subgraph Implementation["src/fusion"]
    Keyed["身份键直挂\n同键归并（调用方保证跨源一致）"]
    KdTree["位置关联\nnanoflann KD-tree 半径搜索"]
    Bearing["方位关联\nBearingCluster 相干门限"]
    Feature["特征门限\n欧氏距离"]
    Window["滑窗 / 失跟删除\n每源 deque + missed_cycles"]
    Conf["置信度融合\nΣ 判决值 × 质量 × 权重"]
  end

  Engine --> Keyed
  Engine --> KdTree
  Engine --> Bearing
  Engine --> Feature
  Engine --> Window
  Window --> Conf
  Detection --> Engine
  Config --> Engine
  Engine --> Target
```

读图方式：
1. 新调用方只依赖 `FusionEngine` 与三个公共值类型（聚合入口 `fusion.hpp`）。
2. 关联是**分层**的：身份键优先，其次空间（位置/方位），特征门限作为最终约束。
3. 航迹状态、滑窗与合成键管理在 `src/` 内部，PIMPL 隔离，公共头不暴露 nanoflann。

跨模块公共规则见 `docs/common/contract.md`（fusion 不参与三层输出模型与传感器周期语义）。
