---
Status: active
Last-reviewed: 2026-08-03
Authority: EOS 设计权威入口
Answers: EOS 模块是什么、和谁交互、设计文档怎么导航
---

# EOS 设计

`electro_optical_sensor` 模块负责光电传感器的配置、单周期输入、环境/大气建模、红外与可见光探测、
融合输出、trace/replay、调试视图和生命周期事件。对外提供稳定 `EosSession` 门面；foundation 物理
算法、pipeline、controller、环境模型保持 internal。

EOS 的心智模型是**光电探测流水线**：

```
目标几何 / 路径
    ↓
大气与辐射传输（透过率 / 路径辐射惩罚）
    ↓
探测器物理（NEP / 背景噪声 / 成像质量）
    ↓
检测判定（红外 / 可见光 / fused，门限与记录）
```

每周期由 `EosPipeline` 先构造目标无关的 `FrameContext`（光学、环境、噪声），再对每个目标构造
`DetectionComputationContext`（路径传输、空间分辨、杂散光），最后由红外/可见光/融合通道产出检测
记录与仿真归属。foundation 物理算法位于 `src/electro_optical_sensor/foundation`，是内部可测试实现，
不是模块间契约，也不是 public customization surface。

## 文档导航

- 模块级边界、非目标、frame_rate/dt 耦合边界、帧级上下文 config 语义、专项序列边界、设计变更规则
  → [boundaries.md](boundaries.md)
- 分层组件图、Public API 边界、执行时序、主探测数据流、输出与仿真归属数据流、状态所有权
  → [data-flow.md](data-flow.md)
- 算法登记表（配置映射 / 环境因子 / 辐射传输 / 光学几何 / 红外 / 可见光 / 噪声·NEP / 空间频谱 /
  杂散光 / 融合）、每算法的实现边界与反直觉点、刻意不实现的扩展点 → [algorithms.md](algorithms.md)

跨模块公共规则（public API 边界、四域配置、三层输出模型、会话配置直接赋值、运行期配置提交
策略、证据优先开发模式等）见 `docs/common/contract.md`。
