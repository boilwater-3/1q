# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

## 当前状态（2026-07-20 实时代码复核）

原 OQ-1、OQ-3、OQ-8、OQ-9、OQ-10a 至 OQ-10m 均已完成、拒绝或冻结；对应结论和测试证据已迁入
`docs/common/contract.md` 及各模块 `design.md`。当前新增四项 SBIRS 非阻塞仿真边界，均不改变已经冻结的
三种互斥跟踪模式，也不构成本批提交的实现要求。

## SBIRS 非阻塞仿真边界

### SBIRS-OQ-1：诊断距离的物理语义

- **现状证据**：`SbirsDetectionAttributionRecord.estimated_range_m` 明确只属于 cue/诊断层，不代表被动红外
  测距能力；Strict/Estimated 当前使用真值距离，Sensor-like 使用真值距离叠加比例误差。
- **未决问题**：字段名称和三模式取值来源是否足以防止调用方把它误解为正式传感器测距输出。
- **当前边界**：不得进入 `SbirsOutputFrame` raw output；消费方只能把它当作仿真归属与诊断辅助量。
- **Stage A 进入条件**：出现真实下游消费者需要区分 truth-derived、filter-derived 或不可用距离，先盘点消费路径，
  再评估重命名、增加来源枚举或显式有效性字段。

### SBIRS-OQ-2：WFOV、Estimated 与 Sensor-like 的分阶段误差统计

- **现状证据**：三条用途使用独立随机子流，但共同读取 `SbirsErrorModelConfig` 的同一组角度/距离统计参数。
- **未决问题**：是否需要分别表达 WFOV 搜索、Estimated 校正量测和 NFOV Sensor-like 输出的精度等级。
- **当前边界**：共享参数是当前确定性简化，不得宣称代表真实 WFOV/NFOV 载荷精度差异。
- **Stage A 进入条件**：取得可追溯的分阶段参数依据，或构造出共享参数无法满足的 SBIRS 场景验收矩阵后，
  再评估拆分配置；不得仅为形式完整扩大 public API。

### SBIRS-OQ-3：多目标随机样本与 scene 输入顺序

- **现状证据**：WFOV、Estimated、Sensor-like 各自是一条全局用途随机流；同一 trace 可确定性 replay，
  但多目标在同一周期获得哪个样本取决于 `scene` 遍历顺序。
- **未决问题**：SBIRS 是否需要保证目标列表置换后，每个 `target_id` 仍获得相同的量测随机序列。
- **当前边界**：replay 只保证相同输入字节和顺序的确定性，不承诺 scene permutation invariance。
- **Stage A 进入条件**：外部场景源无法稳定排序，或批量验证明确要求按 target 不受输入顺序影响时，比较
  按 target/channel 派生子流与现有全局用途子流的 snapshot、热更和目标生命周期成本。

### SBIRS-OQ-4：Estimated 航迹的真值初始化

- **现状证据**：Estimated 首次捕获后用输入场景真值 ECEF 位置和速度初始化滤波均值，后续才使用带误差角度量测。
- **未决问题**：是否需要改为仅由被动角度 cue 和显式距离/运动先验初始化，以形成无真值航迹起始链。
- **当前边界**：`Estimated` 是生产仿真链，但当前仍包含 truth-seeded track initiation 简化，不得描述为完全
  无真值辅助的真实载荷跟踪器。
- **Stage A 进入条件**：先定义被动角度不可观测距离的初始化先验、收敛时间和失败判据，并提供与当前方案的
  捕获率、位置协方差、丢锁率及 replay 对比证据，再决定是否替换。
